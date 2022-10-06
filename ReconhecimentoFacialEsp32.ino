/*******************************************************************************
 * CANAL INTERNET E COISAS                                                     *
 * ESP32-CAM                                                                   *
 * Controle de Acesso com Reconhecimento Facial e integração ao Jarvis         *
 * Baseado no Projeto de Andrew McCabe, uso autorizado                         *
 * https://robotzero.one/access-control-with-face-recognition/                 *
 * 04/2020 - Andre Michelon                                                    *
 * andremichelon@internetecoisas.com.br                                        *
 * https://internetecoisas.com.br                                              *
 ******************************************************************************/

// Bibliotecas ------------------------------------------
#include <ArduinoWebsockets.h>
#include <SPIFFS.h>
#include <HTTPClient.h>
#include "esp_http_server.h"
#include "esp_timer.h"
#include "esp_camera.h"
#include "Arduino.h"
#include "fd_forward.h"
#include "fr_forward.h"
#include "fr_flash.h"

// Definições -------------------------------------------
#define ENROLL_CONFIRM_TIMES  5
#define FACE_ID_SAVE_NUMBER   7
#define MIN_DETECT_INTERVAL   5000

// Parâmetros da ESP32-CAM
#define PWDN_GPIO_NUM         32
#define RESET_GPIO_NUM        -1
#define XCLK_GPIO_NUM         0
#define SIOD_GPIO_NUM         26
#define SIOC_GPIO_NUM         27
#define Y9_GPIO_NUM           35
#define Y8_GPIO_NUM           34
#define Y7_GPIO_NUM           39
#define Y6_GPIO_NUM           36
#define Y5_GPIO_NUM           21
#define Y4_GPIO_NUM           19
#define Y3_GPIO_NUM           18
#define Y2_GPIO_NUM           5
#define VSYNC_GPIO_NUM        25
#define HREF_GPIO_NUM         23
#define PCLK_GPIO_NUM         22

// Inicializa instancias --------------------------------
using namespace   websockets;
WebsocketsServer  socket_server;
HTTPClient        httpClient;

// Estruturas -------------------------------------------
typedef struct {
  uint8_t *image;
  box_array_t *net_boxes;
  dl_matrix3d_t *face_id;
} http_img_process_result;

static inline mtmn_config_t app_mtmn_config() {
  mtmn_config_t mtmn_config = {0};
  mtmn_config.type = FAST;
  mtmn_config.min_face = 80;
  mtmn_config.pyramid = 0.707;
  mtmn_config.pyramid_times = 4;
  mtmn_config.p_threshold.score = 0.6;
  mtmn_config.p_threshold.nms = 0.7;
  mtmn_config.p_threshold.candidate_number = 20;
  mtmn_config.r_threshold.score = 0.7;
  mtmn_config.r_threshold.nms = 0.7;
  mtmn_config.r_threshold.candidate_number = 10;
  mtmn_config.o_threshold.score = 0.7;
  mtmn_config.o_threshold.nms = 0.7;
  mtmn_config.o_threshold.candidate_number = 1;
  return mtmn_config;
}

typedef enum {
  START_STREAM,
  START_DETECT,
  SHOW_FACES,
  START_RECOGNITION,
  START_ENROLL,
  ENROLL_COMPLETE,
  DELETE_ALL,
} en_fsm_state;

typedef struct {
  char enroll_name[ENROLL_NAME_LEN];
} httpd_resp_value;

// Wi-Fi ------------------------------------------------
const char*           SSID                    = "home2.4";
const char*           PASSWORD                = "Home31415";

// API JARVIS -------------------------------------------
const String          JARVIS_API              = "http://192.168.0.31";

// Variáveis globais ------------------------------------
camera_fb_t           *fb                     = NULL;
static dl_matrix3du_t *aligned_face           = NULL;
httpd_handle_t        camera_httpd            = NULL;
mtmn_config_t         mtmn_config             = app_mtmn_config();
unsigned long         last_detected_millis    = 0;
unsigned long         last_recognition_millis = 0;
face_id_name_list     st_face_list;
en_fsm_state          g_state;
httpd_resp_value      st_name;

// Funções auxiliares -----------------------------------
void app_facenet_main();

void app_httpserver_init();

static esp_err_t index_handler(httpd_req_t *req) {
  File file = SPIFFS.open("/Home.htm", "r");
  String s;
  if (file) {
    s = file.readString();
    file.close();
    Serial.println("Home - ok");
  } else {
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_set_status(req, "500 Erro");
    s = "500 - Falha lendo Home";
    Serial.println("Home - erro");
  }
  return httpd_resp_send(req, s.c_str(), s.length());
}

httpd_uri_t index_uri = {
  .uri       = "/",
  .method    = HTTP_GET,
  .handler   = index_handler,
  .user_ctx  = NULL
};

void app_httpserver_init () {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  if (httpd_start(&camera_httpd, &config) == ESP_OK) {
    Serial.println("Inicializando");
    httpd_register_uri_handler(camera_httpd, &index_uri);
  }
}

void app_facenet_main() {
  face_id_name_init(&st_face_list, FACE_ID_SAVE_NUMBER, ENROLL_CONFIRM_TIMES);
  aligned_face = dl_matrix3du_alloc(1, FACE_WIDTH, FACE_HEIGHT, 3);
  read_face_id_from_flash_with_name(&st_face_list);
}

static inline int do_enrollment(face_id_name_list *face_list, dl_matrix3d_t *new_id) {
  int left_sample_face = enroll_face_id_to_flash_with_name(face_list, new_id, st_name.enroll_name);
  Serial.printf("Usuário: %s Amostra %d\n", st_name.enroll_name, ENROLL_CONFIRM_TIMES - left_sample_face);
  return left_sample_face;
}

static esp_err_t send_face_list(WebsocketsClient &client) {
  client.send("delete_faces");
  face_id_node *head = st_face_list.head;
  char add_face[64];
  for (int i = 0; i < st_face_list.count; i++) {
    sprintf(add_face, "listface:%s", head->id_name);
    client.send(add_face);
    head = head->next;
  }
}

static esp_err_t delete_all_faces(WebsocketsClient &client) {
  delete_face_all_in_flash_with_name(&st_face_list);
  client.send("delete_faces");
}

void handle_message(WebsocketsClient &client, WebsocketsMessage msg) {
  if (msg.data() == "stream") {
    g_state = START_STREAM;
    client.send("Vídeo ao Vivo");
  }
  if (msg.data() == "detect") {
    g_state = START_DETECT;
    client.send("Detectando");
  }
  if (msg.data().substring(0, 8) == "capture:") {
    g_state = START_ENROLL;
    char person[FACE_ID_SAVE_NUMBER * ENROLL_NAME_LEN] = {0,};
    msg.data().substring(8).toCharArray(person, sizeof(person));
    memcpy(st_name.enroll_name, person, strlen(person) + 1);
    client.send("Capturando");
  }
  if (msg.data() == "recognise") {
    g_state = START_RECOGNITION;
    client.send("Reconhecendo");
  }
  if (msg.data().substring(0, 7) == "remove:") {
    char person[ENROLL_NAME_LEN * FACE_ID_SAVE_NUMBER];
    msg.data().substring(7).toCharArray(person, sizeof(person));
    delete_face_id_in_flash_with_name(&st_face_list, person);
    send_face_list(client);
  }
  if (msg.data() == "delete_all") {
    delete_all_faces(client);
  }
}

// Setup ------------------------------------------------
void setup() {
  Serial.begin(115200);
  Serial.println();

  if (!SPIFFS.begin()) {
    Serial.println("SPIFFS - Falha");
    while(true);
  }

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  if (psramFound()) {
    Serial.println("PSRAM identificada");
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    Serial.println("PSRAM não identificada");
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Falha inicializando a Câmera, erro 0x%x", err);
    while(true);
  }

  sensor_t * s = esp_camera_sensor_get();
  s->set_framesize(s, FRAMESIZE_QVGA);

  WiFi.begin(SSID, PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi conectado");

  app_httpserver_init();
  app_facenet_main();
  socket_server.listen(82);

  httpClient.begin(JARVIS_API + "/?acessook&ninguem");
  if (httpClient.GET() != HTTP_CODE_OK) {
    Serial.println("API Jarvis indisponível.");
    while(true);
  }
  httpClient.end();

  Serial.print("Câmera pronta, acesse http://");
  Serial.println(WiFi.localIP());
}

// Loop -------------------------------------------------
void loop() {
  auto client = socket_server.accept();
  client.onMessage(handle_message);
  dl_matrix3du_t *image_matrix = dl_matrix3du_alloc(1, 320, 240, 3);
  http_img_process_result out_res = {0};
  out_res.image = image_matrix->item;

  send_face_list(client);
  client.send("Vídeo ao Vivo");

  while (client.available()) {
    client.poll();
    fb = esp_camera_fb_get();
    if (g_state == START_DETECT || g_state == START_ENROLL || g_state == START_RECOGNITION) {
      out_res.net_boxes = NULL;
      out_res.face_id = NULL;
      fmt2rgb888(fb->buf, fb->len, fb->format, out_res.image);
      out_res.net_boxes = face_detect(image_matrix, &mtmn_config);

      if (out_res.net_boxes) {
        if (align_face(out_res.net_boxes, image_matrix, aligned_face) == ESP_OK) {
          out_res.face_id = get_face_id(aligned_face);
          last_detected_millis = millis();
          if (g_state == START_DETECT) {
            client.send("Face detectada");
          }

          if (g_state == START_ENROLL) {
            int left_sample_face = do_enrollment(&st_face_list, out_res.face_id);
            char enrolling_message[64];
            sprintf(enrolling_message, "Amostra #%d para %s", ENROLL_CONFIRM_TIMES - left_sample_face, st_name.enroll_name);
            client.send(enrolling_message);
            if (left_sample_face == 0) {
              g_state = START_STREAM;
              client.send("Face capturada para " + String(st_face_list.tail->id_name));
              send_face_list(client);
            }
          }

          if (g_state == START_RECOGNITION &&
                st_face_list.count > 0 &&
                millis() - last_recognition_millis > MIN_DETECT_INTERVAL) {
            face_id_node *f = recognize_face_with_name(&st_face_list, out_res.face_id);
            if (f) {
              String s = f->id_name;
              Serial.println("Reconhecido: " + s);
              httpClient.begin(JARVIS_API + "/?acessook&" + s);
              Serial.println(httpClient.GET() == HTTP_CODE_OK ? "Notificação enviada" : "Falha na notificação");
              httpClient.end();
              client.send("Usuário identificado: " + s);
            } else {
              Serial.println("Não reconhecido");
              httpClient.begin(JARVIS_API + "/?acessonok");
              Serial.println(httpClient.GET() == HTTP_CODE_OK ? "Notificação enviada" : "Falha na notificação");
              httpClient.end();
              client.send("Usuário não identificado");
            }
            last_recognition_millis = millis();
          }
          dl_matrix3d_free(out_res.face_id);
        }
      } else {
        if (g_state != START_DETECT) {
          client.send("Face não detectada");
        }
      }

      if (g_state == START_DETECT && millis() - last_detected_millis > 500) {
        client.send("Detectando");
      }
    }
    client.sendBinary((const char *)fb->buf, fb->len);
    esp_camera_fb_return(fb);
    fb = NULL;
  }
}
