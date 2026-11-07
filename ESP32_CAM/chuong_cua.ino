#include <WiFi.h>
#include <WiFiClientSecure.h>
#include "esp_camera.h"
#include <UniversalTelegramBot.h>
#include <ArduinoJson.h>
#include "esp_http_server.h"


const char* ssid     = "leqin";
const char* password = "88888888";

#define BOTtoken "8671922360:AAH7fX-BKFq8Ybm7cEQUNa3Q9vh9q_U4G0Q"
#define CHAT_ID  "7266554264"


#define BUTTON_PIN    12
#define BUZZER_PIN    2
#define FLASH_LED_PIN 4


#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

WiFiClientSecure clientTCP;
UniversalTelegramBot bot(BOTtoken, clientTCP);

httpd_handle_t stream_httpd = NULL;

unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 3000;

#define PART_BOUNDARY "123456789000000000000987654321"
static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";


static esp_err_t stream_handler(httpd_req_t *req) {
  camera_fb_t * fb = NULL;
  esp_err_t res = ESP_OK;
  size_t _jpg_buf_len = 0;
  uint8_t * _jpg_buf = NULL;
  char part_buf[64];

  res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
  if (res != ESP_OK) return res;

  while (true) {
    fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Loi lay khung hinh Stream");
      res = ESP_FAIL;
    } else {
      _jpg_buf_len = fb->len;
      _jpg_buf = fb->buf;
    }
    if (res == ESP_OK) {
      size_t hlen = snprintf(part_buf, 64, _STREAM_PART, _jpg_buf_len);
      res = httpd_resp_send_chunk(req, part_buf, hlen);
    }
    if (res == ESP_OK) {
      res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
    }
    if (res == ESP_OK) {
      res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
    }
    if (fb) {
      esp_camera_fb_return(fb);
      fb = NULL;
      _jpg_buf = NULL;
    } else if (res != ESP_OK) {
      break;
    }
  }
  return res;
}

void startCameraServer() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;

  httpd_uri_t stream_uri = {
    .uri       = "/",
    .method    = HTTP_METHOD_GET,
    .handler   = stream_handler,
    .user_ctx  = NULL
  };

  if (httpd_start(&stream_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(stream_httpd, &stream_uri);
  }
}


void configCamera() {
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
    config.frame_size = FRAMESIZE_VGA;
    config.jpeg_quality = 12;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_CIF;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Loi khoi tao Camera: 0x%x\n", err);
    ESP.restart();
  }

  sensor_t * s = esp_camera_sensor_get();
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1);
    s->set_hmirror(s, 0);
    s->set_brightness(s, 1);
  }
}


void sendPhotoToTelegram() {
  digitalWrite(BUZZER_PIN, HIGH); delay(100);
  digitalWrite(BUZZER_PIN, LOW);  delay(50);
  digitalWrite(BUZZER_PIN, HIGH); delay(100);
  digitalWrite(BUZZER_PIN, LOW);

  digitalWrite(FLASH_LED_PIN, HIGH);
  delay(150);

  camera_fb_t * fb = esp_camera_fb_get();
  digitalWrite(FLASH_LED_PIN, LOW);

  if (!fb) {
    Serial.println("Chup anh that bai!");
    bot.sendMessage(CHAT_ID, "⚠️ Co nguoi bam chuong nhung Camera chup anh that bai!", "");
    return;
  }

  bot.sendMessage(CHAT_ID, "🔔 **Có người bấm chuông!**", "Markdown");

  Serial.println("Dang ket noi den Telegram server...");
  if (clientTCP.connect("api.telegram.org", 443)) {
    String head = "--ESP32CAM\r\nContent-Disposition: form-data; name=\"chat_id\"\r\n\r\n" + String(CHAT_ID) + \
                  "\r\n--ESP32CAM\r\nContent-Disposition: form-data; name=\"photo\"; filename=\"esp32-cam.jpg\"\r\nContent-Type: image/jpeg\r\n\r\n";
    String tail = "\r\n--ESP32CAM--\r\n";

    size_t totalLen = fb->len + head.length() + tail.length();

    clientTCP.println("POST /bot" + String(BOTtoken) + "/sendPhoto HTTP/1.1");
    clientTCP.println("Host: api.telegram.org");
    clientTCP.println("Content-Length: " + String(totalLen));
    clientTCP.println("Content-Type: multipart/form-data; boundary=ESP32CAM");
    clientTCP.println();
    clientTCP.print(head);

    uint8_t *fbBuf = fb->buf;
    size_t fbLen = fb->len;
    for (size_t n = 0; n < fbLen; n += 1024) {
      if (n + 1024 < fbLen) {
        clientTCP.write(fbBuf + n, 1024);
      } else {
        clientTCP.write(fbBuf + n, fbLen - n);
      }
    }

    clientTCP.print(tail);
    esp_camera_fb_return(fb);

    while (clientTCP.connected()) {
      String line = clientTCP.readStringUntil('\n');
      if (line == "\r") break;
    }
    clientTCP.stop();
    Serial.println("Gui anh thanh cong!");
  } else {
    esp_camera_fb_return(fb);
    Serial.println("Ket noi toi Telegram server that bai!");
  }
}


void setup() {
  Serial.begin(115200);

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(FLASH_LED_PIN, OUTPUT);

  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(FLASH_LED_PIN, LOW);

  configCamera();

  WiFi.begin(ssid, password);
  clientTCP.setInsecure();

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWi-Fi da ket noi thanh cong!");


  startCameraServer();

  String ipAddr = WiFi.localIP().toString();
  Serial.print("Dia chi xem Camera trong mang LAN: http://");
  Serial.println(ipAddr);

  digitalWrite(BUZZER_PIN, HIGH); delay(300); digitalWrite(BUZZER_PIN, LOW);

  String msg = "✅ **Chuông cửa đã sẵn sàng!**\n🔗 Xem Cam nội bộ: http://" + ipAddr;
  bot.sendMessage(CHAT_ID, msg, "Markdown");
}


void loop() {
  if (digitalRead(BUTTON_PIN) == LOW) {
    if (millis() - lastDebounceTime > debounceDelay) {
      Serial.println("Phat hien bam chuong!");
      sendPhotoToTelegram();
      lastDebounceTime = millis();
    }
  }
}
