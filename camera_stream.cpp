/*
  Derived from the Freenove 4WD Smart Car Kit camera streaming implementation.

  Modifications:
  - Camera configuration tuning
  - XCLK frequency adjustment for improved stream stability
  - Minor changes supporting autonomous patrol operation

  Modified by: Sankaran Iyer
*/
#include "camera_stream.h"
#include "esp_camera.h"
#include <WiFi.h>
#include "esp_http_server.h"
#include <ESPmDNS.h>
// Set to 1 for home/fixed network use
// Set to 0 for hotspot/demo use
#define USE_STATIC_IP 0
// Add these globally (top of file, outside function)
IPAddress local_IP(192, 168, 1, 50);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);


// =========================
// WiFi credentials
// =========================
const char* WIFI_SSID = "YOUR_WIFI_SSID";
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";
extern void handleControlCmd(String cmd);
// =========================
// Camera pin config (WROVER KIT)
// =========================
#define PWDN_GPIO_NUM     -1
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      21
#define SIOD_GPIO_NUM      26
#define SIOC_GPIO_NUM      27

#define Y9_GPIO_NUM        35
#define Y8_GPIO_NUM        34
#define Y7_GPIO_NUM        39
#define Y6_GPIO_NUM        36
#define Y5_GPIO_NUM        19
#define Y4_GPIO_NUM        18
#define Y3_GPIO_NUM         5
#define Y2_GPIO_NUM         4
#define VSYNC_GPIO_NUM     25
#define HREF_GPIO_NUM      23
#define PCLK_GPIO_NUM      22
extern long g_distanceCm;
extern String GetRobotStateJson();

static httpd_handle_t camera_httpd = NULL;
static httpd_handle_t stream_httpd = NULL;

static const char* INDEX_HTML =
"<!doctype html><html><head><title>ESP32 Camera</title></head>"
"<body><h2>ESP32 Camera Stream</h2>"
"<img id='cam' width='640' height='480' style='border:1px solid black;'/>"
"<script>"
"document.getElementById('cam').src = 'http://' + location.hostname + ':81/stream';"
"</script>"
"</body></html>";

static esp_err_t index_handler(httpd_req_t *req)
{
  Serial.println("INDEX page requested");
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, INDEX_HTML, HTTPD_RESP_USE_STRLEN);
}
static esp_err_t distance_handler(httpd_req_t *req)
{
  char json[64];
  snprintf(json, sizeof(json), "{\"distance_cm\":%ld}", g_distanceCm);
  httpd_resp_set_type(req, "application/json");
  return httpd_resp_send(req, json, HTTPD_RESP_USE_STRLEN);
}
static esp_err_t state_handler(httpd_req_t *req)
{
  String json = GetRobotStateJson();

  httpd_resp_set_type(req, "application/json");
  return httpd_resp_send(req, json.c_str(), json.length());
}
static esp_err_t stream_handler(httpd_req_t *req) {
  camera_fb_t * fb = NULL;
  esp_err_t res = ESP_OK;
  char part_buf[64];

  res = httpd_resp_set_type(req, "multipart/x-mixed-replace;boundary=frame");
  if (res != ESP_OK) return res;

  while (true) {
    fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Camera capture failed");
      res = ESP_FAIL;
      break;
    }

    int hlen = snprintf(part_buf, sizeof(part_buf),
                        "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n",
                        (unsigned int)fb->len);

    res = httpd_resp_send_chunk(req, part_buf, hlen);
    if (res != ESP_OK) {
      esp_camera_fb_return(fb);
      Serial.println("Stream header send failed");
      break;
    }

    res = httpd_resp_send_chunk(req, (const char *)fb->buf, fb->len);
    if (res != ESP_OK) {
      esp_camera_fb_return(fb);
      Serial.println("Stream frame send failed");
      break;
    }

    res = httpd_resp_send_chunk(req, "\r\n", 2);
    esp_camera_fb_return(fb);

    if (res != ESP_OK) {
      Serial.println("Stream boundary send failed");
      break;
    }

    vTaskDelay(10 / portTICK_PERIOD_MS);
  }

  return res;
}

static esp_err_t control_handler(httpd_req_t *req)
{
  char query[64];
  char value[32];

  if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
    if (httpd_query_key_value(query, "cmd", value, sizeof(value)) == ESP_OK) {
      Serial.print("HTTP cmd received: ");
      Serial.println(value);
      handleControlCmd(String(value));
      httpd_resp_set_type(req, "text/plain");
      return httpd_resp_send(req, "OK", HTTPD_RESP_USE_STRLEN);
    }
  }

  httpd_resp_set_status(req, "400 Bad Request");
  return httpd_resp_send(req, "Missing cmd", HTTPD_RESP_USE_STRLEN);
}
static void startCameraServer()
{
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;
  config.ctrl_port = 32768;

  httpd_config_t config_stream = HTTPD_DEFAULT_CONFIG();
  config_stream.server_port = 81;
  config_stream.ctrl_port = 32769;

  httpd_uri_t index_uri = {
    .uri      = "/",
    .method   = HTTP_GET,
    .handler  = index_handler,
    .user_ctx = NULL
  };

  httpd_uri_t control_uri = {
    .uri      = "/control",
    .method   = HTTP_GET,
    .handler  = control_handler,
    .user_ctx = NULL
  };

  httpd_uri_t stream_uri = {
    .uri      = "/stream",
    .method   = HTTP_GET,
    .handler  = stream_handler,
    .user_ctx = NULL
  };

  httpd_uri_t distance_uri = {
  .uri      = "/distance",
  .method   = HTTP_GET,
  .handler  = distance_handler,
  .user_ctx = NULL
  };

  httpd_uri_t state_uri = {
  .uri      = "/state",
  .method   = HTTP_GET,
  .handler  = state_handler,
  .user_ctx = NULL
  };

  if (httpd_start(&camera_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(camera_httpd, &index_uri);
    httpd_register_uri_handler(camera_httpd, &control_uri);
    httpd_register_uri_handler(camera_httpd, &distance_uri);
    httpd_register_uri_handler(camera_httpd, &state_uri);
    Serial.println("Main server started on port 80");
  } else {
    Serial.println("Failed to start main HTTP server");
  }

  if (httpd_start(&stream_httpd, &config_stream) == ESP_OK) {
    httpd_register_uri_handler(stream_httpd, &stream_uri);
    Serial.println("Stream server started on port 81");
  } else {
    Serial.println("Failed to start stream HTTP server");
  }
}
void CameraStream_Init() {
  Serial.println("Initializing camera...");

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
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 8000000;
  config.pixel_format = PIXFORMAT_JPEG;

  if (psramFound()) {
    config.frame_size = FRAMESIZE_QVGA;
    config.jpeg_quality = 15;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_QQVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }
   esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed: 0x%x\n", err);
    return;
  }
  sensor_t * s = esp_camera_sensor_get();

  if (s) {
    s->set_vflip(s, 1);   // Flip vertically
    s->set_hmirror(s, 0); // Mirror horizontally (optional)
  }

  Serial.println("Connecting to WiFi...");
  WiFi.mode(WIFI_STA);

#if USE_STATIC_IP
  Serial.println("Using static IP configuration...");
  if (!WiFi.config(local_IP, gateway, subnet)) {
    Serial.println("Static IP configuration failed");
  }
#else
  Serial.println("Using DHCP...");
#endif

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println();
  Serial.println("WiFi connected");
  Serial.print("Open: http://");
  Serial.println(WiFi.localIP());
  if (MDNS.begin("esp32robot")) {
    Serial.println("mDNS started");
    Serial.println("Open: http://esp32robot.local");
} else {
  Serial.println("Error starting mDNS");
}

  startCameraServer();
}