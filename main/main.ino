/*
  ESP32 Matchboxscoe Camera Software
  inspired by:
  https://www.hackster.io/KDPA/esp32-cam-video-surveillance-robot-a22367
*/
// Some URLS for controlling e.g. the light intensity in the browser
// http://192.168.2.168/control?var=flash&val=100
// http://192.168.1.4/controll?var=lens&val=100

/*
  HEADERS
*/

// ESP-IDF headers (in alphabetical order!)
#include <esp_log.h>
#include <esp_camera.h>
#include <esp_http_server.h>
#include <esp_timer.h>
#include <esp_wifi.h>
#include <img_converters.h>
#include <soc/soc.h>
#include <soc/rtc_cntl_reg.h>

// Arduino ESP32 headers (in alphabetical order!)
#include <Arduino.h>
#include <esp32-hal-ledc.h>
#include <FS.h>
#include <HTTPClient.h>
#include <SD_MMC.h>
#include <SPIFFS.h>
#include <Update.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>

// Third-party library headers  (in alphabetical order!)
#include "ArduinoJson.h" // https://github.com/bblanchon/ArduinoJson
#include "ESP32Ping.h"   // https://github.com/marian-craciunescu/ESP32Ping
#include "WiFiManager.h" // https://github.com/tzapu/WiFiManager

// Local header files (in alphabetical order!)
#include "Base64.h"
#include "camera.h"
// #include "cameraM5Stack.h"
#include "device_pref.h"
#include "html.h"

/*
  USER-CONFIGURED SETTINGS
*/

// declare functions
void saveCapturedImageGDrive(void);
void setLens(int lensValue);
void setLED(int ledIntensity);
void setFrameSize(int val);

// Anglerfish

// FIXME: We mostly need to differentiate between Matchboxcope/Anglerfish, where Anglerfish has the "dangerzone" aka: deepsleep that will never wake up
const int focuStackStepsizeAnglerfish = 25; // FIXME: This value should be adjustable through the GUI

// Wifi

const boolean hostWifiAP = true;       // set this variable if you want the ESP32 to be the host
const boolean isCaptivePortal = false; // want to autoconnect to wifi networks?
const char *mSSID = "BenMur";
const char *mPASSWORD = "MurBen3128";
const char *mSSIDAP = "Matchboxscope";
const char *hostname = "matchboxscope";

// Google Drive Upload

const char *myDomain = "script.google.com";
const String myScript = "/macros/s/AKfycbwF8y5az641P2EUkooJjpEVn36Bpu2nAxYpQ8WOcr0kWiBmnxP2jH1xdsvjc55rN14w/exec";
const String myFilename = "filename = ESP32 - CAM.jpg";
const String mimeType = "&mimetype = imagejpeg";
const String myImage = "&datase = ";
const String TAG = "Matchboxscope";

const int waitingTime = 30000; // Wait 30 seconds to google response.

/*
  GLOBAL CONSTANTS & STATE
*/

// LED

const int freq = 8000;       // 800000;//19000; //12000
const int pwmResolution = 8; // 15
const int ledChannel = 4;    // some are used by the camera
const int ledPin = 4;
int ledValueOld = 0;

// LED array
int ledMatrixCount = 2;

// Lens
const int lensChannel = 5; // some are used by the camera
const int lensPin = 12;
uint32_t lensValueOld = 0;

// Button/reed-switch for resetting
const int refocus_button_debounce = 1000;

// Camera
uint32_t gain = 0;
uint32_t exposureTime = 0;
uint32_t frameSize = 0;
uint32_t ledintensity = 0;
uint32_t effect = 2;
bool isStreaming = false;
bool isStreamingStoppped = false;
bool isCameraAttached = false;

// SD Card
boolean sdInitialized = false;
boolean isFirstRun = false;
boolean isUseSD = true;

// Timelapse
uint64_t timelapseInterval = -1; 
static uint64_t t_old = 0;
int uniqueID = random(100000);

// Preferences
Preferences pref;
DevicePreferences device_pref(pref, "camera", __DATE__ " " __TIME__);

// Wifi
WiFiManager wm;
bool isInternetAvailable = false;
// check wifi connectibility if not connected, try to reconnect - or restart?
unsigned long previousCheckWifi = 0;
unsigned long delayReconnect = 20000; // 20 seconds delay

// Server
boolean isWebserver = true;

// Anglerfish
boolean isTimelapseAnglerfish = false; // keep as false!
boolean isAcquireStack = false;        // acquire only single image or stack?
File myFile;

// OTA server
WebServer OTAserver(82);

/*****************************
   Helper Functions
 ******************************/

// https://randomnerdtutorials.com/esp32-deep-sleep-arduino-ide-wake-up-sources/
void print_wakeup_reason()
{
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch (wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0:
      Serial.println("Wakeup caused by external signal using RTC_IO");
      break;
    case ESP_SLEEP_WAKEUP_EXT1:
      Serial.println("Wakeup caused by external signal using RTC_CNTL");
      break;
    case ESP_SLEEP_WAKEUP_TIMER:
      Serial.println("Wakeup caused by timer");
      // VSM still not working after automatic reboot - hitting Reset does the job :/
      ESP.restart(); // FIMXE: Yup, this is weird: Since we connect the awake-Button AND the VCM transistor to Pin12, the pin is still in input mode when the esp is woken up by timer..so we have to get another cause for the wake up=> force restart!
      break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD:
      Serial.println("Wakeup caused by touchpad");
      break;
    case ESP_SLEEP_WAKEUP_ULP:
      Serial.println("Wakeup caused by ULP program");
      break;
    default:
      Serial.printf("Wakeup was not caused by deep sleep: %d\n", wakeup_reason);
      break;
  }
}



void callbackTouchpad()
{
}

bool initCamera()
{
  // INIT Camera
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
  // init with high specs to pre-allocate larger buffers

  config.frame_size = FRAMESIZE_VGA;
  config.jpeg_quality = 10;
  config.fb_count = 2;

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK)
  {
    Serial.printf("Camera init failed with error 0x % x", err);
    // ESP.restart();
    return false;
  }
  else
  {
    Serial.printf("Camera init success!");
    return true;
  }
}

void initCameraSettings()
{
  // Apply manual settings for the camera
  sensor_t *s = esp_camera_sensor_get();
  s->set_framesize(s, FRAMESIZE_QVGA);
  // s->set_vflip(s, 1);
  // s->set_hmirror(s, 1);
  s->set_exposure_ctrl(s, 0);                              // (aec) 0 = disable , 1 = enable
  s->set_aec2(s, 0);                                       // (aec2) Auto EXP DSP 0 = disable , 1 = enable
  s->set_ae_level(s, 0);                                   // (ae_level) -2 to 2
  s->set_aec_value(s, exposureTime);                       // (aec_value) 0 to 1200
  s->set_gain_ctrl(s, 0);                                  // auto gain off
  s->set_brightness(s, 0);                                 // -2 to 2
  s->set_special_effect(s, device_pref.getCameraEffect()); // mono - has to be set again?
  s->set_wb_mode(s, 0);
  s->set_awb_gain(s, 0);
  s->set_lenc(s, 1);
  s->set_agc_gain(s, gain); // 0 to 30
}

bool saveImage(String filename, int ledIntensity, int lensValue)
{

  // Stop stream
  isStreaming = false;

  // TODO: We need to stop the stream here!
  if (sdInitialized)
  { // Do not attempt to save anything to a non-existig SD card
    camera_fb_t *frameBuffer = NULL;

    // move lens
    setLens(lensValue);
    delay(50);
    setLED(ledIntensity);
    // set maximum framesize
    sensor_t *s = esp_camera_sensor_get();
    setFrameSize(device_pref.getCameraFramesize());           // TODO: Why does it change the exposure time/brightness??!
    s->set_aec_value(s, device_pref.getCameraExposureTime()); // (aec_value) 0 to 1200
    s->set_special_effect(s, device_pref.getCameraEffect());  // mono - has to be set again?
    s->set_agc_gain(s, device_pref.getCameraGain());          // 0 to 30

    // Take Picture with Camera, but first let camera warm up => make sure low res frame in buffer is freed
    for (int iwarmup = 0; iwarmup < 3; iwarmup++)
    {
      frameBuffer = esp_camera_fb_get();
      esp_camera_fb_return(frameBuffer);
    }

    // Take picture LED #0
    frameBuffer = esp_camera_fb_get();

    // turn off lens/LED to save energy
    setLED(0);
    setLens(0);

    if (!frameBuffer)
    {
      Serial.println("Camera capture failed");
      ESP.restart();
      return false;
    }

    // Save image to disk
    fs::FS &fs = SD_MMC;
    File imgFile = fs.open(filename.c_str(), FILE_WRITE);
    if (!imgFile)
    {
      Serial.println("Failed to open file in writing mode");
      return false;
    }
    else
    {
      imgFile.write(frameBuffer->buf, frameBuffer->len);
      Serial.println("Saved " + filename);
    }
    imgFile.close();
    esp_camera_fb_return(frameBuffer);
    delay(100);

    // resetFramesize to value before frame caputring
    setFrameSize(device_pref.getCameraFramesize());

    return true;
  }
  else
  {
    return false;
  }
}

typedef struct
{
  httpd_req_t *req;
  size_t len;
} jpg_chunking_t;

#define PART_BOUNDARY "123456789000000000000987654321"
static const char *_STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char *_STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char *_STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

httpd_handle_t stream_httpd = NULL;
httpd_handle_t camera_httpd = NULL;

void setFrameSize(int val)
{
  /*
    FRAMESIZE_UXGA (1600 x 1200)
    FRAMESIZE_QVGA (320 x 240)
    FRAMESIZE_CIF (352 x 288)
    FRAMESIZE_VGA (640 x 480)
    FRAMESIZE_SVGA (800 x 600)
    FRAMESIZE_XGA (1024 x 768)
    FRAMESIZE_SXGA (1280 x 1024)
  */
  sensor_t *s = esp_camera_sensor_get();
  if (val == 0)
    s->set_framesize(s, FRAMESIZE_QQVGA);
  else if (val == 1)
    s->set_framesize(s, FRAMESIZE_HQVGA);
  else if (val == 2)
    s->set_framesize(s, FRAMESIZE_QVGA);
  else if (val == 3)
    s->set_framesize(s, FRAMESIZE_CIF);
  else if (val == 4)
    s->set_framesize(s, FRAMESIZE_VGA);
  else if (val == 5)
    s->set_framesize(s, FRAMESIZE_SVGA);
  else if (val == 6)
    s->set_framesize(s, FRAMESIZE_XGA);
  else if (val == 7)
    s->set_framesize(s, FRAMESIZE_SXGA);
  else if (val == 8)
    s->set_framesize(s, FRAMESIZE_QVGA);
  else
    s->set_framesize(s, FRAMESIZE_QVGA);
}

std::string get_content(httpd_req_t *req)
{
  // https://github.com/chhartmann/RoboProg/blob/ccc3342fccebc030b337dbaf20f5e917b5b24e5f/src/web_interface.cpp
  char buf[1025];
  int received;

  unsigned int remaining = req->content_len;
  std::string content;
  content.reserve(remaining);

  while (remaining > 0)
  {
    if ((received = httpd_req_recv(req, buf, std::min(remaining, sizeof(buf) - 1))) <= 0)
    {
      if (received == HTTPD_SOCK_ERR_TIMEOUT)
      {
        /* Retry if timeout occurred */
        continue;
      }

      /* In case of unrecoverable error, close and delete the unfinished file*/
      httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to receive file");
    }
    else
    {
      buf[received] = '\0';
      content += buf;
      remaining -= received;
    }
  }
  return content;
}

// Stream& input;
StaticJsonDocument<1024> doc;
static esp_err_t json_handler(httpd_req_t *req)
{
  // we have to ways of changinv values through Json and random..
  //  json-only would be better

  // convert http to json
  std::string content = get_content(req);
  DeserializationError error = deserializeJson(doc, content);

  if (error)
  {
    Serial.print("deserializeJson() failed: ");
    Serial.println(error.c_str());
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to create file");
    return ESP_FAIL;
  }
  else
  {
    // Adjust sensor values from REST
    sensor_t *s = esp_camera_sensor_get();

    if (doc.containsKey("gain"))
    {
      gain = doc["gain"];
      device_pref.setCameraGain(gain);
      Serial.println(gain);
      s->set_agc_gain(s, device_pref.getCameraGain());         // 0 to 30
      s->set_special_effect(s, device_pref.getCameraEffect()); // mono - has to be set again?
    }
    if (doc.containsKey("exposureTime"))
    {
      exposureTime = doc["exposureTime"];
      device_pref.setCameraExposureTime(exposureTime);
      Serial.println(exposureTime);
      s->set_aec_value(s, device_pref.getCameraExposureTime()); // (aec_value) 0 to 1200
      s->set_special_effect(s, device_pref.getCameraEffect());  // mono - has to be set again?
    }
    if (doc.containsKey("framesize"))
    {
      // FRAMESIZE
      frameSize = doc["framesize"];
      Serial.print("framesize: ");
      device_pref.setCameraFramesize(frameSize);
      setFrameSize(device_pref.getCameraFramesize());
      Serial.println(device_pref.getCameraFramesize());
      s->set_special_effect(s, device_pref.getCameraEffect()); // mono - has to be set again?
    }
    if (doc.containsKey("effect"))
    {
      // FRAMESIZE
      effect = doc["effect"];
      Serial.print("effect: ");
      device_pref.setCameraEffect(effect);
      s->set_special_effect(s, device_pref.getCameraEffect()); // mono - has to be set again?
      Serial.println(device_pref.getCameraEffect());
    }
    if (doc.containsKey("ledintensity"))
    {
      // LED Intensity
      ledintensity = doc["ledintensity"];
      ledValueOld = ledintensity;
      setLED(ledValueOld);

      Serial.println(ledintensity);
    }
    if (doc.containsKey("ledintensity2"))
    {
      // LED Intensity
      ledintensity = doc["ledintensity2"];
      ledValueOld = ledintensity;
      setLED(ledValueOld);

      Serial.println(ledintensity);
    }
    if (doc.containsKey("lensvalue"))
    {
      // LENS Value
      lensValueOld = doc["lensvalue"];
      setLens(lensValueOld);
      Serial.println(lensValueOld);
    }

    httpd_resp_set_hdr(req, "Connection", "close");
    httpd_resp_send(req, "OK", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
  }
}

int readFile(char *fname, httpd_req_t *req)
{
  // https://github.com/k-kimura123/Final_candyart_project/blob/ff3c950d02614a1ea169afaa0ac1793b503db4c5/main/include/myserver_config.c
  int res;
  char buf[1024];

  FILE *fd = fopen(fname, "rb");
  if (fd == NULL)
  {
    ESP_LOGE(TAG, "ERROR opening file (%d) %s\n", errno, strerror(errno));
    httpd_resp_send_404(req);
    return 0;
  }
  do
  {
    res = fread(buf, 1, sizeof(buf), fd);
    if (res > 0)
    {
      httpd_resp_send_chunk(req, buf, res);
      printf("Read %d\n", res);
    }
  } while (res > 0);
  httpd_resp_send_chunk(req, NULL, 0);
  res = fclose(fd);
  if (res)
  {
    printf("Error closing file\n");
  }
  return 1;
}

static size_t jpg_encode_stream(void *arg, size_t index, const void *data, size_t len)
{
  jpg_chunking_t *j = (jpg_chunking_t *)arg;
  if (!index)
  {
    j->len = 0;
  }
  if (httpd_resp_send_chunk(j->req, (const char *)data, len) != ESP_OK)
  {
    return 0;
  }
  j->len += len;
  return len;
}

static esp_err_t capture_handler(httpd_req_t *req)
{

  // Stop stream
  isStreaming = false;

  // make sure buffer is freed and framesize is taken over
  sensor_t *s = esp_camera_sensor_get();
  setFrameSize(device_pref.getCameraFramesize());           // TODO: Why does it change the exposure time/brightness??!
  s->set_aec_value(s, device_pref.getCameraExposureTime()); // (aec_value) 0 to 1200
  s->set_special_effect(s, device_pref.getCameraEffect());  // mono - has to be set again?
  s->set_agc_gain(s, device_pref.getCameraGain());          // 0 to 30

  camera_fb_t *frameBuffer = NULL;

  // Take Picture with Camera, but first let camera warm up => make sure low res frame in buffer is freed
  for (int iwarmup = 0; iwarmup < 3; iwarmup++)
  {
    frameBuffer = esp_camera_fb_get();
    esp_camera_fb_return(frameBuffer);
  }
  frameBuffer = esp_camera_fb_get();

  if (!frameBuffer)
  {
    Serial.println("Camera capture failed");
    ESP.restart();
    return false;
  }

  httpd_resp_set_type(req, "image/jpeg");
  httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpeg");

  size_t out_len, out_width, out_height;
  uint8_t *out_buf;
  esp_err_t res = -1;
  // bool s;
  {
    size_t fb_len = 0;
    if (frameBuffer->format == PIXFORMAT_JPEG)
    {
      fb_len = frameBuffer->len;
      res = httpd_resp_send(req, (const char *)frameBuffer->buf, frameBuffer->len);
    }
    else
    {
      jpg_chunking_t jchunk = {req, 0};
      res = frame2jpg_cb(frameBuffer, 80, jpg_encode_stream, &jchunk) ? ESP_OK : ESP_FAIL;
      httpd_resp_send_chunk(req, NULL, 0);
      fb_len = jchunk.len;
    }
    esp_camera_fb_return(frameBuffer);
    Serial.printf("JPG: %uB\n", (uint32_t)(fb_len));

    return res;
  }
  esp_camera_fb_return(frameBuffer);
  Serial.println("dl_matrix3du_alloc failed");
  httpd_resp_send_500(req);
  return ESP_FAIL;
}

static esp_err_t stream_handler(httpd_req_t *req)
{

  // enable CORS headers for spectrometer?
  static const char *CORS_HEADER = "Access-Control-Allow-Origin";
  static const char *CORS_HEADER_VALUE = "*";
  const char HEADER[] = "HTTP/1.1 200 OK\r\n"
                        "Access-Control-Allow-Origin: *\r\n"
                        "Content-Type: multipart/x-mixed-replace; boundary=123456789000000000000987654321\r\n";

  camera_fb_t *fb = NULL;
  esp_err_t res = ESP_OK;
  size_t _jpg_buf_len = 0;
  uint8_t *_jpg_buf = NULL;
  char *part_buf[64];
  //  dl_matrix3du_t *image_matrix = NULL;

  static int64_t last_frame = 0;
  if (!last_frame)
  {
    last_frame = esp_timer_get_time();
  }
  res = httpd_resp_set_type(req, HEADER);
  res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
  if (res != ESP_OK)
  {
    return res;
  }

  // in case we want to stop it externally
  isStreaming = true;

  while (isStreaming)
  {
    fb = esp_camera_fb_get();
    if (!fb)
    {
      Serial.println("Camera capture failed");
      res = ESP_FAIL;
      ESP.restart();
    }
    else
    {
      {
        if (fb->format != PIXFORMAT_JPEG)
        {
          bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
          esp_camera_fb_return(fb);
          fb = NULL;
          if (!jpeg_converted)
          {
            Serial.println("JPEG compression failed");
            res = ESP_FAIL;
          }
        }
        else
        {
          _jpg_buf_len = fb->len;
          _jpg_buf = fb->buf;
        }
      }
    }
    // send cors header
    httpd_resp_set_hdr(req, CORS_HEADER, CORS_HEADER_VALUE);
    if (res == ESP_OK)
    {
      size_t hlen = snprintf((char *)part_buf, 64, _STREAM_PART, _jpg_buf_len);
      res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
    }
    if (res == ESP_OK)
    {
      res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
    }
    if (res == ESP_OK)
    {
      res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
    }
    if (fb)
    {
      esp_camera_fb_return(fb);
      fb = NULL;
      _jpg_buf = NULL;
    }
    else if (_jpg_buf)
    {
      free(_jpg_buf);
      _jpg_buf = NULL;
    }
    if (res != ESP_OK)
    {
      break;
    }
  }

  last_frame = 0;
  return res;
}

static esp_err_t cmd_handler(httpd_req_t *req)
{ // FIXME: This should be merged with the REST API in line 200 - website works with this interface..not good!
  // TODO: this should be arduinojson
  char *buf;
  size_t buf_len;
  char variable[32] = {
    0,
  };
  char value[32] = {
    0,
  };

  // adjust parameters
  buf_len = httpd_req_get_url_query_len(req) + 1;
  if (buf_len > 1)
  {
    buf = (char *)malloc(buf_len);
    if (!buf)
    {
      httpd_resp_send_500(req);
      return ESP_FAIL;
    }
    if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK)
    {
      if (httpd_query_key_value(buf, "var", variable, sizeof(variable)) == ESP_OK &&
          httpd_query_key_value(buf, "val", value, sizeof(value)) == ESP_OK)
      {
      }
      else
      {
        free(buf);
        httpd_resp_send_404(req);
        return ESP_FAIL;
      }
    }
    else
    {
      free(buf);
      httpd_resp_send_404(req);
      return ESP_FAIL;
    }
    free(buf);
  }
  else
  {
    httpd_resp_send_404(req);
    return ESP_FAIL;
  }

  int val = atoi(value);
  sensor_t *s = esp_camera_sensor_get();
  int res = 0;

  if (!strcmp(variable, "brightness"))
  {
    Serial.print("brightness ");
    float brightness = (float)val;
    Serial.println(brightness);
    s->set_brightness(s, brightness);                        // -2 to 2
    s->set_special_effect(s, device_pref.getCameraEffect()); // mono - has to be set again?
  }
  if (!strcmp(variable, "effect"))
  {
    Serial.print("effect ");
    float effect = val;
    device_pref.setCameraEffect(effect);
    s->set_special_effect(s, device_pref.getCameraEffect()); // mono - has to be set again?
    Serial.println(device_pref.getCameraEffect());
  }
  else if (!strcmp(variable, "gain"))
  {
    gain = val;
    device_pref.setCameraGain(gain);
    s->set_agc_gain(s, device_pref.getCameraGain());         // 0 to 30
    s->set_special_effect(s, device_pref.getCameraEffect()); // mono - has to be set again?

    Serial.print("gain ");
    Serial.println(gain);
    Serial.println(device_pref.getCameraGain());
  }
  else if (!strcmp(variable, "exposuretime"))
  {
    // EXPOSURE TIME
    exposureTime = val;
    device_pref.setCameraExposureTime(exposureTime);
    s->set_aec_value(s, device_pref.getCameraExposureTime()); // 0 to 30
    s->set_special_effect(s, device_pref.getCameraEffect());  // mono - has to be set again?

    Serial.print("exposureTime ");
    Serial.println(device_pref.getCameraExposureTime());
    Serial.println(exposureTime);
  }
  else if (!strcmp(variable, "framesize"))
  {
    // FRAMESIZE
    frameSize = val;
    device_pref.setCameraFramesize(frameSize);
    setFrameSize(device_pref.getCameraFramesize());
    s->set_special_effect(s, device_pref.getCameraEffect()); // mono - has to be set again?

    Serial.print("framesize: ");
    Serial.println(device_pref.getCameraFramesize());
    Serial.println(val);
  }
  else if (!strcmp(variable, "lenscorrection"))
  {
    // Timelapse Periode
    s->set_lenc(s, val);
    Serial.print(" lenscorrection: ");
    Serial.println(val);
  }
  else if (!strcmp(variable, "timelapseinterval"))
  {
    // Timelapse Periode
    device_pref.setTimelapseInterval(val);
    timelapseInterval = device_pref.getTimelapseInterval();

    Serial.print("timelapse Interval: ");
    Serial.println(device_pref.getTimelapseInterval());
    Serial.println(val);
  }
  else if (!strcmp(variable, "quality"))
  {
    // QUALITY
    Serial.println("quality");
    res = s->set_quality(s, val);
  }
  else if (!strcmp(variable, "flash"))
  {
    // FLASH
    ledValueOld = val;
    setLED(ledValueOld);

    Serial.print("LED VAlue");
    Serial.println(ledValueOld);
  }
  else if (!strcmp(variable, "flash2"))
  {
    // FLASH
    ledValueOld = val;
    setLED(ledValueOld);

    Serial.print("LED VAlue2");
    Serial.println(ledValueOld);
  }
  else if (!strcmp(variable, "lens"))
  {
    // LENS
    lensValueOld = val;
    setLens(lensValueOld);
  }
  else
  {
    Serial.println("variable");
    res = -1;
  }

  if (res)
  {
    return httpd_resp_send_500(req);
  }

  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, NULL, 0);
}

static esp_err_t restart_handler(httpd_req_t *req)
{
  Serial.println("Restarting");
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_send(req, "OK", strlen("OK"));
  ESP.restart();
  return ESP_OK;
}

boolean snapPhoto(String fileName, int ledIntensity, int lensVal)
{
  bool savedSuccessfully = false;

  // FIXME: clever to have the LED turned on here or rather outside the snap function?
  // setLED(ledIntensity); // turn on LED
  savedSuccessfully = saveImage(fileName + ".jpg", ledIntensity, lensVal);
  // TODO make this traggerable from - perhaps a button?
  // FIXME: this should be triggered by a buttun - or only if wifi and internet are available:
  if (isInternetAvailable)
  {
    saveCapturedImageGDrive();
  }

  // setLED(ledValueOld); // tune LED to old value
  return savedSuccessfully;
}

// FIXME: This method sounds not true; Rather focus Stack?
bool doFocus(int lensIncrement, bool isSave, bool isFocus, String fileName)
{
  // reserve buffer
  camera_fb_t *frameBuffer = NULL;

  // Acquire the image and save
  int maxSharpness = 0;
  int lensValMaxSharpness = 0;

  int ledIntensity = 255;
  bool savedSuccessfully = false;

  for (int iLensVal = 0; iLensVal < 255; iLensVal += abs(lensIncrement))
  {

    // save frame - eventually
    if (isSave)
    {
      Serial.println("/" + fileName + "_Z_" + String(iLensVal));
      savedSuccessfully = snapPhoto("/" + fileName + "_Z_" + String(iLensVal), ledIntensity, iLensVal);
    }

    /* FIXME: This causes issues when in deepsleep mode . .see error.h
      // Measure sharpness
      frameBuffer = esp_camera_fb_get();
      esp_camera_fb_return(frameBuffer);
      size_t fb_len = 0;
      fb_len = frameBuffer->len;
      Serial.printf("JPG: %uB", (uint32_t)(fb_len));
      if (fb_len > maxSharpness)
      {
      maxSharpness = fb_len;
      lensValMaxSharpness = iLensVal;
      }

      }
      // autofocus?
      if (isFocus) {
      setLens(lensValMaxSharpness);
      }
    */
  }

  return savedSuccessfully;
}

static esp_err_t stack_handler(httpd_req_t *req)
{
  Serial.println("Recording Stack");
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_send(req, "OK", strlen("OK"));
  // acquire a stack
  uint32_t frame_index = device_pref.getFrameIndex() + 1;
  bool imageSaved = false;
  // FIXME: decide which method to use..
  imageSaved = doFocus(5, true, true, "/anglerfish_" + String(frame_index));
  if (true)
  { //(imageSaved)
    device_pref.setFrameIndex(frame_index);
  }
  // focus back on old value
  // FIXME - perform autofocus here? setLens(lensValueOld);
  return ESP_OK;
}

static esp_err_t id_handler(httpd_req_t *req)
{
  Serial.println("Handling ID");
  static char json_response[1024];
  String uniqueID = "OMNISCOPE" __DATE__ " " __TIME__;
  Serial.println("Handling ID");
  Serial.println(uniqueID);

  char *p = json_response;
  *p++ = '{';
  p += sprintf(p, uniqueID.c_str());
  *p++ = '}';
  *p++ = 0;
  httpd_resp_set_type(req, "applicationjson");
  httpd_resp_set_hdr(req, "Access - Control - Allow - Origin", "*");
  return httpd_resp_send(req, json_response, strlen(json_response));
}

static esp_err_t index_handler(httpd_req_t *req)
{
  httpd_resp_set_type(req, "texthtml");
  return httpd_resp_send(req, (const char *)INDEX_HTML, strlen(INDEX_HTML));
}

static esp_err_t status_handler(httpd_req_t *req)
{
  static char json_response[1024];

  sensor_t *s = esp_camera_sensor_get();
  char *p = json_response;
  *p++ = '{';

  p += sprintf(p, "\"framesize\":%u,", s->status.framesize);
  p += sprintf(p, "\"quality\":%u,", s->status.quality);
  *p++ = '}';
  *p++ = 0;
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, json_response, strlen(json_response));
}

static esp_err_t enable_handler(httpd_req_t *req)
{
  Serial.println("Going into deepsleep mode");
  Serial.println(timelapseInterval);
  device_pref.setIsTimelapse(true);
  static char json_response[1024];
  char *p = json_response;
  *p++ = '{';
  p += sprintf(p, "You have enabled long-time timelpase - reflash the code or release the button to wake up the ESP again");
  *p++ = '}';
  *p++ = 0;
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_send(req, json_response, strlen(json_response));

  SD_MMC.end(); // FIXME: may cause issues when file not closed? categoreis: LED/SD-CARD issues
  ESP.restart();
  return 0;
}

esp_err_t indexhtml_handler(httpd_req_t *req)
{
  ESP_LOGI(TAG, "url %s was hit", req->uri);
  printf("main page requested\r\n");
  httpd_resp_set_type(req, "text/html");
  readFile("/spiffs/index.html", req);

  return ESP_OK;
}

void startCameraServer()
{
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();

  httpd_uri_t index_uri = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = index_handler,
    .user_ctx = NULL
  };

  httpd_uri_t indexhtml_uri = {
    .uri = "/index.html",
    .method = HTTP_GET,
    .handler = indexhtml_handler,
    .user_ctx = NULL
  };

  httpd_uri_t status_uri = {
    .uri = "/status",
    .method = HTTP_GET,
    .handler = status_handler,
    .user_ctx = NULL
  };

  httpd_uri_t cmd_uri = {
    .uri = "/control",
    .method = HTTP_GET,
    .handler = cmd_handler,
    .user_ctx = NULL
  };

  httpd_uri_t enable_uri = {
    .uri = "/enable",
    .method = HTTP_GET,
    .handler = enable_handler,
    .user_ctx = NULL
  };

  httpd_uri_t capture_uri = {
    .uri = "/capture.jpeg",
    .method = HTTP_GET,
    .handler = capture_handler,
    .user_ctx = NULL
  };

  httpd_uri_t stream_uri = {
    .uri = "/stream.mjpeg",
    .method = HTTP_GET,
    .handler = stream_handler,
    .user_ctx = NULL
  };

  httpd_uri_t id_uri = {
    .uri = "/getid",
    .method = HTTP_GET,
    .handler = id_handler,
    .user_ctx = NULL
  };

  httpd_uri_t stack_uri = {
    .uri = "/stack",
    .method = HTTP_GET,
    .handler = stack_handler,
    .user_ctx = NULL
  };

  httpd_uri_t restart_uri = {
    .uri = "/restart",
    .method = HTTP_GET,
    .handler = restart_handler,
    .user_ctx = NULL
  };

  httpd_uri_t postjson_uri = {
    .uri = "/postjson",
    .method = HTTP_POST,
    .handler = json_handler,
    .user_ctx = NULL
  };

  Serial.printf("Starting web server on port: '%d'\n", config.server_port);
  if (httpd_start(&camera_httpd, &config) == ESP_OK)
  {
    httpd_register_uri_handler(camera_httpd, &index_uri);
    httpd_register_uri_handler(camera_httpd, &cmd_uri);
    httpd_register_uri_handler(camera_httpd, &enable_uri);
    httpd_register_uri_handler(camera_httpd, &status_uri);
    httpd_register_uri_handler(camera_httpd, &capture_uri);
    httpd_register_uri_handler(camera_httpd, &id_uri);
    httpd_register_uri_handler(camera_httpd, &stack_uri);
    httpd_register_uri_handler(camera_httpd, &indexhtml_uri);
    httpd_register_uri_handler(camera_httpd, &postjson_uri);
    httpd_register_uri_handler(camera_httpd, &restart_uri);
  }

  config.server_port += 1;
  config.ctrl_port += 1;
  Serial.printf("Starting stream server on port: '%d'\n", config.server_port);
  if (httpd_start(&stream_httpd, &config) == ESP_OK)
  {
    httpd_register_uri_handler(stream_httpd, &stream_uri);
  }
}

void startOTAServer()
{
  /*return index page which is stored in serverIndex */

  Serial.println("Spinning up OTA server");
  OTAserver.on("/", HTTP_GET, []()
  {
    OTAserver.sendHeader("Connection", "close");
    OTAserver.send(200, "text/html", otaindex);
  });
  /*handling uploading firmware file */
  OTAserver.on(
    "/update", HTTP_POST, []()
  {
    OTAserver.sendHeader("Connection", "close");
    OTAserver.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
    ESP.restart();
  },
  []()
  {
    HTTPUpload &upload = OTAserver.upload();
    if (upload.status == UPLOAD_FILE_START)
    {
      Serial.printf("Update: %s\n", upload.filename.c_str());
      if (!Update.begin(UPDATE_SIZE_UNKNOWN))
      { // start with max available size
        Update.printError(Serial);
      }
    }
    else if (upload.status == UPLOAD_FILE_WRITE)
    {
      /* flashing firmware to ESP*/
      if (Update.write(upload.buf, upload.currentSize) != upload.currentSize)
      {
        Update.printError(Serial);
      }
    }
    else if (upload.status == UPLOAD_FILE_END)
    {
      if (Update.end(true))
      { // true to set the size to the current progress
        Serial.printf("Update Success: %u\nRebooting...\n", upload.totalSize);
      }
      else
      {
        Update.printError(Serial);
      }
    }
  });
  OTAserver.begin();
  Serial.println("Starting OTA server on port: '82'");
  Serial.println("Visit http://IPADDRESS_SCOPE:82");
}

void setLens(int lensValue)
{
  if (lensValue > 255)
    lensValue = 255;
  else if (lensValue < 0)
  {
    lensValue = 0;
  }
  Serial.print("LENS Value: ");
  Serial.println(lensValue);
  ledcWrite(lensChannel, lensValue);
}

void setLED(int intensity)
{
  // use internal LED/TORCH
  Serial.print("LED : ");
  Serial.println(intensity);
  ledcWrite(ledChannel, intensity);
}

void blinkLed(int nTimes)
{
  // TODO: Be careful with this - interferes with sensor and ledcWrite?!
  for (int iBlink = 0; iBlink < nTimes; iBlink++)
  {
    setLED(255);
    setLED(255);
    delay(50);
    setLED(0);
    setLED(0);
    delay(50);
  }
  delay(150);
}

// https://github.com/gsampallo/esp32cam-gdrive

void saveCapturedImageGithub()
{
  /* Need to package:
    curl   -X PUT   -H "Accept: application/vnd.github+json"   \
    -H "Authorization: Bearer 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"  \ // FIXME: we can not store tokens in github :D
    https://api.github.com/repos/anglerfishbot/AnglerfishGallery/contents/test3   -\
    d '{"message":"my commit message","committer":{"name":"anglerfishbot","email":"benedictdied@gmail.com"},"content":"bXkgbmV3IGZpbGUgY29udGVudHM="}'
  */
}

// https://github.com/zenmanenergy/ESP8266-Arduino-Examples/
String urlencode(String str)
{
  String encodedString = "";
  char c;
  char code0;
  char code1;
  char code2;
  for (int i = 0; i < str.length(); i++)
  {
    c = str.charAt(i);
    if (c == ' ')
    {
      encodedString += '+';
    }
    else if (isalnum(c))
    {
      encodedString += c;
    }
    else
    {
      code1 = (c & 0xf) + '0';
      if ((c & 0xf) > 9)
      {
        code1 = (c & 0xf) - 10 + 'A';
      }
      c = (c >> 4) & 0xf;
      code0 = c + '0';
      if (c > 9)
      {
        code0 = c - 10 + 'A';
      }
      code2 = '\0';
      encodedString += '%';
      encodedString += code0;
      encodedString += code1;
      // encodedString+=code2;
    }
    yield();
  }
  return encodedString;
}

void saveCapturedImageGDrive()
{
  Serial.println("Connect to " + String(myDomain));
  Serial.println("Connect to " + String(myDomain));
  WiFiClientSecure clientSecure;
  clientSecure.setInsecure(); // run version 1.0.5 or above

  if (clientSecure.connect(myDomain, 443))
  {
    Serial.println("Connection successful");

    setLED(255);
    camera_fb_t *fb = NULL;
    fb = esp_camera_fb_get();
    if (!fb)
    {
      Serial.println("Camera capture failed");
      delay(1000);
      ESP.restart();
      return;
    }

    char *input = (char *)fb->buf;
    char output[base64_enc_len(3)];
    String imageFile = "";
    for (int i = 0; i < fb->len; i++)
    {
      base64_encode(output, (input++), 3);
      if (i % 3 == 0)
        imageFile += urlencode(String(output));
    }
    String Data = myFilename + mimeType + myImage;

    esp_camera_fb_return(fb);

    Serial.println("Send a captured image to Google Drive.");

    clientSecure.println("POST " + myScript + " HTTP/1.1");
    clientSecure.println("Host: " + String(myDomain));
    clientSecure.println("Content-Length: " + String(Data.length() + imageFile.length()));
    clientSecure.println("Content-Type: application/x-www-form-urlencoded");
    clientSecure.println();

    clientSecure.print(Data);
    int Index;
    for (Index = 0; Index < imageFile.length(); Index = Index + 1000)
    {
      clientSecure.print(imageFile.substring(Index, Index + 1000));
    }

    Serial.println("Waiting for response.");
    long int StartTime = millis();
    while (!clientSecure.available())
    {
      Serial.print(".");
      delay(100);
      if ((StartTime + waitingTime) < millis())
      {
        Serial.println();
        Serial.println("No response.");
        // If you have no response, maybe need a greater value of waitingTime
        break;
      }
    }
    Serial.println();
    while (clientSecure.available())
    {
      Serial.print(char(clientSecure.read()));
    }
  }
  else
  {
    Serial.println("Connection to " + String(myDomain) + " failed.");
  }
  clientSecure.stop();
  setLED(0);
}

void initWifiAP(const char *ssid)
{
  Serial.print("Network SSID (AP): ");
  Serial.println(ssid);

  WiFi.softAP(ssid);
  Serial.print("AP IP address: ");
  Serial.println(WiFi.softAPIP());
}

void joinWifi(const char *ssid, const char *password)
{
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  int nConnectTrials = 0;
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(500);
    nConnectTrials += 1;
    if (nConnectTrials > 10)
      ESP.restart();
    // we can even make the ESP32 to sleep
  }

  Serial.print("Connected. IP: ");
  Serial.println(WiFi.localIP());
}

void autoconnectWifi(boolean isResetWifiSettings)
{
  WiFi.mode(WIFI_STA); // explicitly set mode, esp defaults to STA+AP
  // it is a good practice to make sure your code sets wifi mode how you want it.

  // reset settings - wipe stored credentials for testing
  // these are stored by the esp library
  if (isResetWifiSettings)
  {
    Serial.println("First run => resetting Wifi Settings");
    wm.resetSettings();
  }
  wm.setHostname(hostname);
  // wm.setConfigPortalBlocking(false);
  wm.setConfigPortalTimeout(90); // auto close configportal after n seconds
  wm.setConnectTimeout(10);

  // Automatically connect using saved credentials,
  // if connection fails, it starts an access point with the specified name ( "AutoConnectAP"),
  // if empty will auto generate SSID, if password is blank it will be anonymous AP (wm.autoConnect())
  // then goes into a blocking loop awaiting configuration and will return success result
  bool res;
  // res = wm.autoConnect(); // auto generated AP name from chipid
  // res = wm.autoConnect("AutoConnectAP"); // anonymous ap
  res = wm.autoConnect(mSSIDAP); // password protected ap

  if (!res)
  {
    Serial.println("Failed to connect");
    initWifiAP(mSSIDAP);
  }
  else
  {
    // if you get here you have connected to the WiFi
    Serial.println("connected...");
  }

  Serial.print("Connected. IP: ");
  Serial.println(WiFi.localIP());
}

/***************************+
   ACTUAL CODE
 * **************************/

void setup()
{
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); // prevent brownouts by silencing them

  // INIT SERIAL
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

  // Print the wakeup reason for ESP32
  print_wakeup_reason();
  detachInterrupt(T3); // FIXME: just in case?

  /*
    AGLERFISH RELATED
  */
  // Reset the EEPROM's stored timelapse mode after each re - flash
  isFirstRun = device_pref.isFirstRun();
  if (isFirstRun)
  {
    device_pref.setIsTimelapse(false);
  }

  // retrieve old camera setting values
  exposureTime = device_pref.getCameraExposureTime();
  gain = device_pref.getCameraGain();
  timelapseInterval = device_pref.getTimelapseInterval(); // Get timelapseInterval
  Serial.print("exposureTime : ");
  Serial.println(exposureTime);
  Serial.print("gain : ");
  Serial.println(gain);
  Serial.print("timelapseInterval : ");
  Serial.println(timelapseInterval);

  // INIT CAMERA
  isCameraAttached = initCamera();
  if (isCameraAttached)
  {
    Serial.println("Camera is working");
    initCameraSettings(); // set camera settings - exposure time and gain will betaken from preferences
  }
  else
  {
    Serial.println("Camera is not working");
  }

  // INIT SD
  // We initialize SD_MMC here rather than in setup() because SD_MMC needs to reset the light pin
  // with a different pin mode.
  // 1-bit mode as suggested here:https://dr-mntn.net/2021/02/using-the-sd-card-in-1-bit-mode-on-the-esp32-cam-from-ai-thinker
  if (!SD_MMC.begin("/sdcard", true))
  { // FIXME: this sometimes leads to issues Unix vs. Windows formating - text encoding? Sometimes it copies to "sdcard" => Autoformating does this!!!
    Serial.println("SD Card Mount Failed");
    // FIXME: This should be indicated in the GUI
    sdInitialized = false;
    device_pref.setIsTimelapse(false); // FIXME: if SD card is missing => streaming mode!
    // FIXME: won't work since LEDC is not yet initiated blinkLed(5);
    /*
      setLens(255); delay(100); setLens(0); delay(100);
      setLens(255); delay(100); setLens(0); delay(100);
      setLens(255); delay(100); setLens(0); delay(100);
    */
  }
  else
  {
    sdInitialized = true;
    Serial.println("SD Card Mounted");

    // Check for an SD card
    uint8_t cardType = SD_MMC.cardType();
    if (cardType == CARD_NONE)
    {
      Serial.println("No SD card attached");
    }
    else
    {
      Serial.println(cardType);
    }
  }

  // Setting up LED
  ledcSetup(ledChannel, freq, pwmResolution);
  ledcAttachPin(ledPin, ledChannel);
  ledcSetup(lensChannel, freq, pwmResolution);
  ledcAttachPin(lensPin, lensChannel);

  // Test Hardware
  setLED(255);
  delay(100);
  setLED(0);

  setLens(255);
  delay(100);
  setLens(0);

  // only for Anglerfish if already focussed
  isTimelapseAnglerfish = device_pref.isTimelapse(); // set the global variable for the loop function

  if (isTimelapseAnglerfish)
  {
    int ledIntensity = 255;

    // override  camera settings
    sensor_t *s = esp_camera_sensor_get();
    s->set_framesize(s, FRAMESIZE_UXGA);
    s->set_quality(s, 10);

    // ONLY IF YOU WANT TO CAPTURE in ANGLERFISHMODE
    Serial.println("In timelapse mode.");
    // Save image to SD card
    uint32_t frame_index = device_pref.getFrameIndex() + 1;

    // save frame - eventually
    bool imageSaved = false;

    // FIXME: decide which method to use..
    device_pref.setFrameIndex(frame_index);
    imageSaved = doFocus(focuStackStepsizeAnglerfish, true, false, "anglerfish_" + String(frame_index));
    // imageSaved = snapPhoto("anglerfish_" + String(frame_index), ledIntensity);

    // also take Darkfield image
    // FIXME: This becomes obsolete nowimageSaved = snapPhoto("picture_LED1_"  + String(frame_index), 1, ledIntensity);
    /*if (imageSaved) {//FIXME: we should increase framenumber even if failed - since a corrupted file may lead to issues?
      device_pref.setFrameIndex(frame_index);
      }
    */

    // Sleep
    if (timelapseInterval==-1) timelapseInterval = 60; // do timelapse every minute if not set properly
    Serial.print("Sleeping for ");
    Serial.print(timelapseInterval);
    Serial.println(" s");
    static const uint64_t usPerSec = 1000000; // Conversion factor from microseconds to seconds
    esp_sleep_enable_timer_wakeup(timelapseInterval * usPerSec);
    myFile.close();
    SD_MMC.end(); // FIXME: may cause issues when file not closed? categoreis: LED/SD-CARD issues

    // After SD Card init? and after the Lens was used?
    // ATTENTIONN: DON'T USE ANY SD-CARD RELATED GPIO!!
    // set a wakeup pin so that we reset the Snow-white deepsleep and turn on the Wifi again: // FIXME: Makes sense?
    // esp_sleep_enable_ext0_wakeup(GPIO_NUM_15, 1); //=> GPIO: 4, level: 1
    // Setup interrupt on Touch Pad 3 (GPIO15)
    // touchAttachInterrupt(T3, callbackTouchpad, 40);
    // Configure Touchpad as wakeup source
    // esp_sleep_enable_touchpad_wakeup();
    // Ensure LED is switched off
    pinMode(4, OUTPUT);
    digitalWrite(4, LOW);
    gpio_hold_en(GPIO_NUM_4);
    gpio_deep_sleep_hold_en();
    esp_deep_sleep_start();
    return;
  }
  else
  {
    Serial.println("In refocusing mode. Connect to Wifi and go to 192.168.4.1enable once you're done with focusing.");
  }

  setLens(255);
  delay(100);
  setLens(0);

  // After SD Card init? and after the Lens was used?
  // ATTENTIONN: DON'T USE ANY SD - CARD RELATED GPIO!!
  // set a wakeup pin so that we reset the Snow-white deepsleep and turn on the Wifi again: // FIXME: Makes sense?
  // esp_sleep_enable_ext0_wakeup(GPIO_NUM_15, 1); //=> GPIO: 4, level: 1

  // Setup interrupt on Touch Pad 3 (GPIO15)
  // touchAttachInterrupt(T3, callbackTouchpad, 40);
  // Configure Touchpad as wakeup source
  // esp_sleep_enable_touchpad_wakeup();

  Serial.println("Set pin 12 high to wake up the ESP32 from deepsleep");

  // INIT WIFI
  // FIXME: The strategy should be:
  // 1. If starting up and no Wifi is set up (e.g. EEPROM settings empty), offer an AP (SSID: anglerfish)
  // 1.a. if settings are available => connect
  // 1.b. if wifi settings are available but not valid => offer AP
  // 2. Save Wifi settings from "captive portal" (not necessarily the Wifimanager.h - bloatware? ;) ) and connect
  // 3. continousyl check if wifi signal available, if not => restart
  if (isCaptivePortal)
  {
    // create a captive portal to connect to an existing WiFi AP with SSID/PW provided through the portal
    isFirstRun = false;
    autoconnectWifi(isFirstRun);
  }
  else
  {
    if (hostWifiAP)
    {
      // create an ESP32-based AP
      initWifiAP(mSSIDAP);
    }
    else
    {
      // connect to an existing Wifi /W SSID/PW
      joinWifi(mSSID, mPASSWORD);
    }
  }

  // INIT SPIFFS
  /*
    if (!SPIFFS.begin()) { // SPIFFS must be initialized before the web server, which depends on it
    Serial.println("Couldn't open SPIFFS!");
    }
  */

  // INIT Webserver

  // Start the camera and command server
  startCameraServer();

  // OTA
  startOTAServer();

  // initiliaze timer
  t_old = millis();

  // FIXME: This is just a tet to see if this works in general - the standalone application works; My guess: An issue with the image dimensions
  isInternetAvailable = Ping.ping("www.google.com", 3);
  if (!isInternetAvailable or hostWifiAP)
  {
    Serial.println("Ping failed -> we are not connected to the internet most likely!");
  }
  else
  {
    Serial.println("Ping succesful -> we are connected to the internet most likely!.");
    // Save image in Google Drive
    saveCapturedImageGDrive();
  }
}

void loop()
{

  // wait for incoming OTA client udpates
  OTAserver.handleClient(); // FIXME: the OTA, "REST API" and stream run on 3 different ports - cause: me not being able to merge OTA and REST; STREAM shuold be independent to have a non-blockig experience

  // reconnect wifi

  // Perform timelapse imaging
  if (timelapseInterval > 0 and ((millis() - t_old) > (1000 * timelapseInterval)))
  {
    // https://stackoverflow.com/questions/67090640/errors-while-interacting-with-microsd-card
    t_old = millis();
    uint32_t frame_index = device_pref.getFrameIndex() + 1;
    bool imageSaved = false;

    // turn on led
    setLED(ledValueOld);

    if (isAcquireStack)
    { // FIXME: We could have a switch in the GUI for this settig
      // acquire a stack
      // FIXME: decide which method to use..
      imageSaved = doFocus(5, true, false, "/anglerfish_" + String(frame_index));

      // switch off lens
      setLens(0); // save energy
    }
    else
    {
      // Acquire the image and save
      imageSaved = saveImage("/timelapse_" + String(uniqueID) + "_" + String(frame_index) + ".jpg", ledValueOld, lensValueOld);
    }

    if (true)
    { // FIXME: we should increase framenumber even if failed - since a corrupted file may lead to issues? (imageSaved) {
      device_pref.setFrameIndex(frame_index);
    };
    // turn off led
    setLED(0);
  }

  // checking for WIFI connection
  unsigned long currentTime = millis(); // number of milliseconds since the upload
  if (not hostWifiAP and (WiFi.status() != WL_CONNECTED) && (currentTime - previousCheckWifi >= delayReconnect))
  {
    Serial.print(millis());
    Serial.println("Reconnecting to WIFI network");
    WiFi.disconnect();
    // FIXME: Here we should offer an ACCESS POINT where we can supply credentials for SSID and PW
    if (0)
      WiFi.reconnect();
    else
      ESP.restart();
    previousCheckWifi = currentTime;
  }
}
