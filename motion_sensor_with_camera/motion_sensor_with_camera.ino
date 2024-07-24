// #include <stdint.h>
#define CAMERA_MODEL_AI_THINKER
#include "esp_camera.h"
#include "camera_pins.h"
#include <WiFi.h>
#include <WiFiClient.h>
#include <ESP32_FTPClient.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <PubSubClient.h>
#include "time.h"

// Pin declare Inputs

int motionSensorPin = 13;
// #define contactSensorPin  14

// Pin declare Outputs
int mqttLED = 2;
int lightPin = 4;

// Publish Topics
#define fileNameTopic "shatarko1003/hub/cameraHubFileName"
#define statusTopic "shatarko1003/hub/cameraHubOnlineStatus"
#define armModeTopicStatus "shatarko1003/hub/cameraHubArmModeStatus"
#define disArmModeTopicStatus "shatarko1003/hub/cameraHubDisArmModeStatus"
#define buttonTopicStatus "shatarko1003/hub/cameraHubTriggerButtonStatus"
#define motionSensorTopic "shatarko1003/hub/cameraHubMotionSensorTrigger"
#define lightStatusTopic "shatarko1003/hub/cameraHubLightStatusTopic"

// Subscribe Topic
#define modeTopic "shatarko1003/hub/cameraHubMode"
#define buttonTopic "shatarko1003/hub/cameraHubTriggerButton"
#define lightTopic "shatarko1003/hub/cameraHubLightTopic"

// Define Clients
WiFiClient espClient;
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);
PubSubClient client(espClient);

// FTP Credentials
char ftp_server[] = "180.92.224.170";
char ftp_user[] = "Shatarko";
char ftp_pass[] = "123456";
uint16_t ftp_port = 2121;
// FTP Connect
//  you can pass a FTP timeout and debbug mode on the last 2 arguments
ESP32_FTPClient ftp(ftp_server, ftp_port, ftp_user, ftp_pass, 5000, 2);

// Variables to save date and time

String formattedDate;
String dayStamp;
String timeStamp;

const char *ssid = "Rifat";
const char *password = "rifat674";

// MQTT Credentials
const char *mqtt_server = "*****";
// const char* mqtt_server = "test.mosquitto.org";
const char *mqtt_user_name = "***";
const char *mqtt_pass = "****";
const char *clientID = "****";
const char *willTopic = "*****";
const char *willMessage = "0";
// const char* modeTopic = "mode";
int willQoS = 0;
bool willRetain = true;
bool cleanSession = false;
String Mode = "";
String Button = "";

// Flag declearation

bool triggerButtonFlag = false;
bool statusFlag = false;
bool armModeFlag = false;
bool disArmModeFlag = false;
bool fileNameFlag = false;
bool motionSensorFlag = false;

bool lightButtonFlag = false;
bool lightButtonOffFlag = false;

char filename[32] = "";  // Variable declare for file name which should not exceed 32 characters

void config_camera() {
  /* Camera and sensor configuration setup */
  static camera_config_t cam;

  // cam.ledc_channel = LEDC_CHANNEL_0;
  // cam.ledc_timer = LEDC_TIMER_0;

  cam.pin_d0 = Y2_GPIO_NUM;
  cam.pin_d1 = Y3_GPIO_NUM;
  cam.pin_d2 = Y4_GPIO_NUM;
  cam.pin_d3 = Y5_GPIO_NUM;
  cam.pin_d4 = Y6_GPIO_NUM;
  cam.pin_d5 = Y7_GPIO_NUM;
  cam.pin_d6 = Y8_GPIO_NUM;
  cam.pin_d7 = Y9_GPIO_NUM;

  cam.pin_xclk = XCLK_GPIO_NUM;
  cam.pin_pclk = PCLK_GPIO_NUM;
  cam.pin_vsync = VSYNC_GPIO_NUM;
  cam.pin_href = HREF_GPIO_NUM;
  cam.pin_sscb_scl = SIOC_GPIO_NUM;
  cam.pin_sscb_sda = SIOD_GPIO_NUM;
  cam.pin_pwdn = PWDN_GPIO_NUM;
  cam.pin_reset = RESET_GPIO_NUM;
  cam.xclk_freq_hz = 20000000; /* SPI clock frequency set to 20MHz */
  cam.pixel_format = PIXFORMAT_JPEG;
  cam.frame_size = FRAMESIZE_SVGA;
  cam.jpeg_quality = 12;
  cam.fb_count = 1; /* Numbers of frame buffer at a time*/

  // camera init
  esp_err_t err = esp_camera_init(&cam);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }
}

void config_camera_sensor() {
  sensor_t *sensor = esp_camera_sensor_get();         // Get Acess to tweak camera settings
  sensor->set_brightness(sensor, 0);                  // -2 to 2
  sensor->set_contrast(sensor, 0);                    // -2 to 2
  sensor->set_saturation(sensor, 0);                  // -2 to 2
  sensor->set_special_effect(sensor, 0);              // 0 to 6 (0 - No Effect, 1 - Negative, 2 - Grayscale, 3 - Red Tint, 4 - Green Tint, 5 - Blue Tint, 6 - Sepia)
  sensor->set_whitebal(sensor, 1);                    // 0 = disable , 1 = enable
  sensor->set_awb_gain(sensor, 1);                    // 0 = disable , 1 = enable
  sensor->set_wb_mode(sensor, 0);                     // 0 to 4 - if awb_gain enabled (0 - Auto, 1 - Sunny, 2 - Cloudy, 3 - Office, 4 - Home)
  sensor->set_exposure_ctrl(sensor, 1);               // 0 = disable , 1 = enable
  sensor->set_aec2(sensor, 1);                        // 0 = disable , 1 = enable
  sensor->set_ae_level(sensor, 0);                    // -2 to 2
  sensor->set_aec_value(sensor, 1200);                // 0 to 1200
  sensor->set_gain_ctrl(sensor, 1);                   // 0 = disable , 1 = enable
  sensor->set_agc_gain(sensor, 30);                   // 0 to 30
  sensor->set_gainceiling(sensor, (gainceiling_t)6);  // 0 to 6
  sensor->set_bpc(sensor, 1);                         // 0 = disable , 1 = enable
  sensor->set_wpc(sensor, 1);                         // 0 = disable , 1 = enable
  sensor->set_raw_gma(sensor, 1);                     // 0 = disable , 1 = enable
  sensor->set_lenc(sensor, 1);                        // 0 = disable , 1 = enable
  sensor->set_hmirror(sensor, 0);                     // 0 = disable , 1 = enable
  sensor->set_vflip(sensor, 0);                       // 0 = disable , 1 = enable
  sensor->set_dcw(sensor, 1);                         // 0 = disable , 1 = enable
  sensor->set_colorbar(sensor, 0);                    // 0 = disable , 1 = enable
}

// MQTT Connect Function
void reconnect() {
  while (!client.connected()) {
    if (client.connect(clientID, mqtt_user_name, mqtt_pass, willTopic, willQoS, willRetain, willMessage)) {
      Serial.println("MQTT connected");
      // client.publish (statusTopic, "1",true);//************************************************************************
      statusFlag = true;
      // publishFunction();
      client.subscribe(modeTopic);
      client.subscribe(buttonTopic);
      client.subscribe(lightTopic);
    } else {
      Serial.println("failed, rc=");
      Serial.print(client.state());
      Serial.print("try again in 1 seconds");
      delay(1000);
    }
  }
}

void printLocalTime() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    Serial.println("Failed to obtain time");
    return;
  }
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
}

void publishFunction() {
  if (!client.connected()) {
    reconnect();
  } else {
    if (statusFlag == true) {
      client.publish(statusTopic, "1", true);
      Serial.println("status flag printed");
      statusFlag = false;
    }
    if (fileNameFlag == true) {
      client.publish(fileNameTopic, filename);
      Serial.println("filename printed");
      fileNameFlag = false;
    }
    if (armModeFlag == true) {
      client.publish(armModeTopicStatus, "1", true);
      Serial.println("Arm mode printed");
      armModeFlag = false;
    }
    if (disArmModeFlag == true) {
      client.publish(disArmModeTopicStatus, "0", true);
      Serial.println("dis arm mode printed");
      disArmModeFlag = false;
    }
    if (triggerButtonFlag == true) {
      client.publish(buttonTopicStatus, "1");
      Serial.println("trigger button printed");
      triggerButtonFlag = false;
    }

    if (motionSensorFlag == true) {
      client.publish(motionSensorTopic, "1");
      Serial.println("motion sensor printed");
      motionSensorFlag = false;
    }
    if (lightButtonFlag == true) {
      client.publish(lightStatusTopic, "1");
      Serial.println("light is on");
      lightButtonFlag = false;
    }
    if (lightButtonOffFlag == true) {
      client.publish(lightStatusTopic, "0");
      Serial.println("light is off");
      lightButtonOffFlag = false;
    }
  }
}

// FTP upload image function
// In this function firs it will established a FTP Connection
// get local time then it will create the file name
// then it will Initiate a file in the FTP server
// Create the file name wich is created in filename function
// Then write in that file
int FTP_uploadImage(int64_t t, unsigned char *pdata, unsigned int size) {
  // char filename[32] = "";
  Serial.print("FTP_uploadImage=");
  Serial.println(size);

  ftp.OpenConnection();

  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    Serial.println("Failed to obtain time");
    return -1;
  }
  sprintf(filename, "CAPTURE_%04d%02d%02d_%02d%02d%02d.jpg",
          timeinfo.tm_year + 1900,
          timeinfo.tm_mon + 1,
          timeinfo.tm_mday,
          timeinfo.tm_hour,
          timeinfo.tm_min,
          timeinfo.tm_sec);
  Serial.print(filename);

  // ftp.InitFile("Type A");
  // ftp.ChangeWorkDir("/public_html/zyro/gallery_gen/");
  ftp.InitFile("Type I");
  ftp.NewFile(filename);       // "capture.jpg");
  ftp.WriteData(pdata, size);  // Write data
  ftp.CloseFile();
  // client.publish(statusTopic, filename);

  ftp.CloseConnection();
  fileNameFlag = true;
  publishFunction();
  // client.publish(fileNameTopic, filename);//***************************

  return 0;
}

int timeout = 0;
int capture_ftpupload(void) {
  //    digitalWrite (12, HIGH);
  //    delay (3000);
  camera_fb_t *fb = NULL;
  esp_err_t res = ESP_OK;
  int64_t fr_start = esp_timer_get_time();

  fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");

    return ESP_FAIL;
    // digitalWrite (12, LOW);
  }

  // digitalWrite (12, LOW);

  FTP_uploadImage(fr_start, fb->buf, fb->len);

  esp_camera_fb_return(fb);

  return res;
}

void imageButton() {
  // if (triggerButton == true){
  digitalWrite(lightPin, HIGH);
  delay(200);
  capture_ftpupload();
  Serial.print("Button pressed");
  digitalWrite(lightPin, LOW);
  //}
}

// Callback function
void callback(char *topic, byte *payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("]");

  if (strstr(topic, buttonTopic)) {
    for (int i = 0; i < length; i++) {
      Serial.print((char)payload[i]);
    }
    Serial.println();
    // Switch on the LED if an 1 was received as first character
    if ((char)payload[0] == '1') {
      imageButton();
      triggerButtonFlag = true;
      publishFunction();
      // client.publish(buttonTopicStatus, "1");//********************************
    } else {
      triggerButtonFlag = false;

      // client.publish(buttonTopicStatus, "0");//********************************
    }
  } else if (strstr(topic, modeTopic)) {

    for (int i = 0; i < length; i++) {
      Serial.print((char)payload[i]);
      Mode = (char)payload[i];
      Serial.println("");
      Serial.println("value of Mode is");
      Serial.print(Mode);
      if (Mode == "1") {
        Serial.println("Arm Mode is ON");
        armModeFlag = true;
        publishFunction();
        // client.publish(modeTopicStatus,"1");//*************************
      } else if (Mode == "0") {
        Serial.println("DisArm Mode is ON");
        disArmModeFlag = true;
        publishFunction();
        // client.publish(modeTopicStatus,"0");//****************************
      }
    }
  } else if (strstr(topic, lightTopic)) {
    for (int i = 0; i < length; i++) {
      Serial.print((char)payload[i]);
    }
    Serial.println();
    if ((char)payload[0] == '1') {
      digitalWrite(lightPin, HIGH);
      lightButtonFlag = true;
      if (millis() > 10000) {
        digitalWrite(lightPin, LOW);
      }
      publishFunction();
      // client.publish(buttonTopicStatus, "1");//********************************
    } else if ((char)payload[0] == '0') {
      digitalWrite(lightPin, LOW);
      lightButtonOffFlag = true;

      // client.publish(buttonTopicStatus, "0");//********************************
    }
  }

  Serial.println();
}

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();
  pinMode(lightPin, OUTPUT);
  // Pin Mode Input
  pinMode(motionSensorPin, INPUT_PULLUP);
  // Pin Mode Output
  pinMode(mqttLED, OUTPUT);

  pinMode(mqttLED, LOW);
  pinMode(lightPin, LOW);
  delay(200);

  config_camera();

  config_camera_sensor();

  // wifi connection function
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");

  // MQTT Brocker connect functions
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  // Initialize a NTPClient to get time
  timeClient.begin();
  // Set offset time in seconds to adjust for your timezone, for example:
  // GMT +1 = 3600
  // GMT +8 = 28800
  // GMT -1 = -3600
  // GMT 0 = 0
  timeClient.setTimeOffset(21600);  // 21600 s is for Dhaka (GMT +6)

  while (!timeClient.update()) {
    timeClient.forceUpdate();
  }
  // The formattedDate comes with the following format:
  // 2018-05-28T16:00:13Z
  // We need to extract date and time
  formattedDate = timeClient.getFormattedTime();
  Serial.println(formattedDate);

  // Extract date
  int splitT = formattedDate.indexOf("T");
  dayStamp = formattedDate.substring(0, splitT);
  Serial.print("DATE: ");
  Serial.println(dayStamp);

  // Extract time
  timeStamp = formattedDate.substring(splitT + 1, formattedDate.length() - 1);
  Serial.print("HOUR: ");
  Serial.println(timeStamp);

  time_t t = timeClient.getEpochTime();

  // printf("Setting time: %s", asctime(&tm));
  struct timeval now = { .tv_sec = t };
  settimeofday(&now, NULL);

  timeClient.end();
}

// wifi_setup();

void loop() {

  if (!client.connected()) {
    Serial.print("mqtt disconnected");
    digitalWrite(mqttLED, LOW);
    reconnect();
  } else {
    digitalWrite(mqttLED, HIGH);
    // modeRunning();
    // digitalWrite(lightPin, HIGH);
    delay(500);
  }
  client.loop();
  // timeout += 1;
  // delay(100);
  //*************************************************************************HERE IS THE PROBLEM*****************

  if (digitalRead(motionSensorPin) == HIGH && Mode == "1") {
    delay(200);
    digitalWrite(lightPin, HIGH);
    // digitalWrite(mqttLED, LOW);
    // delay(200);
    // digitalWrite(mqttLED, HIGH);
    // delay (3000);
    //     digitalWrite (sirenPin, LOW);//*******************************
    motionSensorFlag = true;
    capture_ftpupload();
    Serial.print("Contact Sensor triggered");
    if (millis() > 10000) {
      digitalWrite(lightPin, LOW);
      //      digitalWrite (sirenPin, HIGH);
    }
  }
}
