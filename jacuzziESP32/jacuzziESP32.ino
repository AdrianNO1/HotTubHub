#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <AESLib.h>
#include <base64.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <EEPROM.h>
#include <time.h>
#include <math.h>
#include "secrets.h"

#define AES_KEY(str) { \
  str[0], str[1], str[2], str[3], \
  str[4], str[5], str[6], str[7], \
  str[8], str[9], str[10], str[11], \
  str[12], str[13], str[14], str[15] \
}

const char* ssid = WIFI_SSID;
const char* password = WIFI_PASSWORD;

const char* mqttBroker = "192.168.1.134";
const int   mqttPort   = 1883;
const char* deviceId   = "jcz_001";
const byte  aes_key[16] = AES_KEY(AES_KEY_STRING);
const char* topicPrefix = "secure_jacuzzi/jcz_001/";

String topicTemperature   = String(topicPrefix) + "temperature";
String topicStatus        = String(topicPrefix) + "status";
String topicTargetTemp    = String(topicPrefix) + "target_temperature";
String topicInitialReq    = String(topicPrefix) + "initial_request";

// 1-Wire temperature sensors
const int temperatureSensorsPin = 33;
OneWire oneWire(temperatureSensorsPin);
DallasTemperature sensors(&oneWire);

// DS18B20 sensor addresses
DeviceAddress heaterSensor = {0x28, 0x31, 0x93, 0x56, 0x00, 0x00, 0x00, 0xEC}; // Sensor in heater
DeviceAddress waterSensor  = {0x28, 0xFD, 0x77, 0x58, 0x00, 0x00, 0x00, 0xDE}; // Sensor in water

// LOW -> ON, HIGH -> OFF
const int HEATER_CONTROL_PIN  = 27;
const int BUBBLES_CONTROL_PIN = 26;

const float SAFETY_MAX_TEMP = 55.0;
const float MIN_TEMP        = 0.0;
const float MAX_TEMP        = 50.0;


const unsigned long heaterMinSwitchTime = 30UL;
unsigned long lastHeaterSwitch = 0;

const unsigned long SENSOR_INTERVAL = 2000UL;
unsigned long lastSensorRead = 0;

const int TARGET_TEMP_ADDR = 0;

float currentWaterTemp   = -127.0;
float currentHeaterTemp  = -127.0;
float targetTemp         = 5.0;
bool  heaterEnabled      = false;
bool  bubblesEnabled     = false;

bool heaterSensorError   = false;
bool waterSensorError    = false;
String heaterErrorType   = "";
String waterErrorType    = "";

WiFiClient    espClient;
PubSubClient  mqttClient(espClient);
AESLib        aesLib;
const char base64_chars[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

String encrypt(String plaintext, byte iv[]) {
  int plainLen = plaintext.length();
  int padLen   = 16 - (plainLen % 16);
  int totalLen = plainLen + padLen;

  byte plainBytes[totalLen];
  plaintext.getBytes(plainBytes, plainLen + 1);
  for (int i = plainLen; i < totalLen; i++) plainBytes[i] = padLen;

  byte cipherBytes[totalLen];
  byte iv_enc[N_BLOCK];
  memcpy(iv_enc, iv, N_BLOCK);
  aesLib.encrypt(plainBytes, totalLen, cipherBytes, aes_key, 128, iv_enc);

  char base64Output[base64_enc_len(totalLen) + 1];
  base64_encode(base64Output, (char*)cipherBytes, totalLen);
  return String(base64Output);
}

String decrypt(String ciphertext_base64, byte iv[]) {
  int cipherLen = base64_dec_len((char*)ciphertext_base64.c_str(), ciphertext_base64.length());
  byte cipherBytes[cipherLen];
  base64_decode((char*)cipherBytes, (char*)ciphertext_base64.c_str(), ciphertext_base64.length());

  byte decryptedBytes[cipherLen];
  byte iv_dec[N_BLOCK];
  memcpy(iv_dec, iv, N_BLOCK);
  aesLib.decrypt(cipherBytes, cipherLen, decryptedBytes, aes_key, 128, iv_dec);

  int padLen   = decryptedBytes[cipherLen - 1];
  int actualLen = cipherLen - padLen;
  decryptedBytes[actualLen] = '\0';

  return String((char*)decryptedBytes);
}

void generateRandomIV(byte iv[]) {
  for (int i = 0; i < N_BLOCK; i++) iv[i] = random(0, 256);
}

String byteArrayToHexString(byte* buffer, int len) {
  String hexStr = "";
  for (int i = 0; i < len; i++) {
    if (buffer[i] < 16) hexStr += "0";
    hexStr += String(buffer[i], HEX);
  }
  return hexStr;
}

void hexStringToByteArray(String hex, byte* bytes, int len) {
  for (int i = 0; i < len; i++) {
    bytes[i] = strtoul(hex.substring(i * 2, i * 2 + 2).c_str(), NULL, 16);
  }
}

void mqttPublishEncrypted(String topic, DynamicJsonDocument &doc) {
  String plaintext;
  serializeJson(doc, plaintext);

  byte iv[N_BLOCK];
  generateRandomIV(iv);
  String ciphertext = encrypt(plaintext, iv);
  String ivHex      = byteArrayToHexString(iv, N_BLOCK);

  DynamicJsonDocument encryptedDoc(512);
  encryptedDoc["iv"]        = ivHex;
  encryptedDoc["ciphertext"] = ciphertext;

  String encryptedMessage;
  serializeJson(encryptedDoc, encryptedMessage);
  mqttClient.publish(topic.c_str(), encryptedMessage.c_str());
}


void enableHeater();
void disableHeater();
void enableBubbles();
void disableBubbles();
void sendTemperatureUpdate();
void sendError(String msg);
void updateSensorsAndControl();


unsigned long lastTargetUpdateTimestamp = 0;

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String topicStr   = String(topic);
  String payloadStr = "";
  for (unsigned int i = 0; i < length; i++) payloadStr += (char)payload[i];

  DynamicJsonDocument encDoc(512);
  if (deserializeJson(encDoc, payloadStr) != DeserializationError::Ok) {
    Serial.println("Failed to parse encrypted JSON");
    return;
  }

  String ivHex     = encDoc["iv"];
  String ciphertext = encDoc["ciphertext"];
  byte iv[N_BLOCK];
  hexStringToByteArray(ivHex, iv, N_BLOCK);

  String decryptedPayload = decrypt(ciphertext, iv);
  DynamicJsonDocument doc(512);
  if (deserializeJson(doc, decryptedPayload) != DeserializationError::Ok) {
    Serial.println("Failed to parse decrypted JSON");
    return;
  }

  if (topicStr == topicInitialReq) {
    sendTemperatureUpdate();
  } else if (topicStr == topicTargetTemp) {
    unsigned long messageTimestamp = doc["timestamp"] | 0;
    if (messageTimestamp < lastTargetUpdateTimestamp) return;

    float newTarget = doc["value"];
    if (newTarget >= MIN_TEMP && newTarget <= MAX_TEMP) {
      targetTemp                = newTarget;
      lastTargetUpdateTimestamp = messageTimestamp;

      // Persist
      EEPROM.put(TARGET_TEMP_ADDR, targetTemp);
      EEPROM.commit();

      // Optional bubbles parameter
      bool newBubbles = doc["bubbles"] | false;
      if (newBubbles != bubblesEnabled) {
        if (newBubbles) enableBubbles();
        else             disableBubbles();
      }

      DynamicJsonDocument status(256);
      status["deviceId"] = deviceId;
      status["type"]     = "target_temp_update";
      status["status"]   = "ok";
      status["message"]  = newTarget;
      mqttPublishEncrypted(topicStatus, status);

      Serial.println("Target temperature updated to " + String(newTarget));
    } else {
      DynamicJsonDocument status(256);
      status["deviceId"] = deviceId;
      status["type"]     = "target_temp_update";
      status["status"]   = "error";
      status["message"]  = "Temperature out of valid range (0-50°C)";
      mqttPublishEncrypted(topicStatus, status);
      Serial.println("Error: Temperature out of valid range");
    }
  }
}


void connectWiFi() {
  Serial.print("Connecting to WiFi");
  WiFi.begin(ssid, password);
  WiFi.setSleep(false);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print('.');
  }
  Serial.println("\nWiFi connected: " + WiFi.localIP().toString());
}

void connectMQTT() {
  while (!mqttClient.connected()) {
    Serial.print("Connecting to MQTT...");
    if (mqttClient.connect(deviceId)) {
      Serial.println("connected");
      mqttClient.subscribe(topicTargetTemp.c_str());
      mqttClient.subscribe(topicInitialReq.c_str());
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" retrying in 5s");
      delay(5000);
    }
  }
}


void updateSensorsAndControl() {
  if (millis() - lastSensorRead < SENSOR_INTERVAL) return;
  lastSensorRead = millis();

  sensors.requestTemperatures();

  currentHeaterTemp = sensors.getTempC(heaterSensor);
  if (currentHeaterTemp == DEVICE_DISCONNECTED_C || currentHeaterTemp <= -126 || currentHeaterTemp > 125) {
    heaterSensorError = true;
    heaterErrorType   = (currentHeaterTemp == DEVICE_DISCONNECTED_C) ? "DISCONNECTED" : "INVALID";
    currentHeaterTemp = -127.0;
    disableHeater();
    sendError("Heater sensor " + heaterErrorType);
  } else {
    heaterSensorError = false;
    heaterErrorType   = "";
  }

  currentWaterTemp = sensors.getTempC(waterSensor);
  if (currentWaterTemp == DEVICE_DISCONNECTED_C || currentWaterTemp <= -126 || currentWaterTemp > 80) {
    waterSensorError = true;
    waterErrorType   = (currentWaterTemp == DEVICE_DISCONNECTED_C) ? "DISCONNECTED" : "INVALID";
    currentWaterTemp = -127.0;
    sendError("Water sensor " + waterErrorType);
  } else {
    waterSensorError = false;
    waterErrorType   = "";
  }

  // Safety cut-off
  if (currentHeaterTemp > SAFETY_MAX_TEMP) {
    disableHeater();
    sendError("SAFETY SHUTOFF: Heater too hot!");
  }

  // Heater control logic with debounce
  if (millis() - lastHeaterSwitch >= heaterMinSwitchTime * 1000UL && !heaterSensorError) {
    float diff = targetTemp - currentWaterTemp; // positive if we are below target
    if (targetTemp == 0 && heaterEnabled) {
      disableHeater();
    } else if (diff >= 0.1 && !heaterEnabled && currentHeaterTemp <= SAFETY_MAX_TEMP && targetTemp != 0) {
      enableHeater();
    } else if ((diff <= -0.1 || currentHeaterTemp > SAFETY_MAX_TEMP) && heaterEnabled) {
      disableHeater();
    }
  }
}


void enableHeater() {
  digitalWrite(HEATER_CONTROL_PIN, LOW);
  heaterEnabled = true;
  lastHeaterSwitch = millis();
  Serial.println("HEATER ON");
}

void disableHeater() {
  digitalWrite(HEATER_CONTROL_PIN, HIGH);
  heaterEnabled = false;
  lastHeaterSwitch = millis();
  Serial.println("HEATER OFF");
}

void enableBubbles() {
  digitalWrite(BUBBLES_CONTROL_PIN, LOW);
  bubblesEnabled = true;
  Serial.println("BUBBLES ON");
}

void disableBubbles() {
  digitalWrite(BUBBLES_CONTROL_PIN, HIGH);
  bubblesEnabled = false;
  Serial.println("BUBBLES OFF");
}


bool bubbleScheduled = false;
unsigned long bubbleStartMillis = 0;
bool scheduledBubblesRun[3] = {false, false, false};

void scheduleAutomaticBubbles() {
  time_t now = time(nullptr);
  struct tm* timeinfo = localtime(&now);
  int hour   = timeinfo->tm_hour;
  int minute = timeinfo->tm_min;

  int times[3] = {2, 10, 18};
  for (int i = 0; i < 3; i++) {
    if (hour == times[i] && minute == 0 && !scheduledBubblesRun[i]) {
      enableBubbles();
      bubbleStartMillis   = millis();
      bubbleScheduled     = true;
      scheduledBubblesRun[i] = true;
    } else if (hour != times[i] || minute != 0) {
      scheduledBubblesRun[i] = false;
    }
  }
}

void checkBubbleTimer() {
  if (bubbleScheduled && millis() - bubbleStartMillis >= 5UL * 60UL * 1000UL) {
    disableBubbles();
    bubbleScheduled = false;
  }
}


void sendTemperatureUpdate() {
  DynamicJsonDocument doc(256);
  doc["deviceId"]   = deviceId;
  doc["value"]      = currentWaterTemp;
  doc["heaterValue"] = currentHeaterTemp;
  doc["target"]     = targetTemp;
  doc["isHeating"]  = heaterEnabled;
  doc["bubbles"]    = bubblesEnabled;
  mqttPublishEncrypted(topicTemperature, doc);
  Serial.println("Published temperature update");
}

void sendError(String errorMessage) {
  DynamicJsonDocument doc(256);
  doc["deviceId"] = deviceId;
  doc["status"]   = "error";
  doc["message"]  = errorMessage;
  mqttPublishEncrypted(topicStatus, doc);
  Serial.println("Error: " + errorMessage);
}


void setup() {
  Serial.begin(115200);
  randomSeed(analogRead(0) + millis());

  pinMode(temperatureSensorsPin, INPUT);

  // Hardware init
  pinMode(HEATER_CONTROL_PIN, OUTPUT);
  digitalWrite(HEATER_CONTROL_PIN, HIGH); // off
  pinMode(BUBBLES_CONTROL_PIN, OUTPUT);
  digitalWrite(BUBBLES_CONTROL_PIN, HIGH); // off

  sensors.begin();
  sensors.setResolution(heaterSensor, 12);
  sensors.setResolution(waterSensor, 12);
  sensors.setWaitForConversion(false);

  Serial.printf("1-Wire parasite power: %s\n", sensors.isParasitePowerMode() ? "YES" : "NO");
  Serial.printf("Device count: %d\n", sensors.getDeviceCount());
  Serial.printf("Heater sensor connected: %s\n", sensors.isConnected(heaterSensor) ? "YES" : "NO");
  Serial.printf("Water  sensor connected: %s\n", sensors.isConnected(waterSensor) ? "YES" : "NO");
  
  EEPROM.begin(sizeof(float));
  EEPROM.get(TARGET_TEMP_ADDR, targetTemp);
  if (isnan(targetTemp) || targetTemp < MIN_TEMP || targetTemp > MAX_TEMP) {
    targetTemp = 5.0;
  }

  connectWiFi();
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");

  mqttClient.setServer(mqttBroker, mqttPort);
  mqttClient.setCallback(mqttCallback);
  mqttClient.setBufferSize(1024);
  connectMQTT();

  // Initial publish so cloud has baseline
  sendTemperatureUpdate();
}

void loop() {
  if (!mqttClient.connected()) connectMQTT();
  mqttClient.loop();

  updateSensorsAndControl();
  scheduleAutomaticBubbles();
  checkBubbleTimer();

  static unsigned long lastTempPub = 0;
  if (millis() - lastTempPub >= 5000UL) {
    sendTemperatureUpdate();
    lastTempPub = millis();
  }
}
