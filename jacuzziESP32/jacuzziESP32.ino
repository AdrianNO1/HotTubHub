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

#include <esp_task_wdt.h>

#define AES_KEY(str) { \
  str[0], str[1], str[2], str[3], \
  str[4], str[5], str[6], str[7], \
  str[8], str[9], str[10], str[11], \
  str[12], str[13], str[14], str[15] \
}

const char* ssid = WIFI_SSID;
const char* password = WIFI_PASSWORD;

const char* mqttBroker = "broker.emqx.io";
const int   mqttPort   = 1883;
const char* deviceId   = "jcz_001";
const byte  aes_key[16] = AES_KEY(AES_KEY_STRING);
const char* topicPrefix = "secure_jacuzzi/jcz_001/";

char topicTemperature[64];
char topicStatus[64];
char topicTargetTemp[64];
char topicInitialReq[64];

// WDT Timeout in seconds
const int WDT_TIMEOUT = 10;

// Reconnection timers
unsigned long lastWiFiReconnectAttempt = 0;
unsigned long lastMQTTReconnectAttempt = 0;
const unsigned long RECONNECT_INTERVAL = 5000;

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

const unsigned long SENSOR_INTERVAL = 1000UL;
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

// Helper to get hex char
char nibbleToHex(byte nibble) {
  return (nibble < 10) ? ('0' + nibble) : ('A' + (nibble - 10));
}

void byteArrayToHexString(byte* buffer, int len, char* output) {
  for (int i = 0; i < len; i++) {
    output[i * 2]     = nibbleToHex((buffer[i] >> 4) & 0x0F);
    output[i * 2 + 1] = nibbleToHex(buffer[i] & 0x0F);
  }
  output[len * 2] = '\0';
}

void hexStringToByteArray(const char* hex, byte* bytes, int len) {
  for (int i = 0; i < len; i++) {
    char high = hex[i * 2];
    char low  = hex[i * 2 + 1];
    byte hVal = (high >= '0' && high <= '9') ? high - '0' : (high >= 'A' && high <= 'F') ? high - 'A' + 10 : high - 'a' + 10;
    byte lVal = (low >= '0' && low <= '9') ? low - '0' : (low >= 'A' && low <= 'F') ? low - 'A' + 10 : low - 'a' + 10;
    bytes[i] = (hVal << 4) | lVal;
  }
}

// Encrypts plaintext into base64 outputBuffer. 
// outputBuffer must be large enough: base64_enc_len(padded_len) + 1
void encrypt(const char* plaintext, byte iv[], char* outputBuffer) {
  int plainLen = strlen(plaintext);
  int padLen   = 16 - (plainLen % 16);
  int totalLen = plainLen + padLen;

  // Reduced buffer size from 512 to 300 based on max message analysis
  // Max expected plaintext is ~256 bytes.
  byte plainBytes[300]; 
  if (totalLen > 300) {
    totalLen = 300; 
  }

  memcpy(plainBytes, plaintext, plainLen);
  for (int i = plainLen; i < totalLen; i++) plainBytes[i] = padLen;

  byte cipherBytes[300];
  byte iv_enc[N_BLOCK];
  memcpy(iv_enc, iv, N_BLOCK);
  aesLib.encrypt(plainBytes, totalLen, cipherBytes, aes_key, 128, iv_enc);

  base64_encode(outputBuffer, (char*)cipherBytes, totalLen);
}

// Decrypts base64 ciphertext into outputBuffer
void decrypt(const char* ciphertext_base64, byte iv[], char* outputBuffer) {
  int inputLen = strlen(ciphertext_base64);
  int cipherLen = base64_dec_len((char*)ciphertext_base64, inputLen);
  
  byte cipherBytes[300];
  if (cipherLen > 300) cipherLen = 300; // Safety cap

  base64_decode((char*)cipherBytes, (char*)ciphertext_base64, inputLen);

  byte decryptedBytes[300];
  byte iv_dec[N_BLOCK];
  memcpy(iv_dec, iv, N_BLOCK);
  aesLib.decrypt(cipherBytes, cipherLen, decryptedBytes, aes_key, 128, iv_dec);

  int padLen = decryptedBytes[cipherLen - 1];
  // Basic validation of padLen
  if (padLen <= 0 || padLen > 16) padLen = 0; 
  
  int actualLen = cipherLen - padLen;
  if (actualLen < 0) actualLen = 0;

  memcpy(outputBuffer, decryptedBytes, actualLen);
  outputBuffer[actualLen] = '\0';
}

void generateRandomIV(byte iv[]) {
  for (int i = 0; i < N_BLOCK; i++) iv[i] = random(0, 256);
}


void mqttPublishEncrypted(const char* topic, DynamicJsonDocument &doc) {
  // Reduced buffer sizes to save stack
  char plaintext[256]; 
  size_t len = serializeJson(doc, plaintext, sizeof(plaintext));
  if (len >= sizeof(plaintext)) return; // Truncated, do not send

  byte iv[N_BLOCK];
  generateRandomIV(iv);
  
  char ciphertext[512]; // Base64 expansion + padding. 256 bytes -> ~350 base64 chars. 512 is safe.
  encrypt(plaintext, iv, ciphertext);
  
  char ivHex[33];
  byteArrayToHexString(iv, N_BLOCK, ivHex);

  DynamicJsonDocument encryptedDoc(768);
  encryptedDoc["iv"]        = ivHex;
  encryptedDoc["ciphertext"] = ciphertext;

  char encryptedMessage[768];
  serializeJson(encryptedDoc, encryptedMessage, sizeof(encryptedMessage));
  
  mqttClient.publish(topic, encryptedMessage);
}


void enableHeater();
void disableHeater();
void enableBubbles();
void disableBubbles();
void sendTemperatureUpdate();
void sendError(const char* msg);
void updateSensorsAndControl();


unsigned long lastTargetUpdateTimestamp = 0;

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  // Use char array for topic comparison to avoid String allocation
  // Direct deserialization from payload buffer to avoid String copy loop

  DynamicJsonDocument encDoc(768);
  if (deserializeJson(encDoc, payload, length) != DeserializationError::Ok) {
    Serial.println("Failed to parse encrypted JSON");
    return;
  }

  const char* ivHex = encDoc["iv"];
  const char* ciphertext = encDoc["ciphertext"];
  
  if (!ivHex || !ciphertext) return;

  byte iv[N_BLOCK];
  hexStringToByteArray(ivHex, iv, N_BLOCK);

  char decryptedPayload[300];
  decrypt(ciphertext, iv, decryptedPayload);
  
  DynamicJsonDocument doc(512);
  if (deserializeJson(doc, decryptedPayload) != DeserializationError::Ok) {
    Serial.println("Failed to parse decrypted JSON");
    return;
  }

  if (strcmp(topic, topicInitialReq) == 0) {
    sendTemperatureUpdate();
  } else if (strcmp(topic, topicTargetTemp) == 0) {
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
  if (WiFi.status() == WL_CONNECTED) return;

  // Non-blocking WiFi connection
  if (millis() - lastWiFiReconnectAttempt >= RECONNECT_INTERVAL) {
    lastWiFiReconnectAttempt = millis();
    Serial.println("Checking WiFi connection...");
    
    // Only disconnect and begin if we are not already connecting
    // WiFi.status() != WL_CONNECTED is already true here
    // We can check if we have an IP or if status is WL_DISCONNECTED to be more specific,
    // but simply calling begin() again is usually okay if enough time has passed.
    // However, to be less disruptive, we can check:
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi not connected. Reconnecting...");
        WiFi.disconnect();
        WiFi.begin(ssid, password);
    }
  }
}

void connectMQTT() {
  if (WiFi.status() != WL_CONNECTED) return;
  if (mqttClient.connected()) return;

  if (millis() - lastMQTTReconnectAttempt >= RECONNECT_INTERVAL) {
    lastMQTTReconnectAttempt = millis();
    Serial.print("Attempting MQTT connection...");
    
    if (mqttClient.connect(deviceId)) {
      Serial.println("connected");
      mqttClient.subscribe(topicTargetTemp);
      mqttClient.subscribe(topicInitialReq);
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again later");
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
    
    char errMsg[64];
    snprintf(errMsg, sizeof(errMsg), "Heater sensor %s", heaterErrorType.c_str());
    sendError(errMsg);
  } else {
    heaterSensorError = false;
    heaterErrorType   = "";
  }

  currentWaterTemp = sensors.getTempC(waterSensor);
  if (currentWaterTemp == DEVICE_DISCONNECTED_C || currentWaterTemp <= -126 || currentWaterTemp > 80) {
    waterSensorError = true;
    waterErrorType   = (currentWaterTemp == DEVICE_DISCONNECTED_C) ? "DISCONNECTED" : "INVALID";
    currentWaterTemp = -127.0;
    disableHeater();

    char errMsg[64];
    snprintf(errMsg, sizeof(errMsg), "Water sensor %s", waterErrorType.c_str());
    sendError(errMsg);
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
  if (millis() - lastHeaterSwitch >= heaterMinSwitchTime * 1000UL && !heaterSensorError && !waterSensorError) {
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

void sendError(const char* errorMessage) {
  DynamicJsonDocument doc(256);
  doc["deviceId"] = deviceId;
  doc["status"]   = "error";
  doc["message"]  = errorMessage;
  mqttPublishEncrypted(topicStatus, doc);
  Serial.println(errorMessage);
}


void setup() {
  Serial.begin(115200);
  randomSeed(analogRead(0) + millis());

  // Initialize Topic Strings
  snprintf(topicTemperature, sizeof(topicTemperature), "%stemperature", topicPrefix);
  snprintf(topicStatus, sizeof(topicStatus), "%sstatus", topicPrefix);
  snprintf(topicTargetTemp, sizeof(topicTargetTemp), "%starget_temperature", topicPrefix);
  snprintf(topicInitialReq, sizeof(topicInitialReq), "%sinitial_request", topicPrefix);

  // Initialize WDT
  Serial.println("Initializing WDT...");
  esp_task_wdt_config_t wdt_config = {
      .timeout_ms = WDT_TIMEOUT * 1000,
      .idle_core_mask = 0,
      .trigger_panic = true
  };
  esp_task_wdt_init(&wdt_config);
  esp_task_wdt_add(NULL); // Add current thread to WDT watch

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

  // Initial WiFi setup (non-blocking attempt)
  WiFi.begin(ssid, password);
  WiFi.setSleep(false);
  
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");

  mqttClient.setServer(mqttBroker, mqttPort);
  mqttClient.setCallback(mqttCallback);
  mqttClient.setBufferSize(1024);
  
  // Note: We don't block for MQTT here. Loop will handle it.
}

void loop() {
  // Pet the dog
  esp_task_wdt_reset();

  connectWiFi();
  connectMQTT();
  
  if (mqttClient.connected()) {
    mqttClient.loop();
  }

  updateSensorsAndControl();
  scheduleAutomaticBubbles();
  checkBubbleTimer();

  static unsigned long lastTempPub = 0;
  if (millis() - lastTempPub >= 5000UL) {
    if (mqttClient.connected()) {
      sendTemperatureUpdate();
    }
    lastTempPub = millis();
  }
}
