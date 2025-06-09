#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <EEPROM.h>
#include <AESLib.h>
#include <base64.h>
#include <time.h>
#include "secrets.h"

#define AES_KEY(str) { \
  str[0], str[1], str[2], str[3], \
  str[4], str[5], str[6], str[7], \
  str[8], str[9], str[10], str[11], \
  str[12], str[13], str[14], str[15] \
}

const char* ssid = WIFI_SSID;
const char* password = WIFI_PASSWORD;

const char* mqttBroker = "test.mosquitto.org";
const int mqttPort = 1883;
const char* deviceId = "jcz_001";
const byte aes_key[16] = AES_KEY(AES_KEY_STRING);
const char* topicPrefix = "secure_jacuzzi/jcz_001/";

String topicTemperature = String(topicPrefix) + "temperature";
String topicStatus = String(topicPrefix) + "status";
String topicTargetTemp = String(topicPrefix) + "target_temperature";
String topicInitialRequest = String(topicPrefix) + "initial_request";

const float MIN_TEMP = 0.0;
const float MAX_TEMP = 50.0;

const int TARGET_TEMP_ADDR = 0;

const unsigned long heaterMinSwitchTime = 30;
unsigned long heaterSwitchTime = 0;
unsigned long lastHeaterSwitch = 0;
unsigned long lastTargetUpdateTimestamp = 0;
unsigned long lastSerialRead = 0;
const unsigned long serialTimeout = 30000;

float currentWaterTemp = -127.0;
float currentHeaterTemp = -127.0;
float targetTemp = MIN_TEMP;
bool heaterEnabled = false;
bool waterSensorError = false;
bool heaterSensorError = false;
String waterErrorType = "";
String heaterErrorType = "";
bool bubblesEnabled = false;
bool waitingForBubblesAck = false;
bool desiredBubblesState = false;
unsigned long bubblesAckRequestTime = 0;
bool bubbleScheduled = false;
unsigned long bubbleStartMillis = 0;
bool scheduledBubblesRun[3] = {false, false, false};
int bubbleRetryCount = 0;
const int BUBBLE_RETRY_LIMIT = 10;
int heaterRetryCount = 0;
const int HEATER_RETRY_LIMIT = 10;
unsigned long mqttNackErrorTime = 0;
const unsigned long mqttErrorCooldown = 2000;

WiFiClient espClient;
PubSubClient mqttClient(espClient);

const char base64_chars[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

AESLib aesLib;

String encrypt(String plaintext, byte iv[]) {
  int plainLen = plaintext.length();
  int padLen = 16 - (plainLen % 16);
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

  int padLen = decryptedBytes[cipherLen - 1];
  int actualLen = cipherLen - padLen;
  decryptedBytes[actualLen] = '\0';

  return String((char*)decryptedBytes);
}

void generateRandomIV(byte iv[]) {
  for (int i = 0; i < N_BLOCK; i++) iv[i] = random(0, 256);
}

String byteArrayToHexString(byte *buffer, int len) {
  String hexStr = "";
  for (int i = 0; i < len; i++) {
    if (buffer[i] < 16) hexStr += "0";
    hexStr += String(buffer[i], HEX);
  }
  return hexStr;
}

void hexStringToByteArray(String hex, byte* bytes, int len) {
  for (int i = 0; i < len; i++) {
    bytes[i] = strtoul(hex.substring(i*2, i*2+2).c_str(), NULL, 16);
  }
}

void mqttPublishEncrypted(String topic, DynamicJsonDocument &doc) {
  String plaintext;
  serializeJson(doc, plaintext);

  byte iv[N_BLOCK];
  generateRandomIV(iv);
  String ciphertext = encrypt(plaintext, iv);
  String ivHex = byteArrayToHexString(iv, N_BLOCK);

  DynamicJsonDocument encryptedDoc(512);
  encryptedDoc["iv"] = ivHex;
  encryptedDoc["ciphertext"] = ciphertext;

  String encryptedMessage;
  serializeJson(encryptedDoc, encryptedMessage);
  mqttClient.publish(topic.c_str(), encryptedMessage.c_str());
}

bool waitingForAck = false;
bool desiredHeaterState = false;
unsigned long ackRequestTime = 0;
const unsigned long ackTimeout = 5000;

void setup() {
  randomSeed(analogRead(0) + millis());
  Serial.begin(115200);
  delay(100);
  
  EEPROM.begin(512);
  
  connectWiFi();
  
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");
  Serial.print("Waiting for NTP time sync");
  while (time(nullptr) < 8 * 3600) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  
  mqttClient.setServer(mqttBroker, mqttPort);
  mqttClient.setCallback(mqttCallback);
  mqttClient.setBufferSize(1024);
  
  loadTargetTemp();
  
  connectMQTT();
}

void loop() {
  if (!mqttClient.connected()) {
    connectMQTT();
  }
  mqttClient.loop();
  
  readSerialData();
  
  if (waitingForAck && millis() - ackRequestTime > ackTimeout) {
    if (heaterRetryCount < HEATER_RETRY_LIMIT) {
      heaterRetryCount++;
      if (desiredHeaterState) {
        Serial.println("CMD:ON");
      } else {
        Serial.println("CMD:OFF");
      }
      ackRequestTime = millis();
    } else {
      sendError("NO ACK RECEIVED FOR HEATER COMMAND. CHECK THE ARDUINO IMMEDIATELY!!!");
      waitingForAck = false;
      heaterRetryCount = 0;
    }
  }
  if (waitingForBubblesAck && millis() - bubblesAckRequestTime > ackTimeout) {
    if (bubbleRetryCount < BUBBLE_RETRY_LIMIT) {
      bubbleRetryCount++;
      if (desiredBubblesState) {
        Serial.println("CMD:BUBBLES_ON");
      } else {
        Serial.println("CMD:BUBBLES_OFF");
      }
      bubblesAckRequestTime = millis();
    } else {
      sendError("NO ACK RECEIVED FOR BUBBLES COMMAND. CHECK THE ARDUINO IMMEDIATELY!!!");
      waitingForBubblesAck = false;
      bubbleRetryCount = 0;
    }
  }
  
  controlHeater();
  
  static unsigned long lastTempUpdate = 0;
  if (millis() - lastTempUpdate >= 5000) {
    if (millis() - lastSerialRead > serialTimeout) {
      sendError("ESP not receiving data from Arduino");
    }
    lastTempUpdate = millis();
    sendTemperatureUpdate();
  }

  scheduleAutomaticBubbles();
  checkBubbleTimer();
}

void connectWiFi() {
  Serial.print("Connecting to WiFi");
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println("\nWiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void connectMQTT() {
  while (!mqttClient.connected()) {
    Serial.print("Connecting to MQTT...");
    
    if (mqttClient.connect(deviceId)) {
      Serial.println("connected");
      
      mqttClient.subscribe(topicTargetTemp.c_str());
      mqttClient.subscribe(topicInitialRequest.c_str());
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" retrying in 5 seconds");
      delay(5000);
    }
  }
}

String base64Encode(const uint8_t* data, size_t length) {
  String encoded = "";
  int i = 0;
  int j = 0;
  uint8_t char_array_3[3];
  uint8_t char_array_4[4];

  while (length--) {
    char_array_3[i++] = *(data++);
    if (i == 3) {
      char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
      char_array_4[1] = ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
      char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);
      char_array_4[3] = char_array_3[2] & 0x3f;

      for(i = 0; i < 4; i++)
        encoded += base64_chars[char_array_4[i]];
      i = 0;
    }
  }

  if (i) {
    for(j = i; j < 3; j++)
      char_array_3[j] = '\0';

    char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
    char_array_4[1] = ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
    char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);

    for (j = 0; j < i + 1; j++)
      encoded += base64_chars[char_array_4[j]];

    while(i++ < 3)
      encoded += '=';
  }

  return encoded;
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String topicStr = String(topic);
  String payloadStr = "";
  for (unsigned int i = 0; i < length; i++) payloadStr += (char)payload[i];

  DynamicJsonDocument encryptedDoc(512);
  DeserializationError error = deserializeJson(encryptedDoc, payloadStr);
  if (error) {
    Serial.println("Failed to parse encrypted JSON");
    return;
  }

  String ivHex = encryptedDoc["iv"];
  String ciphertext = encryptedDoc["ciphertext"];
  byte iv[N_BLOCK];
  hexStringToByteArray(ivHex, iv, N_BLOCK);

  String decryptedPayload = decrypt(ciphertext, iv);
  DynamicJsonDocument doc(512);
  error = deserializeJson(doc, decryptedPayload);
  if (error) {
    Serial.println("Failed to parse decrypted JSON");
    return;
  }

  Serial.println("Decrypted MQTT message on topic: " + topicStr);

  if (topicStr == topicInitialRequest) {
    sendTemperatureUpdate();
  } else if (topicStr == topicTargetTemp) {
    unsigned long messageTimestamp = doc["timestamp"] | 0;
    if (messageTimestamp < lastTargetUpdateTimestamp) {
      Serial.println("Ignoring outdated target temperature request");
      return;
    }

    float newTarget = doc["value"];
    if (newTarget >= MIN_TEMP && newTarget <= MAX_TEMP) {
      targetTemp = newTarget;
      lastTargetUpdateTimestamp = messageTimestamp;
      saveTargetTemp();

      bool newBubbles = doc["bubbles"] | false;
      if (newBubbles != bubblesEnabled) {
        bubbleScheduled = false;
        if (newBubbles) enableBubbles();
        else disableBubbles();
      }

      DynamicJsonDocument statusDoc(256);
      statusDoc["deviceId"] = deviceId;
      statusDoc["type"] = "target_temp_update";
      statusDoc["status"] = "ok";
      statusDoc["message"] = newTarget;
      mqttPublishEncrypted(topicStatus, statusDoc);
      Serial.println("Target temperature updated to " + String(newTarget) + "°C");
    } else {
      DynamicJsonDocument statusDoc(256);
      statusDoc["deviceId"] = deviceId;
      statusDoc["type"] = "target_temp_update";
      statusDoc["status"] = "error";
      statusDoc["message"] = "Temperature out of valid range (0-50°C)";
      mqttPublishEncrypted(topicStatus, statusDoc);
      Serial.println("Error: Temperature out of valid range (0-50°C)");
    }
  }
}

void readSerialData() {
  while (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    
    if (line.length() == 0) continue;
    
    lastSerialRead = millis();
    
    if (line.startsWith("H:")) {
      // Heater temperature
      currentHeaterTemp = line.substring(2).toFloat();
      heaterSensorError = false;
      heaterErrorType = "";
      
    } else if (line.startsWith("W:")) {
      // Water temperature
      currentWaterTemp = line.substring(2).toFloat();
      waterSensorError = false;
      waterErrorType = "";
      
    } else if (line.startsWith("ERR:H:")) {
      // Heater sensor error
      heaterSensorError = true;
      heaterErrorType = line.substring(6);
      currentHeaterTemp = -127.0;
      sendError("Heater sensor " + heaterErrorType);
      
    } else if (line.startsWith("ERR:W:")) {
      // Water sensor error
      waterSensorError = true;
      waterErrorType = line.substring(6);
      currentWaterTemp = -127.0;
      sendError("Water sensor " + waterErrorType);

    } else if (line == "ACK:ON") {
      heaterEnabled = true;
      lastHeaterSwitch = millis();
      waitingForAck = false;
      heaterRetryCount = 0;
    } else if (line == "ACK:OFF") {
      heaterEnabled = false;
      lastHeaterSwitch = millis();
      waitingForAck = false;
      heaterRetryCount = 0;
    } else if (line == "ACK:BUBBLES_ON") {
      bubblesEnabled = true;
      waitingForBubblesAck = false;
      bubbleRetryCount = 0;
    } else if (line == "ACK:BUBBLES_OFF") {
      bubblesEnabled = false;
      waitingForBubblesAck = false;
      bubbleRetryCount = 0;
    } else if (line == "NACK:ON" || line == "NACK:OFF") {
      if (millis() - mqttNackErrorTime > mqttErrorCooldown) {
        if (line == "NACK:ON") {
          sendError("SAFETY SHUTOFF ENGAGED. HEATER TOO HOT!!!");
        } else {
          sendError("Uno NACK for heater command: " + line);
        }
        mqttNackErrorTime = millis();
      }
      waitingForAck = false;
      heaterRetryCount = 0;
    } else if (line == "NACK:BUBBLES_ON" || line == "NACK:BUBBLES_OFF") {
      if (millis() - mqttNackErrorTime > mqttErrorCooldown) {
        sendError("Uno NACK for bubbles command: " + line);
        mqttNackErrorTime = millis();
      }
      waitingForBubblesAck = false;
    } else if (line == "SAFETY:OFF") {
      if (heaterEnabled) {
        heaterEnabled = false;
      }
      sendError("SAFETY SHUTOFF ENGAGED. HEATER TOO HOT!!!");
    }
  }
}

void controlHeater() {
  unsigned long currentTime = millis();
  
  // Check if enough time has passed since last switch
  if (currentTime - lastHeaterSwitch < heaterMinSwitchTime * 1000) {
    return;
  }
  
  // Don't control heater if there's a sensor error
  if (waterSensorError || heaterSensorError) {
    if (heaterEnabled) {
      disableHeater();
    }
    return;
  }
  
  // Control logic
  if (targetTemp == 0) {
    // Target temp is 0, always turn off heater
    if (heaterEnabled) {
      disableHeater();
    }
  } else if (currentWaterTemp < targetTemp && !heaterEnabled) {
    // Turn on heater
    enableHeater();
  } else if (currentWaterTemp >= targetTemp && heaterEnabled) {
    // Turn off heater
    disableHeater();
  }
}

void enableHeater() {
  // Send command to Arduino Uno to turn on heater
  Serial.println("CMD:ON");
  waitingForAck = true;
  desiredHeaterState = true;
  ackRequestTime = millis();
}

void disableHeater() {
  // Send command to Arduino Uno to turn off heater
  Serial.println("CMD:OFF");
  waitingForAck = true;
  desiredHeaterState = false;
  ackRequestTime = millis();
}

void enableBubbles() {
  Serial.println("CMD:BUBBLES_ON");
  waitingForBubblesAck = true;
  desiredBubblesState = true;
  bubblesAckRequestTime = millis();
}

void disableBubbles() {
  Serial.println("CMD:BUBBLES_OFF");
  waitingForBubblesAck = true;
  desiredBubblesState = false;
  bubblesAckRequestTime = millis();
}

// Add automatic bubble scheduling functions
void scheduleAutomaticBubbles() {
  time_t now = time(nullptr);
  struct tm * timeinfo = localtime(&now);
  int hour = timeinfo->tm_hour;
  int minute = timeinfo->tm_min;

  int times[3] = {2, 10, 18};
  for (int i = 0; i < 3; i++) {
    if (hour == times[i] && minute == 0 && !scheduledBubblesRun[i]) {
      enableBubbles();
      bubbleStartMillis = millis();
      bubbleScheduled = true;
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
  doc["deviceId"] = deviceId;
  doc["value"] = currentWaterTemp;
  doc["heaterValue"] = currentHeaterTemp;
  doc["target"] = targetTemp;
  doc["isHeating"] = heaterEnabled;
  doc["bubbles"] = bubblesEnabled;

  mqttPublishEncrypted(topicTemperature, doc);
  Serial.println("Sent encrypted temperature update");
}

void sendError(String errorMessage) {
  DynamicJsonDocument doc(256);
  doc["deviceId"] = deviceId;
  doc["status"] = "error";
  doc["message"] = errorMessage;
  
  String message;
  serializeJson(doc, message);
  mqttPublishEncrypted(topicStatus, doc);
  Serial.println("Sent error: " + message);
}

void loadTargetTemp() {
  EEPROM.get(TARGET_TEMP_ADDR, targetTemp);
  
  if (isnan(targetTemp) || targetTemp < MIN_TEMP || targetTemp > MAX_TEMP) {
    Serial.println("Invalid target temperature in flash. Setting to 5 degrees");
    targetTemp = 5;
    saveTargetTemp();
  }
}

void saveTargetTemp() {
  EEPROM.put(TARGET_TEMP_ADDR, targetTemp);
  EEPROM.commit();
}
