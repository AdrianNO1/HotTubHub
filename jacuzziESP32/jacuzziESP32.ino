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
DeviceAddress heaterSensor = {0x28, 0x5A, 0x2E, 0x57, 0x00, 0x00, 0x00, 0xFB};
DeviceAddress waterSensor  = {0x28, 0x31, 0x93, 0x56, 0x00, 0x00, 0x00, 0xEC};

// DeviceAddress heaterSensor = {0x28, 0x5A, 0x2E, 0x57, 0x00, 0x00, 0x00, 0xFB};
// DeviceAddress waterSensor  = {0x28, 0x17, 0x9D, 0x57, 0x00, 0x00, 0x00, 0xC5};

// LOW -> ON, HIGH -> OFF
const int HEATER_CONTROL_PIN  = 19;
const int BUBBLES_CONTROL_PIN = 18;
const int LIGHT_CONTROL_PIN_1 = 16;
const int LIGHT_CONTROL_PIN_2 = 17;

const float SAFETY_MAX_TEMP = 55.0;
const float MIN_TEMP        = 0.0;
const float MAX_TEMP        = 50.0;

const int LIGHT_OFFSET_MINUTES = 5;

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

int minutesOfDayLocal() {
  time_t now = time(nullptr);
  struct tm lt;
  localtime_r(&now, &lt);
  return lt.tm_hour * 60 + lt.tm_min; // 0..1439
}

static double deg2rad(double d) { return d * M_PI / 180.0; }
static double rad2deg(double r) { return r * 180.0 / M_PI; }

static double normalize360(double x) {
  x = fmod(x, 360.0);
  if (x < 0) x += 360.0;
  return x;
}

static double normalize24(double x) {
  x = fmod(x, 24.0);
  if (x < 0) x += 24.0;
  return x;
}

// dayOfYear: 1..366
static int dayOfYearFromTm(const struct tm& lt) {
  return lt.tm_yday + 1;
}

// Computes local time (hours 0..24) of sunrise/sunset for given date & position.
// zenithDeg: 90.833 = "official" sunrise/sunset (includes refraction).
// Returns false if sun never rises/sets that day (polar day/night).
static bool calcSunTimeLocalHours(
    int year, int month, int day, int dayOfYear,
    double latitudeDeg, double longitudeDeg,
    bool isSunrise,
    double zenithDeg,
    double &outLocalHours)
{
  (void)year; (void)month; (void)day; // not needed by this variant; kept for clarity

  // Longitude hour value
  double lngHour = longitudeDeg / 15.0;

  // Approx time
  double t = dayOfYear + ((isSunrise ? 6.0 : 18.0) - lngHour) / 24.0;

  // Sun's mean anomaly
  double M = 0.9856 * t - 3.289;

  // Sun's true longitude
  double L = M + 1.916 * sin(deg2rad(M)) + 0.020 * sin(deg2rad(2 * M)) + 282.634;
  L = normalize360(L);

  // Sun's right ascension
  double RA = rad2deg(atan(0.91764 * tan(deg2rad(L))));
  RA = normalize360(RA);

  // Put RA in same quadrant as L
  double Lquadrant  = floor(L / 90.0) * 90.0;
  double RAquadrant = floor(RA / 90.0) * 90.0;
  RA = RA + (Lquadrant - RAquadrant);

  // Convert RA to hours
  RA /= 15.0;

  // Sun's declination
  double sinDec = 0.39782 * sin(deg2rad(L));
  double cosDec = cos(asin(sinDec));

  // Sun local hour angle
  double cosH =
      (cos(deg2rad(zenithDeg)) - (sinDec * sin(deg2rad(latitudeDeg)))) /
      (cosDec * cos(deg2rad(latitudeDeg)));

  if (cosH > 1.0 || cosH < -1.0) {
    return false; // no sunrise or no sunset on this date at this latitude
  }

  double H = isSunrise ? (360.0 - rad2deg(acos(cosH))) : rad2deg(acos(cosH));
  H /= 15.0;

  // Local mean time
  double T = H + RA - 0.06571 * t - 6.622;

  // UTC
  double UT = T - lngHour;
  UT = normalize24(UT);

  // Convert UTC -> local using current TZ/DST rules
  // We need to establish the exact epoch timestamp for the calculated UTC time.
  // UT is hours past UTC midnight.
  // So we calculate timestamp of UTC midnight for that day, add UT offsets,
  // then convert to local time structure.

  struct tm utcMid = {};
  utcMid.tm_year = year - 1900;
  utcMid.tm_mon  = month - 1;
  utcMid.tm_mday = day;
  utcMid.tm_hour = 0;
  utcMid.tm_min  = 0;
  utcMid.tm_sec  = 0;

  // Temporarily switch to UTC to get strict UTC epoch via mktime
  // (mktime interprets struct as local time defined by TZ)
  String oldTz = "";
  if (getenv("TZ")) oldTz = getenv("TZ");
  setenv("TZ", "UTC0", 1);
  tzset();

  time_t utcMidEpoch = mktime(&utcMid);

  // Restore TZ
  if (oldTz.length() > 0) setenv("TZ", oldTz.c_str(), 1);
  else unsetenv("TZ");
  tzset();

  time_t eventEpoch = utcMidEpoch + (time_t)lround(UT * 3600.0);

  struct tm eventLocal;
  localtime_r(&eventEpoch, &eventLocal);

  outLocalHours = eventLocal.tm_hour + eventLocal.tm_min / 60.0 + eventLocal.tm_sec / 3600.0;
  return true;
}

// Convenience: compute minutes since local midnight for today
bool sunriseSunsetMinutesToday(double lat, double lon, int &sunriseMin, int &sunsetMin) {
  time_t now = time(nullptr);
  struct tm lt;
  localtime_r(&now, &lt);

  int year  = lt.tm_year + 1900;
  int month = lt.tm_mon + 1;
  int day   = lt.tm_mday;
  int doy   = dayOfYearFromTm(lt);

  double srH, ssH;
  const double ZENITH = 90.833; // official

  if (!calcSunTimeLocalHours(year, month, day, doy, lat, lon, true,  ZENITH, srH)) return false;
  if (!calcSunTimeLocalHours(year, month, day, doy, lat, lon, false, ZENITH, ssH)) return false;

  sunriseMin = (int)lround(srH * 60.0);
  sunsetMin  = (int)lround(ssH * 60.0);
  return true;
}

bool isDaylightNow() {
  int sr, ss;
  if (!sunriseSunsetMinutesToday(LATITUDE, LONGITUDE, sr, ss)) {
    // polar day/night handling (choose what makes sense for you)
    return false;
  }
  int nowMin = minutesOfDayLocal();
  return (nowMin >= (sr + LIGHT_OFFSET_MINUTES) && nowMin < (ss - LIGHT_OFFSET_MINUTES));
}

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
      if (!bubblesEnabled) {
        enableBubbles();
        bubbleStartMillis = millis();
        bubbleScheduled = true;
      }
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
  pinMode(LIGHT_CONTROL_PIN_1, OUTPUT);
  pinMode(LIGHT_CONTROL_PIN_2, OUTPUT);

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
  setenv("TZ", TIMEZONE_STR, 1);
  tzset();

  mqttClient.setServer(mqttBroker, mqttPort);
  mqttClient.setCallback(mqttCallback);
  mqttClient.setBufferSize(1024);
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
  checkBubbleTimer();
  
  static unsigned long lastTempPub = 0;
  if (millis() - lastTempPub >= 5000UL) {
    scheduleAutomaticBubbles();
    if (isDaylightNow()) {
      digitalWrite(LIGHT_CONTROL_PIN_1, HIGH);
      digitalWrite(LIGHT_CONTROL_PIN_2, LOW);
    } else {
      digitalWrite(LIGHT_CONTROL_PIN_1, LOW);
      digitalWrite(LIGHT_CONTROL_PIN_2, HIGH);
    }
    if (mqttClient.connected()) {
      sendTemperatureUpdate();
    }
    lastTempPub = millis();
  }
}
