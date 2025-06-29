#include <OneWire.h>
#include <DallasTemperature.h>
#include <EEPROM.h>

const int temperatureSensorsPin = 8;
OneWire oneWire(temperatureSensorsPin);
DallasTemperature sensors(&oneWire);

// Hard-coded sensor addresses
DeviceAddress heaterSensor = {0x28, 0x5A, 0x2E, 0x57, 0x00, 0x00, 0x00, 0xFB}; // Sensor 0 (Heater)
DeviceAddress waterSensor  = {0x28, 0x17, 0x9D, 0x57, 0x00, 0x00, 0x00, 0xC5}; // Sensor 1 (Water)

unsigned long previousMillis = 0;
const unsigned long interval = 1000;

const int HEATER_CONTROL_PIN = 7;
const float SAFETY_MAX_TEMP = 55.0;
bool heaterEnabled = false;
const int BUBBLES_CONTROL_PIN = 10;
bool bubblesEnabled = false;

float heaterTemp = 0;
float waterTemp = 0;
bool heaterInvalidTemperature = false;

const int TARGET_TEMP_ADDR = 0;
float targetTemp = 5.0;

// Define temperature bounds and heater switch interval
const float MIN_TEMP = 0.0;
const float MAX_TEMP = 50.0;
const unsigned long heaterMinSwitchTime = 30UL; // seconds
unsigned long lastHeaterSwitch = 0;

void setup() {
  Serial.begin(115200);
  EEPROM.get(TARGET_TEMP_ADDR, targetTemp);
  if (isnan(targetTemp) || targetTemp < MIN_TEMP || targetTemp > MAX_TEMP) {
    targetTemp = 5.0;
  }
  sensors.begin();
  pinMode(HEATER_CONTROL_PIN, OUTPUT);
  digitalWrite(HEATER_CONTROL_PIN, HIGH);
  pinMode(BUBBLES_CONTROL_PIN, OUTPUT);
  digitalWrite(BUBBLES_CONTROL_PIN, HIGH);
  // Send current target temperature to ESP on startup
  Serial.print("T:");
  Serial.println(targetTemp);
}

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.startsWith("T:")) {
      float newTarget = cmd.substring(2).toFloat();
      if (newTarget >= 0.0 && newTarget <= 50.0) {
        targetTemp = newTarget;
        EEPROM.put(TARGET_TEMP_ADDR, targetTemp);
        Serial.print("ACK:T:");
        Serial.println(targetTemp);
      } else {
        Serial.println("ERR:T:OUT_OF_RANGE");
      }
    } else if (cmd == "REQ:T") {
      // ESP is requesting the currently stored target temperature
      Serial.print("T:");
      Serial.println(targetTemp);
    } else if (cmd == "CMD:BUBBLES_ON") {
      digitalWrite(BUBBLES_CONTROL_PIN, LOW);
      bubblesEnabled = true;
      Serial.println("ACK:BUBBLES_ON");
    } else if (cmd == "CMD:BUBBLES_OFF") {
      digitalWrite(BUBBLES_CONTROL_PIN, HIGH);
      bubblesEnabled = false;
      Serial.println("ACK:BUBBLES_OFF");
    }
  }

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    sensors.requestTemperatures();

    heaterTemp = sensors.getTempC(heaterSensor);
    if (heaterTemp == DEVICE_DISCONNECTED_C) {
      Serial.println("ERR:H:DISCONNECTED"); // Heater sensor disconnected
      disableHeater();
      heaterInvalidTemperature = true;
    } else if (heaterTemp <= -126 || heaterTemp > 125) {
      Serial.println("ERR:H:INVALID"); // Heater sensor invalid reading
      disableHeater();
      heaterInvalidTemperature = true;
    } else {
      heaterInvalidTemperature = false;
      Serial.print("H:");
      Serial.println(heaterTemp);
      // Safety check
      if (heaterTemp > SAFETY_MAX_TEMP) {
        disableHeater();
      }
    }

    waterTemp = sensors.getTempC(waterSensor);
    if (waterTemp == DEVICE_DISCONNECTED_C) {
      Serial.println("ERR:W:DISCONNECTED"); // Water sensor disconnected
    } else if (waterTemp <= -126 || waterTemp > 80) {
      Serial.println("ERR:W:INVALID"); // Water sensor invalid reading
    } else {
      Serial.print("W:");
      Serial.println(waterTemp);
      
      if (millis() - lastHeaterSwitch >= heaterMinSwitchTime * 1000) {
        if (targetTemp == 0 && heaterEnabled) {
          disableHeater();
        }
        else if (waterTemp < targetTemp && !heaterEnabled && heaterTemp <= SAFETY_MAX_TEMP && !heaterInvalidTemperature) {
          enableHeater();
        } else if ((waterTemp >= targetTemp || heaterTemp > SAFETY_MAX_TEMP) && heaterEnabled && !heaterInvalidTemperature) {
          disableHeater();
        }
      }
    }
  }
}

void enableHeater() {
  digitalWrite(HEATER_CONTROL_PIN, LOW);
  heaterEnabled = true;
  lastHeaterSwitch = millis();
  Serial.println("HEATER:ON");
}

void disableHeater() {
  digitalWrite(HEATER_CONTROL_PIN, HIGH);
  heaterEnabled = false;
  lastHeaterSwitch = millis();
  Serial.println("HEATER:OFF");
}
