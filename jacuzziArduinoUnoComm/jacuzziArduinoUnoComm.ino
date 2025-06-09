#include <OneWire.h>
#include <DallasTemperature.h>

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

void setup() {
  Serial.begin(115200);
  sensors.begin();
  pinMode(HEATER_CONTROL_PIN, OUTPUT);
  digitalWrite(HEATER_CONTROL_PIN, LOW);
  pinMode(BUBBLES_CONTROL_PIN, OUTPUT);
  digitalWrite(BUBBLES_CONTROL_PIN, LOW);
}

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd == "CMD:ON") {
      if (!heaterEnabled && heaterTemp <= SAFETY_MAX_TEMP) {
        digitalWrite(HEATER_CONTROL_PIN, HIGH);
        heaterEnabled = true;
        Serial.println("ACK:ON");
      } else if (heaterTemp > SAFETY_MAX_TEMP) {
        Serial.println("NACK:ON");
      }
    } else if (cmd == "CMD:OFF") {
      digitalWrite(HEATER_CONTROL_PIN, LOW);
      heaterEnabled = false;
      Serial.println("ACK:OFF");
    } else if (cmd == "CMD:BUBBLES_ON") {
      digitalWrite(BUBBLES_CONTROL_PIN, HIGH);
      bubblesEnabled = true;
      Serial.println("ACK:BUBBLES_ON");
    } else if (cmd == "CMD:BUBBLES_OFF") {
      digitalWrite(BUBBLES_CONTROL_PIN, LOW);
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
    } else if (heaterTemp <= -126 || heaterTemp > 125) {
      Serial.println("ERR:H:INVALID"); // Heater sensor invalid reading
    } else {
      Serial.print("H:");
      Serial.println(heaterTemp);
      // Safety check
      if (heaterTemp > SAFETY_MAX_TEMP && heaterEnabled) {
        digitalWrite(HEATER_CONTROL_PIN, LOW);
        heaterEnabled = false;
        Serial.println("SAFETY:OFF");
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
    }
  }
}
