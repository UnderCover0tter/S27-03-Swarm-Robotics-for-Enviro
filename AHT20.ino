#include <Wire.h>
#include <Adafruit_AHTX0.h>

constexpr int SDA_PIN = 21;
constexpr int SCL_PIN = 22;

Adafruit_AHTX0 aht;
bool sensorReady = false;

void setup() {
  Serial.begin(115200);

  if (!Wire.begin(SDA_PIN, SCL_PIN, 100000)) {
    Serial.println("Could not start I2C.");
    return;
  }

  sensorReady = aht.begin(&Wire);
  if (!sensorReady) {
    Serial.println("Could not initialize AHT20. Check the wiring.");
  }
}

void loop() {
  if (sensorReady) {
    sensors_event_t humidity;
    sensors_event_t temperature;

    if (aht.getEvent(&humidity, &temperature)) {
      Serial.printf("Temperature: %.2f C | Humidity: %.2f %% RH\n",
                    temperature.temperature, humidity.relative_humidity);
    } else {
      Serial.println("AHT20 read failed.");
    }
  }

  delay(2000);
}
