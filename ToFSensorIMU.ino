
#include <Wire.h>
#include "SparkFun_LSM6DSV16X.h"
#include "DFRobot_URM09.h"

// Create sensor objects
SparkFun_LSM6DSV16X imu;
DFRobot_URM09 ultrasonic;

// Store accelerometer and gyroscope readings
sfe_lsm_data_t accel;
sfe_lsm_data_t gyro;

// ESP32-S3 I2C communication pins
const int SDA_PIN = 8;
const int SCL_PIN = 9;

// Variables for the 5-sample moving average filter
const int WINDOW_SIZE = 5;
float distanceBuffer[WINDOW_SIZE] = {0};
int bufferIndex = 0;
int sampleCount = 0;

// Variables for estimating relative heading
float heading = 0.0;
float gyroBiasZ = 0.0;

// Track timing for sensor readings and Serial output
unsigned long lastIMUTime = 0;
unsigned long lastDistanceTime = 0;
unsigned long lastPrintTime = 0;

// Store raw and smoothed ultrasonic measurements
// -1 indicates that no valid reading is available
float rawDistance = -1;
float filteredDistance = -1;

// Track whether each sensor initialized successfully
bool imuReady = false;
bool ultrasonicReady = false;

// Check whether a sensor responds at its I2C address
bool detectSensor(byte address) {
  Wire.beginTransmission(address);
  return Wire.endTransmission() == 0;
}

// Apply a moving average filter to the last 5 valid distances
float smoothDistance(float distance) {

  // Store the newest measurement in the buffer
  distanceBuffer[bufferIndex] = distance;

  // Move to the next position, wrapping around when full
  bufferIndex = (bufferIndex + 1) % WINDOW_SIZE;

  // Track how many valid samples have been collected
  if (sampleCount < WINDOW_SIZE) {
    sampleCount++;
  }

  // Calculate the average of the stored measurements
  float sum = 0;
  for (int i = 0; i < sampleCount; i++) {
    sum += distanceBuffer[i];
  }

  return sum / sampleCount;
}

void setup() {

  // Start Serial communication
  Serial.begin(115200);
  delay(1500);

  // Initialize I2C communication at 100 kHz
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);

  Serial.println("=== SWARM BOT SENSOR SYSTEM ===");

  // Check whether both sensors are connected
  Serial.print("IMU: ");
  Serial.println(detectSensor(0x6B) ? "DETECTED" : "MISSING");

  Serial.print("Ultrasonic: ");
  Serial.println(detectSensor(0x11) ? "DETECTED" : "MISSING");

  // Initialize the LSM6DSV16X IMU
  if (detectSensor(0x6B)) {
    imuReady = imu.begin();

    if (imuReady) {

      // Reset the IMU and wait for reset completion
      imu.deviceReset();

      unsigned long start = millis();
      while (!imu.getDeviceReset() &&
             millis() - start < 2000) {
        delay(1);
      }

      // Enable consistent sensor data updates
      imu.enableBlockDataUpdate();

      // Configure accelerometer sampling rate and range
      imu.setAccelDataRate(LSM6DSV16X_ODR_AT_60Hz);
      imu.setAccelFullScale(LSM6DSV16X_16g);

      // Configure gyroscope sampling rate and range
      imu.setGyroDataRate(LSM6DSV16X_ODR_AT_60Hz);
      imu.setGyroFullScale(LSM6DSV16X_2000dps);

      Serial.println("IMU initialized");
    }
  }

  // Initialize the URM09 ultrasonic sensor
  if (detectSensor(0x11)) {
    ultrasonicReady = ultrasonic.begin(0x11);

    if (ultrasonicReady) {

      // Use passive measurement mode with a 500 cm range setting
      ultrasonic.setModeRange(
        MEASURE_MODE_PASSIVE,
        MEASURE_RANG_500
      );

      Serial.println("Ultrasonic initialized");
    }
  }

  // Calibrate the gyroscope while the IMU is stationary
  if (imuReady) {
    Serial.println("Keep IMU still for calibration...");

    float sum = 0;
    int count = 0;

    // Collect up to 100 gyroscope readings
    for (int i = 0; i < 100; i++) {
      if (imu.checkStatus()) {
        imu.getGyro(&gyro);

        // Convert millidegrees/sec to degrees/sec
        sum += gyro.zData / 1000.0;
        count++;
      }

      delay(20);
    }

    // Calculate the average stationary gyroscope offset
    if (count > 0) {
      gyroBiasZ = sum / count;
    }

    Serial.print("Gyro bias: ");
    Serial.print(gyroBiasZ, 4);
    Serial.println(" deg/s");
  }

  // Initialize timing variables
  lastIMUTime = millis();
  lastDistanceTime = millis();
  lastPrintTime = millis();

  Serial.println("=== SYSTEM READY ===");
}

void loop() {

  // Record current time for sensor updates
  unsigned long now = millis();

  // Read acceleration and angular velocity from the IMU
  if (imuReady && imu.checkStatus()) {
    imu.getAccel(&accel);
    imu.getGyro(&gyro);

    // Calculate elapsed time between IMU readings in seconds
    float dt = (now - lastIMUTime) / 1000.0;
    lastIMUTime = now;

    // Convert gyroscope Z reading to degrees/sec
    float gyroZ = gyro.zData / 1000.0;

    // Remove the stationary gyroscope bias
    float correctedRate = gyroZ - gyroBiasZ;

    // Ignore small angular velocities caused by sensor noise
    if (abs(correctedRate) < 0.5) {
      correctedRate = 0;
    }

    // Integrate angular velocity to estimate relative heading
    if (dt > 0 && dt < 0.5) {
      heading += correctedRate * dt;
    }

    // Normalize heading to the range -180 to +180 degrees
    while (heading > 180) heading -= 360;
    while (heading < -180) heading += 360;
  }

  // Take an ultrasonic measurement every 500 milliseconds
  if (ultrasonicReady &&
      now - lastDistanceTime >= 500) {

    lastDistanceTime = now;

    // Trigger a new ultrasonic measurement
    ultrasonic.measurement();
    delay(100);

    // Read the measured distance in centimeters
    int16_t distance = ultrasonic.getDistance();

    // Only process measurements within the valid range
    if (distance > 0 && distance <= 500) {
      rawDistance = distance;

      // Apply the 5-sample moving average filter
      filteredDistance = smoothDistance(distance);
    } else {

      // Mark invalid readings without adding them to the filter
      rawDistance = -1;
    }
  }

  // Display combined sensor readings every 500 milliseconds
  if (millis() - lastPrintTime >= 500) {
    lastPrintTime = millis();

    if (imuReady) {

      // Print X, Y, and Z acceleration in g
      Serial.print("Accel (g): ");
      Serial.print(accel.xData / 1000.0, 3);
      Serial.print(", ");
      Serial.print(accel.yData / 1000.0, 3);
      Serial.print(", ");
      Serial.print(accel.zData / 1000.0, 3);

      // Print angular velocity around the Z-axis
      Serial.print(" | Gyro Z (deg/s): ");
      Serial.print(gyro.zData / 1000.0, 3);

      // Print estimated relative heading
      Serial.print(" | Heading (deg): ");
      Serial.print(heading, 2);

    } else {
      Serial.print("IMU unavailable");
    }

    if (ultrasonicReady) {
      Serial.print(" | Distance: ");

      // Print the most recent raw distance measurement
      if (rawDistance > 0) {
        Serial.print(rawDistance, 0);
        Serial.print(" cm");
      } else {
        Serial.print("invalid");
      }

      Serial.print(" | Smoothed: ");

      // Print the filtered distance measurement
      if (filteredDistance > 0) {
        Serial.print(filteredDistance, 1);
        Serial.print(" cm");
      } else {
        Serial.print("N/A");
      }

    } else {
      Serial.print(" | Ultrasonic unavailable");
    }

    // Start a new line for the next set of readings
    Serial.println();
  }

  // Small delay to avoid unnecessary CPU usage
  delay(10);
}
