#include "Arduino.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

// Variables for angle calculation
float angleX = 0;
float angleY = 0;
float lastTime = 0;

// Complementary filter coefficient (0 < alpha < 1)
// Higher alpha gives more weight to gyroscope
const float alpha = 0.96;

void setup() {
  Serial.begin(115200);

  // Try to initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  // Set up the accelerometer range
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  // Set up the gyroscope range
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  // Set filter bandwidth
  mpu.setFilterBandwidth(MPU6050_BAND_260_HZ);

  lastTime = micros();
}

void loop() {
  // Get current time
  float currentTime = micros();
  float dt = (currentTime - lastTime) / 1000000.0; // Convert to seconds
  lastTime = currentTime;

  // Get new sensor events with the readings
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Calculate angles from accelerometer
  float accel_angleX = atan2(a.acceleration.y, a.acceleration.z) * 180.0 / PI;
  float accel_angleY = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180.0 / PI;

  // Integrate gyroscope data
  angleX = alpha * (angleX + g.gyro.x * dt * 180.0 / PI) + (1.0 - alpha) * accel_angleX;
  angleY = alpha * (angleY + g.gyro.y * dt * 180.0 / PI) + (1.0 - alpha) * accel_angleY;

  // Print estimated angles
  Serial.print("Angle X: ");
  Serial.print(angleX);
  Serial.print(", Y: ");
  Serial.println(angleY);

  // Wait to achieve approximately 50Hz update rate
  delay(20);
}
