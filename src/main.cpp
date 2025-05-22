#include "Arduino.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "MadgwickAHRS.h"

Adafruit_MPU6050 mpu;
Madgwick filter;

// Constants
const float SAMPLE_FREQ = 100.0f; // Hz
const float SAMPLE_TIME = 1000000.0f / SAMPLE_FREQ; // in microseconds

// Timing variables
unsigned long lastUpdate = 0;

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

  // Initialize Madgwick filter
  filter.begin(SAMPLE_FREQ);
}

void loop() {
  unsigned long now = micros();
  if ((now - lastUpdate) >= SAMPLE_TIME) {
    lastUpdate = now;

    // Get new sensor events with the readings
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // Update Madgwick filter
    filter.update(g.gyro.x,
                 g.gyro.y,
                 g.gyro.z,
                 a.acceleration.x,
                 a.acceleration.y,
                 a.acceleration.z);

    // Get Euler angles in degrees
    float roll = filter.getRoll();
    float pitch = filter.getPitch();
    float yaw = filter.getYaw();

    // Print angles
    Serial.print("Roll: ");
    Serial.print(roll);
    Serial.print(", Pitch: ");
    Serial.print(pitch);
    Serial.print(", Yaw: ");
    Serial.println(yaw);
  }
}
