#include "Arduino.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "MadgwickAHRS.h"
#include "RCInput.h"

Adafruit_MPU6050 mpu;
Madgwick filter;
RCInput rcRoll(2);  // Roll control on GPIO2
RCInput rcPitch(3); // Pitch control on GPIO3

// Constants
const float SAMPLE_FREQ = 100.0f; // Hz
const float SAMPLE_TIME = 1000000.0f / SAMPLE_FREQ; // in microseconds

// Timing variables
unsigned long lastUpdate = 0;

void setup() {
  Serial.begin(115200);

  // Initialize RC inputs
  rcRoll.begin();
  rcPitch.begin();

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

    // Get RC input angles
    float rcRollAngle = rcRoll.getAngle();
    float rcPitchAngle = rcPitch.getAngle();

    // Print debug info
    Serial.print("IMU:\t");
    Serial.print(roll);
    Serial.print("\t");
    Serial.print(pitch);
    Serial.print("\t");
    Serial.print(yaw);
    Serial.print("\t RC:\t");
    Serial.print(rcRollAngle);
    Serial.print("(");
    Serial.print(rcRoll.getPulseWidth());
    Serial.print(")\t");
    Serial.print(rcPitchAngle);
    Serial.print("(");
    Serial.print(rcPitch.getPulseWidth());
    Serial.println(")");
  }
}
