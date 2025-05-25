#include "Arduino.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "MadgwickAHRS.h"
#include "RCInput.h"

// ODrive communication settings
#define ODRIVE_SERIAL Serial1
#define ODRIVE_BAUD_RATE 115200
#define ODRIVE_TX_PIN 0  // GPIO0
#define ODRIVE_RX_PIN 1  // GPIO1

Adafruit_MPU6050 mpu;
Madgwick filter;
RCInput rcRoll(2);  // Roll control on GPIO2
RCInput rcPitch(3); // Pitch control on GPIO3

// Constants
const float SAMPLE_FREQ = 100.0f; // Hz
const float SAMPLE_TIME = 1000000.0f / SAMPLE_FREQ; // in microseconds
const unsigned long MOTOR_ENABLE_DELAY = 5000000; // 5 seconds in microseconds

// Timing variables
unsigned long lastUpdate = 0;
unsigned long startTime = 0;
bool motorEnableFlag = false;

// Function declarations
String sendODriveCommand(const char* command);
void setMotorVelocity(float velocity, int axis);

// Function to send command to ODrive and read response
String sendODriveCommand(const char* command) {
  ODRIVE_SERIAL.print(command);
  ODRIVE_SERIAL.print("\n");
  String response = ODRIVE_SERIAL.readStringUntil('\n');
  return response;
}

// Function to set motor velocity
void setMotorVelocity(float velocity, int axis) {
  char command[50];
  snprintf(command, sizeof(command), "v %d %.3f", axis, velocity);
  sendODriveCommand(command);
}

// Function to check ODrive errors
bool checkODriveErrors() {
  // Check axis0 errors
  String axis0Errors = sendODriveCommand("r axis0.error");
  String axis0MotorErrors = sendODriveCommand("r axis0.motor.error");
  String axis0EncoderErrors = sendODriveCommand("r axis0.encoder.error");
  
  // Check axis1 errors
  String axis1Errors = sendODriveCommand("r axis1.error");
  String axis1MotorErrors = sendODriveCommand("r axis1.motor.error");
  String axis1EncoderErrors = sendODriveCommand("r axis1.encoder.error");

  // Print all errors
  Serial.println("ODrive Error Status:");
  Serial.println("Axis 0 - Main: " + axis0Errors + 
                " Motor: " + axis0MotorErrors + 
                " Encoder: " + axis0EncoderErrors);
  Serial.println("Axis 1 - Main: " + axis1Errors + 
                " Motor: " + axis1MotorErrors + 
                " Encoder: " + axis1EncoderErrors);

  // Return true if any error is non-zero
  return (axis0Errors.toInt() == 0 && axis0MotorErrors.toInt() == 0 && axis0EncoderErrors.toInt() == 0 &&
          axis1Errors.toInt() == 0 && axis1MotorErrors.toInt() == 0 && axis1EncoderErrors.toInt() == 0);
}

void setup() {
  Serial.begin(115200);
  startTime = micros();  // Record start time for motor enable delay
  
  // Initialize ODrive serial communication
  ODRIVE_SERIAL.setTX(ODRIVE_TX_PIN);
  ODRIVE_SERIAL.setRX(ODRIVE_RX_PIN);
  ODRIVE_SERIAL.begin(ODRIVE_BAUD_RATE);
  delay(100);  // Wait for ODrive to initialize

  Serial.println("Checking ODrive status...");
  if (!checkODriveErrors()) {
    Serial.println("ODrive reported errors. Please check the error codes above.");
  } else {
    Serial.println("ODrive initialized successfully!");
  }

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

    // Check if it's time to enable motors
    if (!motorEnableFlag && (now - startTime) >= MOTOR_ENABLE_DELAY) {
      motorEnableFlag = true;
      Serial.println("Motors enabled!");
    }

    // Convert pitch angle to velocity (-90 to +90 degrees maps to -1 to +1)
    float targetVelocity = 0.0f;
    if (motorEnableFlag) {
      targetVelocity = constrain(rcPitchAngle / 90.0f, -1.0f, 1.0f);
      setMotorVelocity(targetVelocity, 0);  // Set velocity for axis 0
      setMotorVelocity(targetVelocity, 1);  // Set velocity for axis 1
    }

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
    Serial.print(")");
    
    // Print motor status and velocity
    Serial.print("\tMotors:");
    Serial.print(motorEnableFlag ? "ON" : "OFF");
    if (motorEnableFlag) {
      Serial.print("\tVel:");
      Serial.print(targetVelocity, 3);
    }
    Serial.println();
  }
}
