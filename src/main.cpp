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

// Pin definitions
const int BUZZER_PIN = 8;  // Buzzer connected to GPIO8

// Constants
const float SAMPLE_FREQ = 400.0f; // Hz - Required for fast balance control
const float SAMPLE_TIME = 1000000.0f / SAMPLE_FREQ; // in microseconds
const unsigned long SERIAL_UPDATE_INTERVAL = 20000; // Print every 20ms (50Hz)
const unsigned long MOTOR_ENABLE_DELAY = 5000000; // 5 seconds in microseconds
const unsigned long BUZZER_INTERVAL = 500000; // Beep every 500ms
const unsigned long WARNING_START_TIME = 2000000; // Start warning 2 seconds after boot

// Balance control constants
const float DEG_PER_RAD = 57.2957795131f;  // 180/pi
const float COMPLEMENTARY_FILTER_ALPHA = 0.96f;

// PID constants
const float PITCH_KP = 0.15f;   // Proportional gain
const float PITCH_KI = 0.05f;   // Integral gain
const float PITCH_KD = 0.02f;   // Derivative gain
const float RATE_FF = 0.01f;    // Rate feedforward gain

// Safety limits
const float MAX_SAFE_PITCH = 45.0f;  // Maximum pitch angle before emergency stop
const float MAX_MOTOR_SPEED = 2.0f;   // Maximum motor speed (-2.0 to +2.0)

// Timing variables
unsigned long lastUpdate = 0;
unsigned long lastSerialUpdate = 0;
unsigned long lastBuzzerToggle = 0;
unsigned long startTime = 0;
bool motorEnableFlag = false;
bool buzzerState = false;

// Balance control variables
float pitchAngle = 0.0f;      // Current pitch angle in degrees
float pitchRate = 0.0f;       // Current pitch rate in degrees/second
float filteredPitch = 0.0f;   // Filtered pitch angle

// PID control variables
float pitchError = 0.0f;      // Current pitch error
float pitchErrorSum = 0.0f;   // Integral of pitch error
float lastPitchError = 0.0f;  // Previous pitch error
float targetPitch = 0.0f;     // Target pitch angle (0 = upright)

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
  snprintf(command, sizeof(command), "v %d %.3f\n", axis, velocity);
  ODRIVE_SERIAL.print(command);
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
  
  // Initialize buzzer pin and start warning sequence immediately
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, HIGH);
  delay(100);
  digitalWrite(BUZZER_PIN, LOW);
  
  // Initialize ODrive serial communication in background
  ODRIVE_SERIAL.setTX(ODRIVE_TX_PIN);
  ODRIVE_SERIAL.setRX(ODRIVE_RX_PIN);
  ODRIVE_SERIAL.begin(ODRIVE_BAUD_RATE);
  
  // Note: ODrive initialization and checks will be done in the loop
  Serial.println("Starting up... Waiting for ODrive...");

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

  Serial.println("MPU6050 initialized successfully!");
  Serial.println("Waiting for IMU calibration...");
  
  // Wait for IMU readings to stabilize
  delay(2000);
  
  Serial.println("Starting balance control...");
}

void loop() {
  unsigned long now = micros();
  if ((now - lastUpdate) >= SAMPLE_TIME) {
    lastUpdate = now;

    // Get new sensor events with the readings
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // Calculate pitch angle from accelerometer
    float accelPitch = atan2f(a.acceleration.x, sqrtf(a.acceleration.y * a.acceleration.y + 
                             a.acceleration.z * a.acceleration.z)) * DEG_PER_RAD;

    // Get pitch rate from gyroscope (positive pitch is forward tilt)
    pitchRate = -g.gyro.y * DEG_PER_RAD;  // Convert to deg/s

    // Complementary filter for pitch angle
    filteredPitch = COMPLEMENTARY_FILTER_ALPHA * (filteredPitch + pitchRate * (SAMPLE_TIME / 1000000.0f)) + 
                   (1.0f - COMPLEMENTARY_FILTER_ALPHA) * accelPitch;
    
    // Store the filtered pitch for control
    pitchAngle = filteredPitch;

    // Handle warning buzzer and ODrive initialization
    if (!motorEnableFlag) {
      unsigned long timeElapsed = now - startTime;
      
      // Debug output for timing
      if ((now - lastSerialUpdate) >= SERIAL_UPDATE_INTERVAL) {
        Serial.print("Time: ");
        Serial.print(timeElapsed / 1000000.0f, 1);
        Serial.print("s ");
      }
      
      // Start warning sequence immediately
      if (timeElapsed < MOTOR_ENABLE_DELAY) {
        // Toggle buzzer at specified interval
        if ((now - lastBuzzerToggle) >= BUZZER_INTERVAL) {
          lastBuzzerToggle = now;
          buzzerState = !buzzerState;
          digitalWrite(BUZZER_PIN, buzzerState);
          
          if ((now - lastSerialUpdate) >= SERIAL_UPDATE_INTERVAL) {
            Serial.print(buzzerState ? "BEEP " : "STOP ");
          }
        }
      }
      
      // Check if it's time to enable motors and ODrive is ready
      if (timeElapsed >= MOTOR_ENABLE_DELAY) {
        // Check ODrive status
        if (!checkODriveErrors()) {
          // If ODrive is not ready, keep beeping but don't enable motors
          Serial.println("\nWaiting for ODrive... Check connections and errors.");
          return;
        }
        
        motorEnableFlag = true;
        digitalWrite(BUZZER_PIN, LOW);  // Ensure buzzer is off
        Serial.println("\nODrive ready - Motors enabled!");
      }
    }

    // Balance control
    if (motorEnableFlag) {
      // Safety check - disable motors if pitch is too large
      if (abs(pitchAngle) > MAX_SAFE_PITCH) {
        motorEnableFlag = false;
        setMotorVelocity(0, 0);
        setMotorVelocity(0, 1);
        Serial.println("Emergency stop: Pitch angle too large!");
      } else {
        // Calculate PID control
        pitchError = targetPitch - pitchAngle;
        
        // Update integral term with anti-windup
        if (abs(pitchError) < 10.0f) {  // Only integrate small errors
          pitchErrorSum += pitchError * (SAMPLE_TIME / 1000000.0f);
          pitchErrorSum = constrain(pitchErrorSum, -10.0f, 10.0f);  // Limit integral windup
        }
        
        // Calculate derivative term
        float pitchErrorRate = (pitchError - lastPitchError) / (SAMPLE_TIME / 1000000.0f);
        lastPitchError = pitchError;
        
        // Combine PID terms and rate feedforward
        float controlOutput = PITCH_KP * pitchError +
                            PITCH_KI * pitchErrorSum +
                            PITCH_KD * pitchErrorRate +
                            RATE_FF * pitchRate;
        
        // Limit motor speed
        controlOutput = constrain(controlOutput, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);
        
        // Apply control to both motors
        setMotorVelocity(controlOutput, 0);
        setMotorVelocity(-controlOutput, 1);  // Invert for axis1
      }
    }

    // Print debug info only every SERIAL_UPDATE_INTERVAL
    if ((now - lastSerialUpdate) >= SERIAL_UPDATE_INTERVAL) {
      lastSerialUpdate = now;
      
      // Print balance control data
      Serial.print("P:");
      Serial.print(pitchAngle, 1);
      Serial.print("\tR:");
      Serial.print(pitchRate, 1);
      Serial.print("\tE:");
      Serial.print(pitchError, 1);
      Serial.print("\tI:");
      Serial.print(pitchErrorSum, 1);
      Serial.print("\tM:");
      Serial.print(motorEnableFlag ? "ON" : "OFF");
      Serial.println();

      
      
    }
  }
}
