#include "Arduino.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "MadgwickAHRS.h"
#include "RCInput.h"

// RC Input configuration
#define RC_PITCH_PIN 2  // GPIO2 for pitch control
RCInput rcPitch(RC_PITCH_PIN);

// ODrive communication settings
#define ODRIVE_SERIAL Serial1
#define ODRIVE_BAUD_RATE 115200
#define ODRIVE_TX_PIN 0  // GPIO0
#define ODRIVE_RX_PIN 1  // GPIO1

Adafruit_MPU6050 mpu;
Madgwick filter;

// Pin definitions
const int BUZZER_PIN = 8;  // Buzzer connected to GPIO8

// Constants
const float SAMPLE_FREQ = 400.0f; // Hz - Required for fast balance control
const float SAMPLE_TIME = 1000000.0f / SAMPLE_FREQ; // in microseconds
const unsigned long SERIAL_UPDATE_INTERVAL = 100000; // Print every 100ms (10Hz)
const unsigned long MOTOR_ENABLE_DELAY = 5000000; // 5 seconds in microseconds
const unsigned long BUZZER_INTERVAL = 500000; // Beep every 500ms
const unsigned long WARNING_START_TIME = 2000000; // Start warning 2 seconds after boot

// Balance control constants
const float DEG_PER_RAD = 57.2957795131f;  // 180/pi
const float COMPLEMENTARY_FILTER_ALPHA = 0.96f;

// Control parameters - now variable for tuning
float pitchKp = 15.0f;  // Proportional gain
float pitchKi = 1.0f;   // Integral gain
float pitchKd = 0.3f;   // Derivative gain
float rateFF = 0.8f;    // Rate feedforward gain
float pitchOffset = 0.0f; // Gyro mounting offset compensation

// RC control parameters
const float RC_PITCH_SCALE = 0.2f;  // Scale RC input to ±0.2 degree pitch offset

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
bool isControlEnabled = true;  // 制御状態を管理

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
  // Clear any response without blocking
  while (ODRIVE_SERIAL.available()) {
    ODRIVE_SERIAL.read();
  }
  return "OK"; // Non-blocking operation, no response needed
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

// Debug print macros
#ifdef DEBUG_ON
  #define DEBUG_PRINT(...) Serial.print(__VA_ARGS__)
  #define DEBUG_PRINTLN(...) Serial.println(__VA_ARGS__)
#else
  #define DEBUG_PRINT(...)
  #define DEBUG_PRINTLN(...)
#endif

// Function to process serial commands
void processSerialCommand() {
  if (Serial.available() > 0) {
    // シリアル入力があった時点でモータを停止
    isControlEnabled = false;
    setMotorVelocity(0, 0);
    setMotorVelocity(1, 0);
    
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    
    // Parse command and value
    int spaceIndex = cmd.indexOf(' ');
    if (spaceIndex == -1) {
      // Commands without parameters
      if (cmd == "status") {
        Serial.println("Current parameters:");
        Serial.print("Kp:"); Serial.println(pitchKp, 3);
        Serial.print("Ki:"); Serial.println(pitchKi, 3);
        Serial.print("Kd:"); Serial.println(pitchKd, 3);
        Serial.print("FF:"); Serial.println(rateFF, 3);
        Serial.print("Offset:"); Serial.println(pitchOffset, 3);
        Serial.print("Control:"); Serial.println(isControlEnabled ? "Enabled" : "Disabled");
        // パラメータ確認後に制御を再開
        isControlEnabled = true;
      }
      return;
    }
    
    String command = cmd.substring(0, spaceIndex);
    float value = cmd.substring(spaceIndex + 1).toFloat();
    
    // Process commands with parameters
    if (command == "kp") {
      pitchKp = value;
      Serial.print("Set Kp to "); Serial.println(value, 3);
    }
    else if (command == "ki") {
      pitchKi = value;
      Serial.print("Set Ki to "); Serial.println(value, 3);
    }
    else if (command == "kd") {
      pitchKd = value;
      Serial.print("Set Kd to "); Serial.println(value, 3);
    }
    else if (command == "ff") {
      rateFF = value;
      Serial.print("Set FF to "); Serial.println(value, 3);
    }
    else if (command == "start") {
      isControlEnabled = true;
      Serial.println("Control enabled");
    }
    else if (command == "stop") {
      isControlEnabled = false;
      setMotorVelocity(0, 0);
      setMotorVelocity(1, 0);
      Serial.println("Control disabled");
    }
    else if (command == "offset") {
      pitchOffset = value;
      Serial.print("Set pitch offset to "); Serial.println(value, 3);
      // パラメータ設定後に制御を再開
      isControlEnabled = true;
    }
  }
}

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  startTime = micros();  // Record start time for motor enable delay
  
  // Initialize RC input
  rcPitch.begin();
  
  // Initialize buzzer pin and start warning sequence immediately
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, HIGH);
  delay(100);
  digitalWrite(BUZZER_PIN, LOW);
  
  // Initialize ODrive serial communication in background
  ODRIVE_SERIAL.setTX(ODRIVE_TX_PIN);
  ODRIVE_SERIAL.setRX(ODRIVE_RX_PIN);
  ODRIVE_SERIAL.begin(ODRIVE_BAUD_RATE);
  
  // Print available commands
  Serial.println("Available commands:");
  Serial.println("status - Show current parameters");
  Serial.println("kp <value> - Set proportional gain");
  Serial.println("ki <value> - Set integral gain");
  Serial.println("kd <value> - Set derivative gain");
  Serial.println("ff <value> - Set rate feedforward gain");
  Serial.println("offset <value> - Set pitch offset");
  Serial.println("start - Enable balance control");
  Serial.println("stop - Disable balance control");
  
  // Note: ODrive initialization and checks will be done in the loop
  DEBUG_PRINTLN("Starting up... Waiting for ODrive...");

  // Try to initialize MPU6050
  if (!mpu.begin()) {
    DEBUG_PRINTLN("Failed to find MPU6050 chip");
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

  DEBUG_PRINTLN("MPU6050 initialized successfully!");
  DEBUG_PRINTLN("Waiting for IMU calibration...");
  
  // Wait for IMU readings to stabilize
  delay(2000);
  
  DEBUG_PRINTLN("Starting balance control...");
}

void loop() {
  processSerialCommand();
  unsigned long now = micros();
  if (isControlEnabled && now - lastUpdate >= SAMPLE_TIME) {
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
    
    // Get RC pitch input and scale to ±1 degree
    float rcPitchOffset = 0.0f;
    if (rcPitch.hasNewData()) {
      rcPitchOffset = rcPitch.getAngle() * (RC_PITCH_SCALE / 90.0f);  // Scale from ±90 to ±1 degree
      rcPitch.clearNewData();
      
      if ((now - lastSerialUpdate) >= SERIAL_UPDATE_INTERVAL) {
        DEBUG_PRINT("RC:");
        DEBUG_PRINT(rcPitchOffset, 2);
        DEBUG_PRINT("\t");
      }
    }
    
    // Store the filtered pitch plus RC offset and mounting offset for control
    pitchAngle = filteredPitch + rcPitchOffset + pitchOffset;

    // Handle warning buzzer and ODrive initialization
    if (!motorEnableFlag) {
      unsigned long timeElapsed = now - startTime;
      
      // Debug output for timing
      if ((now - lastSerialUpdate) >= SERIAL_UPDATE_INTERVAL) {
        DEBUG_PRINT("Time: ");
        DEBUG_PRINT(timeElapsed / 1000000.0f, 1);
        DEBUG_PRINT("s ");
      }
      
      // Start warning sequence immediately
      if (timeElapsed < MOTOR_ENABLE_DELAY) {
        // Toggle buzzer at specified interval
        if ((now - lastBuzzerToggle) >= BUZZER_INTERVAL) {
          lastBuzzerToggle = now;
          buzzerState = !buzzerState;
          digitalWrite(BUZZER_PIN, buzzerState);
          
          if ((now - lastSerialUpdate) >= SERIAL_UPDATE_INTERVAL) {
            DEBUG_PRINT(buzzerState ? "BEEP " : "STOP ");
          }
        }
      }
      
      // Check if it's time to enable motors and ODrive is ready
      if (timeElapsed >= MOTOR_ENABLE_DELAY) {
        // Check ODrive status
        if (!checkODriveErrors()) {
          // If ODrive is not ready, keep beeping but don't enable motors
          DEBUG_PRINTLN("\nWaiting for ODrive... Check connections and errors.");
          return;
        }
        
        motorEnableFlag = true;
        digitalWrite(BUZZER_PIN, LOW);  // Ensure buzzer is off
        DEBUG_PRINTLN("\nODrive ready - Motors enabled!");
      }
    }

    // Balance control
    if (motorEnableFlag) {
      // Safety check - disable motors if pitch is too large
      if (abs(pitchAngle) > MAX_SAFE_PITCH) {
        motorEnableFlag = false;
        setMotorVelocity(0, 0);
        setMotorVelocity(0, 1);
        DEBUG_PRINTLN("Emergency stop: Pitch angle too large!");
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
        float controlOutput = pitchKp * pitchError +
                            pitchKi * pitchErrorSum +
                            pitchKd * pitchErrorRate +
                            rateFF * pitchRate;
        
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
      DEBUG_PRINT("P:");
      DEBUG_PRINT(pitchAngle, 1);
      DEBUG_PRINT("\tR:");
      DEBUG_PRINT(pitchRate, 1);
      DEBUG_PRINT("\tE:");
      DEBUG_PRINT(pitchError, 1);
      DEBUG_PRINT("\tI:");
      DEBUG_PRINT(pitchErrorSum, 1);
      DEBUG_PRINT("\tM:");
      DEBUG_PRINT(motorEnableFlag ? "ON" : "OFF");
      DEBUG_PRINTLN("");

      
      
    }
  }
}
