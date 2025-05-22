#ifndef MadgwickAHRS_h
#define MadgwickAHRS_h

#include <Arduino.h>

class Madgwick {
private:
    float beta;              // Algorithm gain
    float q0, q1, q2, q3;   // Quaternion
    float sampleFreq;        // Sample frequency in Hz
    float roll, pitch, yaw;  // Euler angles

    void computeAngles();

public:
    Madgwick();
    void begin(float sampleFrequency) { sampleFreq = sampleFrequency; }
    void update(float gx, float gy, float gz, float ax, float ay, float az);
    
    float getRoll() const { return roll; }
    float getPitch() const { return pitch; }
    float getYaw() const { return yaw; }
    void getQuaternion(float& w, float& x, float& y, float& z) const {
        w = q0;
        x = q1;
        y = q2;
        z = q3;
    }
};

#endif
