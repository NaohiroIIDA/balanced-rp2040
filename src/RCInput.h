#ifndef RCINPUT_H
#define RCINPUT_H

#include <Arduino.h>

class RCInput {
private:
    uint8_t pin;
    volatile uint32_t pulseStartTime;
    volatile uint32_t pulseWidth;
    volatile bool newData;

    static void handleInterruptStatic(uint gpio, uint32_t events);
    void handleInterrupt(bool rising);

public:
    RCInput(uint8_t inputPin);
    void begin();
    float getAngle();  // Returns angle from -90 to +90 degrees
    uint32_t getPulseWidth() { return pulseWidth; }  // Returns raw pulse width in microseconds
    bool hasNewData() { return newData; }
    void clearNewData() { newData = false; }

    // Static members for interrupt handling
    static RCInput* instances[2];  // Support for 2 instances (roll and pitch)
    static uint8_t instanceCount;
};

#endif
