#include "RCInput.h"
#include "hardware/gpio.h"

// Initialize static members
RCInput* RCInput::instances[2] = {nullptr, nullptr};
uint8_t RCInput::instanceCount = 0;

RCInput::RCInput(uint8_t inputPin) : pin(inputPin), pulseStartTime(0), pulseWidth(0), newData(false) {
    if (instanceCount < 2) {
        instances[instanceCount++] = this;
    }
}

void RCInput::begin() {
    pinMode(pin, INPUT);
    gpio_set_irq_enabled_with_callback(pin, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, 
        reinterpret_cast<gpio_irq_callback_t>(handleInterruptStatic));
}

void RCInput::handleInterruptStatic(uint gpio, uint32_t events) {
    // Find the instance for this GPIO
    for (uint8_t i = 0; i < instanceCount; i++) {
        if (instances[i] && instances[i]->pin == gpio) {
            instances[i]->handleInterrupt(events & GPIO_IRQ_EDGE_RISE);
            break;
        }
    }
}

void RCInput::handleInterrupt(bool rising) {
    uint32_t now = micros();
    if (rising) {
        // Rising edge
        pulseStartTime = now;
        newData = false;
    } else {
        // Falling edge
        if (pulseStartTime > 0 && now > pulseStartTime) {
            uint32_t width = now - pulseStartTime;
            // Strictly validate pulse width between 1000-2000µs
            if (width >= 1000 && width <= 2000) {
                pulseWidth = width;
                newData = true;
            }
        }
    }
}

float RCInput::getAngle() {
    // Ensure we have valid pulse data
    if (pulseWidth == 0) return 0.0f;

    // Convert pulse width to angle
    float angle;
    
    if (pulseWidth <= 1000) {
        angle = -90.0f;  // Minimum pulse -> maximum negative angle
    } else if (pulseWidth >= 2000) {
        angle = 90.0f;   // Maximum pulse -> maximum positive angle
    } else {
        // Normal range: 1000-2000µs maps to -90 to +90 degrees
        // Linear mapping: (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
        angle = (static_cast<float>(pulseWidth) - 1000.0f) * 180.0f / 1000.0f - 90.0f;
    }
    
    return angle;
}
