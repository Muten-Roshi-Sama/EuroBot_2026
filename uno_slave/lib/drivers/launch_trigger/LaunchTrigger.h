// lib/drivers/launch_trigger/LaunchTrigger.h
#pragma once
#include <Arduino.h>

class LaunchTrigger {
    public:
        LaunchTrigger(uint8_t pin = 0xFF, uint8_t stableCounts = 3);
        
        bool begin();              // Initialize; return true if success
        void update();             // Poll sensor
        
        bool isTriggered() const;  // true only ONCE after first rope pull
        void reset();              // Reset for next trigger (optional)
        
    private:
        uint8_t sensorPin;
        uint8_t stableCounts;
        
        bool debouncedState;       // false=idle (HIGH), true=triggered (LOW)
        bool lastRaw;
        uint8_t stableCounter;
        bool eventFell;            // Edge detected: HIGH→LOW
        bool hasTriggered;         // ONE-TIME flag: prevents re-trigger
};