// lib/drivers/launch_trigger/LaunchTrigger.h
#pragma once
#include <Arduino.h>

// Debounced magnetic rope-pull sensor for robot launch
// Stays LOW (triggered) while magnet present; HIGH (idle) otherwise
class LaunchTrigger {
    public:
        LaunchTrigger(uint8_t pin = 0xFF, uint8_t stableCounts = 3);
        
        void begin();
        void update();  // call periodically (e.g., every 50-100ms)
        
        bool isTriggered() const;  // true = rope was pulled (debounced LOW detected)
        void reset();              // clear trigger flag for next run

    private:
        uint8_t sensorPin;
        uint8_t stableCounts;
        
        bool debouncedState;       // false=idle (HIGH), true=triggered (LOW)
        bool lastRaw;
        uint8_t stableCounter;
        bool eventFell;            // edge: HIGH→LOW (rope pulled)
};