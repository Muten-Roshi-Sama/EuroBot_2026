// lib/drivers/launch_trigger/LaunchTrigger.cpp
#include "LaunchTrigger.h"
#include "../util/Debug.h" 

LaunchTrigger::LaunchTrigger(uint8_t pin, uint8_t stableCounts)
    :   sensorPin(pin), stableCounts(stableCounts),
        debouncedState(false), lastRaw(true), stableCounter(0),
        eventFell(false), hasTriggered(false) {}

bool LaunchTrigger::begin() {
    if (sensorPin == 0xFF) return;
    pinMode(sensorPin, INPUT_PULLUP);
    bool raw = digitalRead(sensorPin);
    lastRaw = raw;
    debouncedState = (raw == LOW);
    stableCounter = stableCounts;
    eventFell = false;
    hasTriggered = false;  // Reset one-time flag
    debugPrintf(DBG_LAUNCH_TGR, "LT begin pin=%u raw=%d", sensorPin, raw);
}

void LaunchTrigger::update() {
    if (sensorPin == 0xFF || hasTriggered) return;  // Stop polling after triggered
    
    bool raw = digitalRead(sensorPin);

    if (raw == lastRaw) {
        if (stableCounter < stableCounts) stableCounter++;
    } else {
        lastRaw = raw;
        stableCounter = 1;
    }

    if (stableCounter >= stableCounts) {
        bool logical = (raw == LOW);

        if (debouncedState != logical) {
            if (logical) {
                eventFell = true;
                hasTriggered = true;  // Lock: never trigger again
                debugPrintf(DBG_LAUNCH_TGR, "LT TRIGGERED (ONE-TIME)");
            }
            debouncedState = logical;
        }
        stableCounter = stableCounts;
    }
}

bool LaunchTrigger::isTriggered() const {
    return eventFell;  // Returns true once, then stays true
}

void LaunchTrigger::reset() {
    eventFell = false;
    hasTriggered = false;  // Allow re-trigger after reset
    debugPrintf(DBG_LAUNCH_TGR, "LT reset (can trigger again)");
}