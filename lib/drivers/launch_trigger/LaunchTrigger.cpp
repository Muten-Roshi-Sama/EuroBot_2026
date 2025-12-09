// lib/drivers/launch_trigger/LaunchTrigger.cpp
#include "LaunchTrigger.h"
#include "../util/Debug.h" 

LaunchTrigger::LaunchTrigger(uint8_t pin, uint8_t stableCounts)
    :   sensorPin(pin), stableCounts(stableCounts),
        debouncedState(false), lastRaw(true), stableCounter(0),
        eventFell(false) {}

void LaunchTrigger::begin() {
    if (sensorPin == 0xFF) return;
    pinMode(sensorPin, INPUT_PULLUP);
    bool raw = digitalRead(sensorPin);
    lastRaw = raw;
    debouncedState = (raw == LOW);  // triggered = LOW
    stableCounter = stableCounts;
    eventFell = false;
    debugPrintf(DBG_LAUNCH_TGR, "LT begin pin=%u raw=%d trig=%d", sensorPin, raw, debouncedState);
}

void LaunchTrigger::update() {
    if (sensorPin == 0xFF) return;
    bool raw = digitalRead(sensorPin);

    if (raw == lastRaw) {
        if (stableCounter < stableCounts) stableCounter++;
    } else {
        lastRaw = raw;
        stableCounter = 1;
    }

    if (stableCounter >= stableCounts) {
        bool logical = (raw == LOW);  // triggered = LOW

        if (debouncedState != logical) {
            if (logical) {
                eventFell = true;  // HIGH→LOW: rope pulled!
                debugPrintf(DBG_LAUNCH_TGR, "LT edge HIGH->LOW (TRIGGERED)");
            }
            else {
                debugPrintf(DBG_LAUNCH_TGR, "LT edge LOW->HIGH (released)"); // rising edge (LOW→HIGH): rope released
            }
            debouncedState = logical;
        }

        stableCounter = stableCounts;
    }
}

bool LaunchTrigger::isTriggered() const {
    if (eventFell) debugPrintf(DBG_LAUNCH_TGR, "LT isTriggered=1");
    return eventFell;  // true only after first debounced LOW detection
}

void LaunchTrigger::reset() {
    eventFell = false;  // clear for next run
    debugPrintf(DBG_LAUNCH_TGR, "LT reset");
}