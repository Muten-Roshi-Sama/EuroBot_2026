#include "Switch.h"
#include <Arduino.h>

Switch::Switch(int pin) : pin(pin) {}

void Switch::begin() {
    pinMode(pin, INPUT_PULLUP);
}

bool Switch::isOn() {
    // ON = LOW (circuit fermé vers GND)
    return digitalRead(pin) == LOW;
}
