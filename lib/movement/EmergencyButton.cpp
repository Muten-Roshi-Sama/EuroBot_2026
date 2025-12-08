#include "EmergencyButton.h"

EmergencyButton::EmergencyButton(uint8_t pin) {
    this->pin = pin;
}

void EmergencyButton::begin() {
    pinMode(pin, INPUT_PULLUP);  // bouton vers GND = appui → LOW
}

bool EmergencyButton::isPressed() {
    return digitalRead(pin) == HIGH;
}
