#include "StartContact.h"
#include <Arduino.h>

StartContact::StartContact(int pin) : pin(pin) {}

void StartContact::begin() {
    // INPUT_PULLUP : HIGH = ouvert, LOW = fermé
    pinMode(pin, INPUT_PULLUP);
}

bool StartContact::isInserted() {
    return digitalRead(pin) == LOW;  // circuit fermé → tirette en place
}

bool StartContact::isRemoved() {
    return digitalRead(pin) == HIGH; // circuit ouvert → tirette retirée
}
