#include "EmergencyStop.h"

static uint8_t emergency_pin = 255;
static int lastState = LOW;
static bool triggered = false;

void emergency_stop_init(uint8_t pin) {
  emergency_pin = pin;
  lastState = LOW;
  triggered = false;
  pinMode(emergency_pin, INPUT_PULLUP);
}

void emergency_stop_update() {
  if (emergency_pin == 255 || triggered) return;
  int currentState = digitalRead(emergency_pin);
  // Détection front montant (LOW -> HIGH)
  if (lastState == LOW && currentState == HIGH) {
    triggered = true;
  }
  lastState = currentState;
}

bool emergency_button_pressed() {
  return triggered;
}
