#include "Demarage.h"

static bool ready = false;
static uint8_t contact_pin = 255;

void demarage_init(uint8_t pin) {
    contact_pin = pin;
    ready = false;
    pinMode(contact_pin, INPUT_PULLUP);
}

void demarage_update() {
    if (contact_pin == 255 || ready) return;
    int state = digitalRead(contact_pin);
    // Si le contact magnétique est retiré (passage LOW -> HIGH)
    if (state == LOW) {
        ready = true;
    }
}

bool demarage_is_ready() {
    return ready;
}
