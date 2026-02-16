/*
 * LEDController - Implémentation
 */

#include "LEDController.h"

LEDController::LEDController() {
}

void LEDController::init_led(uint8_t pin) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
}

void LEDController::led_on(uint8_t pin) {
    digitalWrite(pin, HIGH);
    // Arrêter le clignotement si actif
    stop_blink(pin);
}

void LEDController::led_off(uint8_t pin) {
    digitalWrite(pin, LOW);
    // Arrêter le clignotement si actif
    stop_blink(pin);
}

void LEDController::blink(uint8_t pin, uint32_t interval_ms, uint32_t cycles) {
    // Arrêter le clignotement précédent
    stop_blink(pin);
    
    // Créer un nouvel état de clignotement
    BlinkState state;
    state.pin = pin;
    state.interval_ms = interval_ms;
    state.cycles_remaining = cycles * 2;  // on + off = 1 cycle
    state.last_toggle_ms = millis();
    state.is_on = false;
    
    blinks.push_back(state);
    
    // Démarrer avec LED OFF
    digitalWrite(pin, LOW);
}

void LEDController::stop_blink(uint8_t pin) {
    auto it = blinks.begin();
    while (it != blinks.end()) {
        if (it->pin == pin) {
            it = blinks.erase(it);
        } else {
            ++it;
        }
    }
}

void LEDController::stop_all_blinks() {
    blinks.clear();
}

void LEDController::update() {
    uint32_t now = millis();
    
    for (auto it = blinks.begin(); it != blinks.end(); ) {
        if (now - it->last_toggle_ms >= it->interval_ms) {
            // Toggle
            it->is_on = !it->is_on;
            digitalWrite(it->pin, it->is_on ? HIGH : LOW);
            it->last_toggle_ms = now;
            it->cycles_remaining--;
            
            // Si terminé
            if (it->cycles_remaining == 0) {
                digitalWrite(it->pin, LOW);
                it = blinks.erase(it);
            } else {
                ++it;
            }
        } else {
            ++it;
        }
    }
}

bool LEDController::is_on(uint8_t pin) {
    return digitalRead(pin) == HIGH;
}

BlinkState* LEDController::find_blink(uint8_t pin) {
    for (auto& b : blinks) {
        if (b.pin == pin) return &b;
    }
    return nullptr;
}

void LEDController::remove_blink(uint8_t pin) {
    auto it = blinks.begin();
    while (it != blinks.end()) {
        if (it->pin == pin) {
            it = blinks.erase(it);
        } else {
            ++it;
        }
    }
}
