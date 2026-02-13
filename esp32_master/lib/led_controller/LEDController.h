/*
 * LEDController - Contrôle des LEDs avec clignotement non-bloquant
 * Supporte plusieurs LEDs et clignotements simultanés
 */

#ifndef LED_CONTROLLER_H
#define LED_CONTROLLER_H

#include <Arduino.h>
#include <vector>

struct BlinkState {
    uint8_t pin;
    uint32_t interval_ms;
    uint32_t cycles_remaining;
    uint32_t last_toggle_ms;
    bool is_on;
};

class LEDController {
public:
    LEDController();
    
    // Initialiser une LED
    void init_led(uint8_t pin);
    
    // Allumer une LED
    void led_on(uint8_t pin);
    
    // Éteindre une LED
    void led_off(uint8_t pin);
    
    // Clignoter une LED (non-bloquant)
    // interval_ms: intervalle entre chaque toggle
    // cycles: nombre de cycles (on→off compte pour 1 cycle)
    void blink(uint8_t pin, uint32_t interval_ms, uint32_t cycles);
    
    // Arrêter le clignotement d'une LED
    void stop_blink(uint8_t pin);
    
    // Mettre à jour les clignotements (appeler régulièrement dans loop())
    void update();
    
    // Obtenir l'état d'une LED
    bool is_on(uint8_t pin);
    
private:
    std::vector<BlinkState> blinks;
    
    BlinkState* find_blink(uint8_t pin);
    void remove_blink(uint8_t pin);
};

#endif // LED_CONTROLLER_H
