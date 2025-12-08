#ifndef EMERGENCYBUTTON_H
#define EMERGENCYBUTTON_H

#include <Arduino.h>

class EmergencyButton {
private:
    uint8_t pin;

public:
    EmergencyButton(uint8_t pin);
    void begin();
    bool isPressed();     // retourne true si bouton appuyé
};

#endif
