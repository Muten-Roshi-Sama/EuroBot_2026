#ifndef EMERGENCY_STOP_H
#define EMERGENCY_STOP_H
#include <Arduino.h>

void emergency_stop_init(uint8_t pin);
void emergency_stop_update();
bool emergency_button_pressed();

#endif
