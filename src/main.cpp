// v0.0

#include <Arduino.h>
#include "FSM.h"
#include "Mapping.h"
// #include "Movement.h"

Mapping boardMap;

void setup() {
    Serial.begin(115200);
    Serial.println("Setup.");
    boardMap.begin();
}

void loop() {
    boardMap.update();
}


