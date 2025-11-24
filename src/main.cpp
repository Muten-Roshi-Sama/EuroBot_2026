
#include <Arduino.h>
#include "FSM.h"
#include "EmergencyStop.h"
#include "Demarage.h"

FsmContext ctx;

void setup() {
  Serial.begin(115200);
  fsmInit(ctx);
}

void loop() {
  fsmStep(ctx);
}




