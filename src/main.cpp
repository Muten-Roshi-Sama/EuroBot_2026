
#include <Arduino.h>
#include "FSM.h"
#include "EmergencyStop.h"

FsmContext ctx;

void setup() {
  Serial.begin(115200);
  emergency_stop_init(13);
  fsmInit(ctx);
}

void loop() {
  emergency_stop_update();
  fsmStep(ctx);
}




