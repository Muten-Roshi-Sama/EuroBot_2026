
// v1.0
#include <Arduino.h>
#include "FSM.h"
#include <Adafruit_MotorShield.h>
#include <Encoder.h>


static FsmContext gFsm; 



void setup() {
  Serial.begin(9600);
  while (!Serial) {}
  Serial.println("Setup.");
  
  fsmInit(gFsm);
    delay(2000);
}



void loop(){

  fsmStep(gFsm);
}
