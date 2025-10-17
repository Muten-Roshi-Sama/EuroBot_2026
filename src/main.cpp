// v0.0


//  TODO :
//        - Fix 
//        - Add 
//        -
//        - 

// v0.1 wiring FSM minimal usage
#include <Arduino.h>
#include "C:/Users/felix/Documents/Master 1 ECAM/Embbeded_project/Projet_git/EuroBot_2026/lib/fsm/FSM.h"


static FsmContext gFsm; 

void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  Serial.println("Setup.");
  
  fsmInit(gFsm);
}



void loop(){

  fsmStep(gFsm);
}




