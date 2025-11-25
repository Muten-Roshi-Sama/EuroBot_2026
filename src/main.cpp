

//  TODO :
//        - Fix 
//        - Add 
//        -
//        - 

// v0.1 wiring FSM minimal usage



#include <Arduino.h>
#include "FSM.h"
#include "settings.h"


FsmContext ctx;

void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  Serial.println("Setup.");
  
  // Init ?
  init();

  

  fsmInit(gFsm);  // ! ALL init functions  ====>  HERE  <=======
}

void loop() {
  fsmStep(ctx);
}

