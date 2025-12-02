// v0.0


//  TODO :
//        - Fix 
//        - Add 
//        -
//        - 

// v0.1 wiring FSM minimal usage



#include <Arduino.h>
#include "FSM.h"
#include "settings.h"
#include "../util/Debug.h"


static FsmContext gFsm; 

void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  Serial.println("Setup.");
  
  // Init ?
  init();

  // DEBUG prints
  debugInit(115200,    // comment DBG_ to deactivate its related prints
    DBG_FSM | 
    DBG_TASKMANAGER |
    DBG_MOVEMENT |
    DBG_SENSORS |
    DBG_COMMS |
    DBG_ENCODER
  );

  fsmInit(gFsm);  //! ALL init functions  ====>  HERE  <=======
}



void loop(){

  fsmStep(gFsm);
}

