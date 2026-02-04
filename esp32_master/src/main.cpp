// v2.0
#include <Arduino.h>
#include "FSM.h"
#include "settings.h"
#include "../util/Debug.h"
#include "globals.h"



FsmContext context;
TaskManager* taskManager = nullptr;

void setup() {
  debugInit(115200,    // does serial.begin() in this function
    DBG_FSM | 
    DBG_TASKMANAGER     // comment DBG_ to deactivate its related prints
    // DBG_MOVEMENT |
    // DBG_SENSORS |
    // DBG_COMMS |
    // DBG_ENCODER |
    // DBG_LAUNCH_TGR
  );
  delay(2000);
  debugPrintf(DBG_FSM, "==================== START ===================");

  fsmInitializeSystem(context);  //! ALL init functions  ====>  HERE  <=======
}

void loop() {
  fsmStep(context);

  // static unsigned long count = 0;
  // Serial.print("HB:");
  // Serial.println(count++);
  delay(1000);
  Serial.println("I'm Alive !");
}
