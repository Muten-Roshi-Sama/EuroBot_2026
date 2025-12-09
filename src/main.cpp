

//  TODO :
//        - Fix 
//        - Add 
//        -
//        - 

// v0.1 wiring FSM minimal usage



// v1.0
#include <Arduino.h>
#include "FSM.h"
#include "settings.h"
#include "../util/Debug.h"


// #include <Adafruit_MotorShield.h>
// #include <Encoder.h>


FsmContext ctx;

void setup() {
  
  debugInit(9600,    // does serial.begin() in this function
    DBG_FSM | 
    DBG_TASKMANAGER |    // comment DBG_ to deactivate its related prints
    DBG_MOVEMENT |
    DBG_SENSORS |
    DBG_COMMS |
    DBG_ENCODER
  );
  delay(200);  // Serial.begin(115200);

  // Serial.println(" **************** START **************** ");
  debugPrintf(DBG_FSM, "==================== START ===================");
  
  // Init ?
  // init();    //! MAKEs EVERYTHING CRASH

  

  fsmInit(ctx);  //! ALL init functions  ====>  HERE  <=======
}

void loop() {
  fsmStep(ctx);


  // static unsigned long count = 0;
  // Serial.print("HB:");
  // Serial.println(count++);
  // delay(1000);


}



