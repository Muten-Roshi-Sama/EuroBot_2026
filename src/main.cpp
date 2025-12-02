

// static FsmContext gFsm; 



// void setup() {
//   Serial.begin(9600);
//   while (!Serial) {}
//   Serial.println("Setup.");
  
//   fsmInit(gFsm);
//     delay(2000);
// }



// void loop(){

//   fsmStep(gFsm);
// }


// v0.1 wiring FSM minimal usage



// v1.0
#include <Arduino.h>
#include "FSM.h"
#include "settings.h"
#include "../util/Debug.h"

FsmContext ctx;
// #include <Adafruit_MotorShield.h>
// #include <Encoder.h>

//   if (!initLidar()) {
//     Serial.println("ERREUR: Capteur non detecte!");
//     Serial.println("Verifiez les connexions I2C");
//     while (1) {
//       delay(1000);
//     }
//   }

static FsmContext gFsm; 

// void loop() {
//   int distance = lireDistance();
//   afficherEtatObstacle(distance);

//   delay(500);
// }


void setup() {
  Serial.begin(9600);
  while (!Serial) {}
  Serial.println("Setup.");
  
  // Init ?
  init();

  // DEBUG prints
  debugInit(9600,    // comment DBG_ to deactivate its related prints
    DBG_FSM | 
    DBG_TASKMANAGER |
    DBG_MOVEMENT |
    DBG_SENSORS |
    DBG_COMMS |
    DBG_ENCODER
  );

  fsmInit(gFsm);  //! ALL init functions  ====>  HERE  <=======
}

void loop() {
  fsmStep(ctx);
}
