// v2.0


#include <Arduino.h>

int c1 = 0;
int c2 = 0;

void task1(void *parameters){
  for(;;){
    Serial.print("Task 1: ");
    Serial.println(c1++);
    vTaskDelay(1000 / portTICK_PERIOD_MS); // delay for 1 second
  } 
}

void task2(void *parameters){
  for(;;){
    Serial.print("Task 2: ");
    Serial.println(c2++);
    vTaskDelay(2000 / portTICK_PERIOD_MS); // delay for 1 second
  } 
}

void setup() {
  Serial.begin(115200); delay(2000);

  xTaskCreate(task1, "Task1", 10000, NULL, 1, NULL);
  xTaskCreate(task2, "Task2", 10000, NULL, 1, NULL);
}

void loop() {

}



// #include <Arduino.h>
// #include "FSM.h"
// #include "settings.h"
// #include "../util/Debug.h"
// #include "globals.h"



// FsmContext context;
// TaskManager* taskManager = nullptr;

// void setup() {
//   debugInit(115200,    // does serial.begin() in this function
//     DBG_FSM | 
//     DBG_TASKMANAGER     // comment DBG_ to deactivate its related prints
//     // DBG_MOVEMENT |
//     // DBG_SENSORS |
//     // DBG_COMMS |
//     // DBG_ENCODER |
//     // DBG_LAUNCH_TGR
//   );
//   delay(2000);
//   debugPrintf(DBG_FSM, "==================== START ===================");

//   fsmInitializeSystem(context);  //! ALL init functions  ====>  HERE  <=======
// }

// void loop() {
//   fsmStep(context);

//   // static unsigned long count = 0;
//   // Serial.print("HB:");
//   // Serial.println(count++);
//   delay(1000);
//   Serial.println("I'm Alive !");
// }
