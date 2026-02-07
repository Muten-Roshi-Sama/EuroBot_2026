// v2.0


#include <Arduino.h>

int c1 = 0;
int c2 = 0;

TaskHandle_t task1_handle = NULL;

void task1(void *parameters){
  for(;;){
    Serial.print("Task 1: ");
    Serial.println(c1++);
    vTaskDelay(1000 / portTICK_PERIOD_MS); // delay for 1 second

    if (c1 > 3) {
      Serial.println("Suspending Task 1"); vTaskSuspend(task1_handle);
    }


  } 
}

void task2(void *parameters){
  for(;;){
    Serial.print("Task 2: ");
    Serial.println(c2++);
    vTaskDelay(1000 / portTICK_PERIOD_MS); // delay for 1 second
  } 
}

void superImportantTask(){
  vTaskSuspendAll(); // suspend scheduler to run critical code without interruption
  // critical code here (e.g., emergency stop)
  xTaskResumeAll(); // resume scheduler after critical code is done
}


void setup() {
  Serial.begin(115200); delay(2000);

  xTaskCreate(task1, "Task1", 10000, NULL, 1, &task1_handle);
  xTaskCreate(task2, "Task2", 10000, NULL, 1, NULL);
}

void loop() {


  if (c2 == 5 && task1_handle != NULL) {
      Serial.println("Resuming Task 1"); vTaskResume(task1_handle);
      // Note : need to add the check if (task1_handle != NULL) in the loop()
    }


  if (c2 == 10 && task1_handle != NULL) {
      Serial.println("Deleting task!"); vTaskDelete(task1_handle);
    }

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
