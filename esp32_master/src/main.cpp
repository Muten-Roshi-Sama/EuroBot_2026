// v2.0


#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <Wire.h>

#include "imu.h"

IMU imu;

// SemaphoreHandle_t i2cMutex;
// QueueHandle_t imuQueue; // Shared queue for IMU data


// void imuTask(void *arg){
//   TickType_t last = xTaskGetTickCount();
//   // AccelData data;
//   while(1){
//     xSemaphoreTake(i2cMutex, portMAX_DELAY);

//     data.ax = 0;
//   }
// }


void i2c_scanner() {
  Serial.println("I2C Scanner");
  byte count = 0;
  for (byte i = 8; i < 120; i++) {
    Wire.beginTransmission(i);
    if (Wire.endTransmission() == 0) {
      Serial.print("Found device at 0x");
      Serial.println(i, HEX);
      count++;
      delay(10);
    }
  }
  if(count == 0) Serial.println("No I2C devices found");
}



// ======================

void setup() {
  Wire.begin(21, 22);
  Serial.begin(115200); delay(2000);

  imu.begin();


  // i2c_scanner();


  // Minimal FreeRTOS task example
  // xTaskCreatePinnedToCore(
  //   [](void *arg){
  //     while(1){
  //       Serial.println("Task alive!");
  //       vTaskDelay(pdMS_TO_TICKS(1000)); // runs every 1s
  //     }
  //   },
  //   "DemoTask",    // Task name
  //   4096,          // Stack size (bytes)
  //   NULL,          // parameters
  //   1,             // priority
  //   NULL,          // handle
  //   1              // core
  // );

}



void loop() {

  // Do nothing
  // vTaskDelay(portMAX_DELAY); // makes tasks sleep without blocking other tasks

  float ax, ay, az;
  imu.readG(ax, ay, az);

  Serial.print("X: "); Serial.print(ax, 3);
  Serial.print("  Y: "); Serial.print(ay, 3);
  Serial.print("  Z: "); Serial.println(az, 3);

  vTaskDelay(pdMS_TO_TICKS(500));


}










// =======================

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
