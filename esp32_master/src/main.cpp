// v2.0


#include <Arduino.h>
#include "FSM.h"
#include "globals.h"
#include "config.h"
#include "imu.h"
#include "../util/Debug.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include <Wire.h>
#include <Ultrasonic.h>


// Sensors 
IMU imu;
IMUData imuData;
Ultrasonic us(US_TRIG_PIN, US_ECHO_PIN, US_TIMEOUT);

SensorsData sensorsData;
SemaphoreHandle_t sensorsMutex;

// FSM 
FsmContext fsmContext;


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


// ====== FreeRTOS Tasks =========
static int fsmSpeed = 1000; // ms delay between FSM steps (adjust as needed)
static int imuSpeed = 500;
static int ultrasonicSpeed = 50; 




void fsmTask(void* param) {
    FsmContext* ctx = (FsmContext*) param;
    fsmInitializeSystem(*ctx);

    // IMUData imuData;

    while(true) {
      // Copy latest sensor data safely
        SensorsData current;
        xSemaphoreTake(sensorsMutex, portMAX_DELAY);
        current = sensorsData;
        xSemaphoreGive(sensorsMutex);

      // FSM step with current sensor data
        fsmStep(*ctx, current); // normal FSM logic
        vTaskDelay(pdMS_TO_TICKS(fsmSpeed)); // FSM loop
    }
}



void imuTask(void* param) {
    IMU* imu = (IMU*) param;

    while(true) {
        IMUData local;

        imu->readG(local.ax, local.ay, local.az);
        imu->readAngles(local.roll, local.pitch);

        // write
        xSemaphoreTake(sensorsMutex, portMAX_DELAY);
        sensorsData.imu = local;
        xSemaphoreGive(sensorsMutex);
        // Serial.print("X: "); Serial.print(ax, 3); Serial.print("  Y: "); Serial.print(ay, 3); Serial.print("  Z: "); Serial.println(az, 3);
        

        vTaskDelay(pdMS_TO_TICKS(imuSpeed)); // 20Hz
    }
}

void ultrasonicTask(void* param) {
    while(true) {

      // ---- Front US ----
      float distanceFront = us.read();
      bool validFront = (distanceFront > 0);
      xSemaphoreTake(sensorsMutex, portMAX_DELAY);
        sensorsData.usFront.distanceCm = distanceFront;
        sensorsData.usFront.valid = validFront;
        xSemaphoreGive(sensorsMutex);

      // ---- Back US ----
      // ....

        vTaskDelay(pdMS_TO_TICKS(ultrasonicSpeed)); // 10Hz
    }
}





// ======================

void setup() {
  Wire.begin(22, 23);
  debugInit(115200,    // does serial.begin() in this function
    DBG_FSM | 
    DBG_TASKMANAGER     // comment DBG_ to deactivate its related prints
    // DBG_MOVEMENT |
    // DBG_SENSORS |
    // DBG_COMMS |
    // DBG_ENCODER |
    // DBG_LAUNCH_TGR
  );


  // Instanciate Drivers
  imu.begin();


  // Shared resources
  sensorsMutex = xSemaphoreCreateMutex();

  // Create Tasks
  xTaskCreatePinnedToCore(imuTask, "IMU", 2048, &imu,       2, nullptr, 1);  // Start IMU task (medium priority, core 1)
  xTaskCreatePinnedToCore(fsmTask, "FSM", 4096, &fsmContext, 3, nullptr, 1);  // Start FSM task (high priority, core 1)
  xTaskCreatePinnedToCore(ultrasonicTask, "ULTRA", 4096, &us, 2, nullptr, 1);


  // i2c_scanner();
}



void loop() {
  // Do nothing
  vTaskDelay(portMAX_DELAY); // makes tasks sleep without blocking other tasks
}
