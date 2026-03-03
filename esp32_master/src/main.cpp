// v2.0


#include <Arduino.h>
#include "FSM.h"
#include "globals.h"
#include "config.h"
#include "../util/Debug.h"
#include "../fsm_control/FSMFlags.h"
// [WiFi] #include "../wifi_protocol/WiFiProtocol.h"
// [WiFi] #include "../wifi_protocol/LEDController.h"
#include "../bluetooth/bluetooth.h"

#include <Wire.h>
#include <Ultrasonic.h>
#include <VL53L0X.h>
#include "imu.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "esp_chip_info.h"
#include "ioexpander.h"

// Sensors 
IMU imu;
IMUData imuData;
Ultrasonic us(US_TRIG_PIN, US_ECHO_PIN, US_TIMEOUT);
VL53L0X lox;

IOExpander ioExpander(0x20);

SensorsData sensorsData;
SemaphoreHandle_t sensorsMutex;
SemaphoreHandle_t i2cMutex;

IOExpanderData ioExpanderData = {false, false, false};  // Initialize: teamSwitch, launchTrigger, ready
SemaphoreHandle_t ioExpanderMutex;

// FSM 
FsmContext fsmContext;
FSMFlags fsmFlags;  // FSM control flags

// [WiFi] WiFi Protocol & LED Controller
// [WiFi] LEDController ledController;
// [WiFi] WiFiProtocol* wifiProtocol = nullptr;

// Bluetooth Protocol
BluetoothProtocol* bleProtocol = nullptr;
static int fsmSpeed = 50;          // ~20Hz
static int imuSpeed = 10;         // ~100Hz
static int ultrasonicSpeed = 50; // ~20Hz
static int lidarSpeed = 50;     // ~20Hz   (vl53l0x max is 20ms/50Hz)

void fsmTask(void* param) {
    FsmContext* ctx = (FsmContext*) param;
    fsmInitializeSystem(*ctx);

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

void lidarTask(void* param) {
  while(true){
    LidarData local;
    local.distanceCm = 0;
    local.valid = true;
    
    xSemaphoreTake(i2cMutex, portMAX_DELAY);  // block if I2C in use
    uint16_t distanceMM = lox.readRangeContinuousMillimeters();
    bool timeout = lox.timeoutOccurred();
    if (timeout) { Serial.print(" TIMEOUT"); }
    xSemaphoreGive(i2cMutex);

    local.valid = !timeout && distanceMM >= 30 && distanceMM <= 2000;  // VL53L0X specs: 30mm - 2000mm
    local.distanceCm = distanceMM / 10.0;

    xSemaphoreTake(sensorsMutex, portMAX_DELAY);
    sensorsData.lidarFront = local;
    xSemaphoreGive(sensorsMutex);

    vTaskDelay(pdMS_TO_TICKS(lidarSpeed));
  }
}


// ----- Helpers -----
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
void printEsp32Info() {
    Serial.println("=== ESP32 Chip Info ===");
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);

    Serial.printf("Model: ESP32\n");
    Serial.printf("Cores: %d\n", chip_info.cores);
    Serial.printf("Revision: %d\n", chip_info.revision);
    Serial.printf("Features: %s%s%s\n",
        (chip_info.features & CHIP_FEATURE_WIFI_BGN) ? "WiFi " : "",
        (chip_info.features & CHIP_FEATURE_BLE) ? "BLE " : "",
        (chip_info.features & CHIP_FEATURE_BT) ? "BT" : "");

    Serial.printf("Flash: %dMB %s\n",
        spi_flash_get_chip_size() / (1024 * 1024),
        (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    Serial.printf("Free heap: %u bytes\n", ESP.getFreeHeap());
    Serial.printf("CPU frequency: %u MHz\n", ESP.getCpuFreqMHz());
    Serial.printf("SDK version: %s\n", ESP.getSdkVersion());
    Serial.printf("Sketch size: %u bytes\n", ESP.getSketchSize());
    Serial.printf("Free sketch space: %u bytes\n", ESP.getFreeSketchSpace());
    Serial.printf("Chip ID: %06X\n", (uint32_t)ESP.getEfuseMac());
    // Serial.printf("MAC address: %s\n", WiFi.macAddress().c_str());
    // Serial.printf("Flash chip ID: 0x%X\n", spi_flash_get_id());
    // Serial.printf("Flash speed: %u Hz\n", spi_flash_get_speed());
    // Serial.printf("Flash mode: %u\n", spi_flash_get_mode());
}

void lidarinit() {
  if (!lox.init()) { Serial.println("Failed to boot VL53L0X");
  } else {
    Serial.println("VL53L0X booted successfully");
    lox.setTimeout(100);
    lox.setMeasurementTimingBudget(33000);   // HIGH-SPEED mode
    lox.startContinuous(lidarSpeed);         //? Note : Continuous must be > Timing budget !
    // lox.setAddress(LIDAR1_Addr);
  }
  //
}

// ====================== FSM BLE CALLBACKS ======================
/**
 * Handle FSM commands from BluetoothProtocol
 * Commands: FSM_SET_READY, FSM_TRIGGER_LAUNCH, FSM_EMERGENCY_STOP
 */
// [WiFi] void handle_fsm_commands(const char* cmd, const JsonObject& params, DynamicJsonDocument& response) {
void handle_fsm_commands(const char* cmd, const JsonObject& params, JsonDocument& response) {
  if (strcmp(cmd, "FSM_SET_READY") == 0) {
    bool value = params["value"] | false;
    fsmFlags.set_ready(value);
    
    response["status"] = "ok";
    response["message"] = "ready flag set";
    response["data"]["flag"] = "ready";
    response["data"]["value"] = value;
    
    Serial.printf("[FSM_CMD] SET_READY = %d\n", value);
  }
  else if (strcmp(cmd, "FSM_TRIGGER_LAUNCH") == 0) {
    bool value = params["value"] | false;
    fsmFlags.set_trigger_launch(value);
    
    response["status"] = "ok";
    response["message"] = "trigger_launch flag set";
    response["data"]["flag"] = "trigger_launch";
    response["data"]["value"] = value;
    
    Serial.printf("[FSM_CMD] TRIGGER_LAUNCH = %d\n", value);
  }
  else if (strcmp(cmd, "FSM_EMERGENCY_STOP") == 0) {
    bool value = params["value"] | false;
    fsmFlags.set_emergency_stop(value);
    
    response["status"] = "ok";
    response["message"] = "emergency_stop flag set";
    response["data"]["flag"] = "emergency_stop";
    response["data"]["value"] = value;
    
    Serial.printf("[FSM_CMD] EMERGENCY_STOP = %d\n", value);
  }
  else {
    response["status"] = "error";
    response["message"] = "Commande FSM inconnue";
  }
}



// Callback connexion BLE -> met à jour le flag FSM
void handle_ble_connection(bool connected) {
  fsmFlags.set_ble_connected(connected);
  Serial.printf("[BLE_CONN] FSM ble_connected = %d\n", connected);
}

void setup() {
  Wire.begin(22, 23); Wire.setClock(100000);
  debugInit(115200,    // does serial.begin() in this function
    DBG_FSM | 
    DBG_TASKMANAGER     // comment DBG_ to deactivate its related prints
    // DBG_MOVEMENT |
    // DBG_SENSORS |
    // DBG_COMMS |
    // DBG_ENCODER |
    // DBG_LAUNCH_TGR
  );
  delay(200);

  // i2c_scanner();
  // printEsp32Info();

  // Shared resources
  sensorsMutex = xSemaphoreCreateMutex();
  i2cMutex     = xSemaphoreCreateMutex();
  ioExpanderMutex = xSemaphoreCreateMutex();

  // Instanciate Drivers
  lidarinit();
  imu.begin();
  ioExpander.begin(); 
  // US init done in constructor
  
  // [WiFi] LED Controller init
  // [WiFi] ledController.init_led(16);  // Pin 16 for test LED
  
  // FSM setup: associate flags
  fsmContext.flags = &fsmFlags;

  // [WiFi] WiFi Protocol setup
  // [WiFi] Default: EuroBot AP (SSID="EuroBot_AP", Password="12345678")
  // [WiFi] Port 5001 for Master
  // [WiFi] wifiProtocol = new WiFiProtocol("felix123", "ecamwouw", 5000);
  // [WiFi] if (wifiProtocol->setup()) {
  // [WiFi]   Serial.println("[Main] WiFiProtocol initialized successfully");
  // [WiFi]   wifiProtocol->on_command(handle_fsm_commands);
  // [WiFi]   wifiProtocol->set_led_controller(&ledController);
  // [WiFi] } else {
  // [WiFi]   Serial.println("[Main] WARNING: WiFiProtocol setup failed");
  // [WiFi] }

  // Bluetooth Protocol setup
  bleProtocol = new BluetoothProtocol("EuroBot");
  if (bleProtocol->setup()) {
    Serial.println("[Main] BluetoothProtocol initialized successfully");
    // Register FSM command callback
    bleProtocol->on_command(handle_fsm_commands);
    // Register connection callback (updates FSM flag)
    bleProtocol->on_connection(handle_ble_connection);
  } else {
    Serial.println("[Main] WARNING: BluetoothProtocol setup failed");
  }
  
  // Create Tasks
  xTaskCreatePinnedToCore(imuTask, "IMU", 2048, &imu,       2, nullptr, 1);  // Start IMU task (medium priority, core 1)
  xTaskCreatePinnedToCore(fsmTask, "FSM", 4096, &fsmContext, 3, nullptr, 1);  // Start FSM task (high priority, core 1)
  xTaskCreatePinnedToCore(ultrasonicTask, "US", 4096, &us, 2, nullptr, 1);
  xTaskCreatePinnedToCore(lidarTask, "LIDAR", 4096, &lox, 2, nullptr, 1);
  xTaskCreatePinnedToCore(IOExpander::taskEntry, "IOExpander", 2048, &ioExpander, 2, nullptr, 0);  // Core 0 to avoid I2C contention
  
  // Wait for IOExpander to ready (first I2C read complete)
  uint32_t timeout = millis();
  while (!ioExpanderData.ready && (millis() - timeout < 5000)) {
    delay(10);
  }
  if (ioExpanderData.ready) {
    Serial.println("[Setup] IOExpander ready!");
  } else {
    Serial.println("[Setup] WARNING: IOExpander not ready after timeout");
  }
}



void loop() {
  // [WiFi] WiFi Protocol update (non-blocking)
  // [WiFi] if (wifiProtocol) {
  // [WiFi]   wifiProtocol->update();
  // [WiFi] }
  
  // [WiFi] LED Controller update
  // [WiFi] ledController.update();

  // Bluetooth Protocol update (non-blocking)
  if (bleProtocol) {
    bleProtocol->update();
  }
  
  // Small delay to avoid CPU saturation
  delayMicroseconds(100);
}
