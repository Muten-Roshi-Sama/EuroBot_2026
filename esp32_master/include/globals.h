#pragma once
#include <Arduino.h>

// Global shared objects between modules
// Button driver included from lib path; PlatformIO adds lib/ to include paths.


#include "../lib/task_manager/TaskManager.h"
class TaskManager;
extern TaskManager* taskManager;


// ==============================

struct IMUData { 
    float ax; float ay; float az;   // acceleration/translation
    float roll; float pitch;        // Angles
};

struct SensorsData {
    IMUData imu;
    // add other sensors later, e.g.:
    // float distanceLidar;
    // bool bumperPressed;
};

// Shared instance
extern SensorsData sensorsData;
extern SemaphoreHandle_t sensorsMutex;  // Mutex protecting access


