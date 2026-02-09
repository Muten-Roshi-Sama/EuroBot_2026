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

struct USData {
    float distanceCm;
    bool valid;
};

struct LidarData {
    float distanceCm;
    bool valid;
};

struct EncoderData {
    int32_t ticks;
    float speed_cms;
    float distance_cm;
};

struct SensorsData {
    IMUData imu;
    USData usFront;
    LidarData lidarFront;
    //
    EncoderData encoderLeft;
    EncoderData encoderRight;
};

// Shared instance
extern SensorsData sensorsData;
extern SemaphoreHandle_t sensorsMutex;  // Mutex protecting access


