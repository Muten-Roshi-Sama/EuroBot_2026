#pragma once
#include <Arduino.h>

// Global shared objects between modules
// Button driver included from lib path; PlatformIO adds lib/ to include paths.


#include "../lib/task_manager/TaskManager.h"
class TaskManager;
extern TaskManager* taskManager;


// ==============================

struct IMUData { 
    float ax, ay, az;   // acceleration/translation
    float roll, pitch;        // Angles
};

struct MPUData {
    float ax, ay, az;   // acceleration/translation
    float gx, gy, gz;   // gyro/rotation
    float roll, pitch, yaw;        // Angles
    // uint32_t timestampUs;
    bool valid;
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
    IMUData imu;  // only accel (unused)
    MPUData mpu;    // full gyro + accel
    USData usFront;
    LidarData lidarFront;
    //
    EncoderData encoderLeft;
    EncoderData encoderRight;
};

// Shared instance
extern SensorsData sensorsData;
extern SemaphoreHandle_t sensorsMutex;  // Mutex protecting access


