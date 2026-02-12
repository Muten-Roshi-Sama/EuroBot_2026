#pragma once
#include <Arduino.h>

// Global shared objects between modules
// Button driver included from lib path; PlatformIO adds lib/ to include paths.


#include "../lib/task_manager/TaskManager.h"
class TaskManager;
extern TaskManager* taskManager;


// =======================================================
//                MOTOR PID & Movement
// =======================================================

struct PID {
    float kp, ki, kd;
    float integral, previousError;

    PID(float _kp=0, float _ki=0, float _kd=0) : kp(_kp), ki(_ki), kd(_kd), integral(0), previousError(0) {}
};

const PID DISTANCE_PID_DEFAULT(1.5f, 0.0f, 0.2f); // KP, KI, KD initiaux pour la distance
const PID ANGLE_PID_DEFAULT   (2.0f, 0.0f, 0.3f); // KP, KI, KD initiaux pour l'angle


struct MovementTarget {
    float distanceCm;   // Target distance in cm
    float angleDeg;     // Target rotation in degrees
    bool active;        // Is movement ongoing
};


// =======================================================
//                    SENSORS 
// =======================================================
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


