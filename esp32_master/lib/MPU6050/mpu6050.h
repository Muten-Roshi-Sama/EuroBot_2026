#pragma once
#include <Arduino.h>
#include "MPU6050.h"   // Jeff Rowberg's MPU6050 class
#include "globals.h"


struct MPUConfig {
    bool accel;
    bool gyro;
    bool yaw;
    bool roll;  
    bool pitch; 
};


class MPU {
    public:
        // MPU();                 // constructor
        // MPUData mpu_data;
        MPUConfig config;
        bool begin(bool accel, bool gyro, bool yaw, bool roll, bool pitch); // bool dmp);

        void read(MPUData &mpu_data, float dt);

        // void readRaw(float &ax, float &ay, float &az,
        //             float &gx, float &gy, float &gz);

        // void readAngles(float &rollDeg, float &pitchDeg, float &yawDeg); // roll/pitch from accel only (no gyro integration)

    private:
        MPU6050 dev;
        // bool useDMP = true; // set to true if you want to use DMP features (e.g., quaternion, gyro integration)
        
        unsigned long lastUpdateUs = 0;
        float yaw = 0.0f; //degrees
};
