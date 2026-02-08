#include "imu.h"
#include <Wire.h>
#include <math.h>

IMU::IMU() : accel(ADXL345()) {} // keep empty

bool IMU::begin() {
    // Wire.begin(22, 23);  // done in main.cpp
    accel.initialize();

    if (!accel.testConnection()) { Serial.println("ADXL345 connection failed!"); return false; }
    
    Serial.println("ADXL345 connected!");
    accel.setRange(ADXL345_RANGE_2G);
    return true;
}

void IMU::readRaw(int16_t &x, int16_t &y, int16_t &z) {
    accel.getAcceleration(&x, &y, &z);
}

void IMU::readG(float &x, float &y, float &z) {
    int16_t rx, ry, rz;
    readRaw(rx, ry, rz);
    x = rx * scaleFactor;
    y = ry * scaleFactor;
    z = rz * scaleFactor;
}

void IMU::readAngles(float &rollDeg, float &pitchDeg) {
    float ax, ay, az;
    readG(ax, ay, az);

    float roll  = atan2(ay, az);
    float pitch = atan2(-ax, sqrt(ay * ay + az * az));

    rollDeg  = roll  * 180.0f / PI;
    pitchDeg = pitch * 180.0f / PI;
}




