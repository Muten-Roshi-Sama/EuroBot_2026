#include "imu.h"
#include <Wire.h>

IMU::IMU() {
    // nothing here; actual initialization in begin()
}

bool IMU::begin() {
    Wire.begin(21, 22);  // ESP32 custom SDA/SCL
    accel.initialize();

    if (!accel.testConnection()) {
        Serial.println("ADXL345 connection failed!");
        return false;
    }
    Serial.println("ADXL345 connected!");
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
