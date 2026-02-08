#pragma once
#include <Arduino.h>
#include "ADXL345.h"   // Jeff Rowberg's ADXL345 class

class IMU {
public:
    IMU();                 // constructor
    bool begin();          // initialize I2C and sensor
    void readRaw(int16_t &x, int16_t &y, int16_t &z); // raw X/Y/Z
    void readG(float &x, float &y, float &z);        // scaled in g

private:
    ADXL345 accel;
    const float scaleFactor = 0.0039f; // raw -> g for ±2g range
};
