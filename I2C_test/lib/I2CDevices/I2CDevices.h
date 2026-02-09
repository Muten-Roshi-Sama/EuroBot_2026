#pragma once

#include <Arduino.h>
#include <Wire.h>

// PCF8574 I/O expander (8-bit)
class PCF8574 {
public:
    PCF8574() : _addr(0) {}
    bool begin(uint8_t addr);
    // read raw 8-bit port value
    bool read(uint8_t &value);
    // write raw 8-bit port value
    bool write(uint8_t value);
private:
    uint8_t _addr;
};

// ADXL345 accelerometer (basic)
class ADXL345 {
public:
    ADXL345() : _addr(0) {}
    bool begin(uint8_t addr = 0x53);
    bool readAccel(int16_t &ax, int16_t &ay, int16_t &az);
private:
    uint8_t _addr;
    bool writeRegister(uint8_t reg, uint8_t val);
    bool readRegisters(uint8_t reg, uint8_t *buf, size_t len);
};

