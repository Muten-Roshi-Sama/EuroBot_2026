#include <Wire.h>
#include "Movement.h"
#include <i2c_addr.h>

// Define Arduino I2C address
#define arduino_addr ARDUINO_ADDR   // from i2c_addr.h

// Example command codes
#define CMD_FORWARD  0x01
#define CMD_BACKWARD 0x02
#define CMD_STOP     0x03
#define CMD_ROTATE   0x04
#define CMD_MOVE_DIST 0x05

Movement::Movement() {}

void Movement::begin() {
    Wire.begin(); // ESP32 as I2C master
}

void Movement::forward(int speed) {
    Wire.beginTransmission(arduino_addr);
    Wire.write(CMD_FORWARD);
    Wire.write(speed);
    Wire.endTransmission();
}

void Movement::backward(int speed) {
    Wire.beginTransmission(arduino_addr);
    Wire.write(CMD_BACKWARD);
    Wire.write(speed);
    Wire.endTransmission();
}

void Movement::stop() {
    Wire.beginTransmission(arduino_addr);
    Wire.write(CMD_STOP);
    Wire.endTransmission();
}

void Movement::rotate(float degrees, int speed) {
    Wire.beginTransmission(arduino_addr);
    Wire.write(CMD_ROTATE);
    Wire.write((int)degrees); // or send as bytes for float
    Wire.write(speed);
    Wire.endTransmission();
}

void Movement::moveDistance(float cm, int speed) {
    Wire.beginTransmission(arduino_addr);
    Wire.write(CMD_MOVE_DIST);
    Wire.write((int)cm); // or send as bytes for float
    Wire.write(speed);
    Wire.endTransmission();
}




