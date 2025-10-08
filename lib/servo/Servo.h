#ifndef SERVO_H
#define SERVO_H

#include <Arduino.h>

class Servo {
public:
    Servo();
    void begin();
    void update();
    void grab();
    void release();
};

#endif
