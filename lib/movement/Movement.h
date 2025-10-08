#ifndef MOVEMENT_H
#define MOVEMENT_H

#include <Arduino.h>

class Movement {
public:
    Movement();
    void begin();
    void driveForward(int speed);
    void stop();
};

#endif
