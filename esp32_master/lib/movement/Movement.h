#ifndef MOVEMENT_H
#define MOVEMENT_H

#include <Arduino.h>

class Movement {
public:
    Movement();
    void begin();
    void forward(int speed);
    void backward(int speed);
    void stop();
    void rotate(float degrees, int speed);
    void moveDistance(float cm, int speed);
};

#endif