#pragma once
#include <Servo.h>

class ServoController {
    public:
        ServoController(uint8_t pin);
        void begin();
        void write(uint8_t angle);
        uint8_t read() const;
        void moveToMin();
        void moveToMax();

    private:
        Servo servo; 
        uint8_t pin;
        uint8_t currentAngle;
};