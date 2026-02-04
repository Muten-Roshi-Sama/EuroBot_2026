#pragma once
#include <AccelStepper.h>

class StepperController {
    public:
        StepperController(
            uint8_t m1_pin1, uint8_t m1_pin2, 
            uint8_t m1_pin3, uint8_t m1_pin4,
            uint8_t m2_pin1, uint8_t m2_pin2, 
            uint8_t m2_pin3, uint8_t m2_pin4
        );
        
        void begin();
        void moveUp();
        void moveDown();
        void stop();
        void run();          // call in loop
        bool isIdle(); // true when both motors reached target

    private:
        AccelStepper moteur1;
        AccelStepper moteur2;
};