#include "StepperController.h"

StepperController::StepperController(
    uint8_t m1_pin1, uint8_t m1_pin2, 
    uint8_t m1_pin3, uint8_t m1_pin4,
    uint8_t m2_pin1, uint8_t m2_pin2, 
    uint8_t m2_pin3, uint8_t m2_pin4
    )
    : 
    moteur1(AccelStepper::FULL4WIRE, 
        m1_pin1, m1_pin2, m1_pin3, m1_pin4
    ),
    moteur2(AccelStepper::FULL4WIRE, 
        m2_pin1, m2_pin2, m2_pin3, m2_pin4
    )
{}

void StepperController::begin() {
    moteur1.setMaxSpeed(400);
    moteur1.setAcceleration(150);
    moteur2.setMaxSpeed(400);
    moteur2.setAcceleration(150);
}

void StepperController::moveUp() {
    moteur1.moveTo(-750);
    moteur2.moveTo(750);
}

void StepperController::moveDown() {
    moteur1.moveTo(750);
    moteur2.moveTo(-750);
}

void StepperController::stop() {
    moteur1.stop();
    moteur2.stop();
}

void StepperController::run() {
    moteur1.run();
    moteur2.run();
}

bool StepperController::isIdle() {
    return (moteur1.distanceToGo() == 0 && moteur2.distanceToGo() == 0);
}