#include "ServoController.h"
#include "../../include/settings.h"

ServoController::ServoController(uint8_t pin_)
    : pin(pin_), currentAngle(90) {}

void ServoController::begin() {
    servo.attach(pin);
    currentAngle = 90;
}

void ServoController::write(uint8_t angle) {
    servo.write(angle);
    currentAngle = angle;
}

uint8_t ServoController::read() const {
    return currentAngle;
}

void ServoController::moveToMin() {
    write(SERVO_MIN_ANGLE);
}

void ServoController::moveToMax() {
    write(SERVO_MAX_ANGLE);
}