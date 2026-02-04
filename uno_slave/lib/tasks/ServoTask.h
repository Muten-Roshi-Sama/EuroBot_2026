#pragma once
#include "Task.h"
#include "../drivers/servo_controller/ServoController.h"

class ServoTask : public Task {
public:
    ServoTask(ServoController* ctrl, uint8_t targetAngle, unsigned long delayMs = 1000);
    void start(Movement& mov) override;
    void update(Movement& mov) override;
    bool isFinished() const;

private:
    ServoController* controller;
    uint8_t angle;
    unsigned long delayMs;
    unsigned long startTime;
    bool finished;
};