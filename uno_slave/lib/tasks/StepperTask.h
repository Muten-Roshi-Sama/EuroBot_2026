#pragma once
#include "Task.h"
#include "../drivers/stepper_controller/StepperController.h"


// ====================================
//         UP TASK
// ====================================
class StepperUpTask : public Task {
public:
    StepperUpTask(StepperController* ctrl, unsigned long timeoutMs = 5000);
    void start(Movement& mov) override;
    void update(Movement& mov) override;
    bool isFinished() const;

private:
    StepperController* controller;
    unsigned long startTime;
    unsigned long timeout;
    bool finished;
};


// ====================================
//         DOWN TASK
// ====================================
class StepperDownTask : public Task {
public:
    StepperDownTask(StepperController* ctrl, unsigned long timeoutMs = 5000);
    void start(Movement& mov) override;
    void update(Movement& mov) override;
    bool isFinished() const;

private:
    StepperController* controller;
    unsigned long startTime;
    unsigned long timeout;
    bool finished;
};





