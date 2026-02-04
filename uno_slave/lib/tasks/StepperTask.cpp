#include "StepperTask.h"
#include "../util/Debug.h"



// ====================================
//         UP TASK
// ====================================
StepperUpTask::StepperUpTask(StepperController* ctrl, unsigned long timeoutMs)
    : controller(ctrl), timeout(timeoutMs), finished(false), startTime(0) {}

void StepperUpTask::start(Movement& mov) {
    (void)mov;
    startTime = millis();
    finished = false;
    controller->moveUp();
    debugPrintf(DBG_STEPPER, "StepperUpTask started");
}

void StepperUpTask::update(Movement& mov) {
    (void)mov;
    controller->run();
    
    if (controller->isIdle()) {
        finished = true;
        debugPrintf(DBG_STEPPER, "StepperUpTask done (idle)");
    } else if (millis() - startTime >= timeout) {
        controller->stop();
        finished = true;
        debugPrintf(DBG_STEPPER, "StepperUpTask timeout");
    }
}

bool StepperUpTask::isFinished() const{
    return finished;
}




// ====================================
//         DOWN TASK
// ====================================
StepperDownTask::StepperDownTask(StepperController* ctrl, unsigned long timeoutMs)
    : controller(ctrl), timeout(timeoutMs), finished(false), startTime(0) {}

    void StepperDownTask::start(Movement& mov) {
    (void)mov;
    startTime = millis();
    finished = false;
    controller->moveDown();
    debugPrintf(DBG_STEPPER, "StepperDownTask started");
}

void StepperDownTask::update(Movement& mov) {
  (void)mov;
  controller->run();
  
  if (controller->isIdle()) {
    finished = true;
    debugPrintf(DBG_STEPPER, "StepperDownTask done (idle)");
  } else if (millis() - startTime >= timeout) {
    controller->stop();
    finished = true;
    debugPrintf(DBG_STEPPER, "StepperDownTask timeout");
  }
}

bool StepperDownTask::isFinished() const {
    return finished;
}









