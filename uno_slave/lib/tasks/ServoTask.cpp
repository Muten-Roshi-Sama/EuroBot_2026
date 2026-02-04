#include "ServoTask.h"
#include "../util/Debug.h"

ServoTask::ServoTask(ServoController* ctrl, uint8_t targetAngle, unsigned long delayMs_)
    : controller(ctrl), angle(targetAngle), delayMs(delayMs_), startTime(0), finished(false) {}

void ServoTask::start(Movement& mov) {
    (void)mov;
    startTime = millis();
    finished = false;
    controller->write(angle);
    debugPrintf(DBG_SERVO, "ServoTask: moving to %d degrees", angle);
}

void ServoTask::update(Movement& mov) {
    (void)mov;
    
    if (millis() - startTime >= delayMs) {
        finished = true;
        debugPrintf(DBG_SERVO, "ServoTask done");
    }
}

bool ServoTask::isFinished() const {
    return finished;
}