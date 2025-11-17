#include "MoveTask.h"
// #include "Movement.h"

void MoveTask::start(Movement &mv) {
  started = true;
  startMs = millis();
  mv.resetEncoders();
  targetTicks = mv.cmToTicks(targetCm);
  uint8_t s = getSpeed() ? getSpeed() : DEFAULT_SPEED;
  mv.forward(s);
  startedDrive = true;
}

void MoveTask::update(Movement &mv) {
  if (cancelled || finished) return;
  // atomic read wrapper recommended; using simple call here
  long left = mv.getLeftTicks();
  long right = mv.getRightTicks();
  if (abs(left) >= targetTicks || abs(right) >= targetTicks) {
    mv.stop();
    finished = true;
  } else if (timeoutMs && (millis() - startMs) > timeoutMs) {
    mv.stop();
    cancelled = true;
    finished = true;
  }
}

void MoveTask::updateISR(Movement &mv) {
  // optional: could compute RPM or check battery periodically
}

TaskInterruptAction MoveTask::handleInterrupt(Movement &mv, uint8_t isrFlags) {
  // Example: if emergency flag set, cancel immediately
  if (isrFlags & ISR_FLAG_EMERGENCY) {
    mv.stop();
    cancelled = true;
    finished = true;
    return TaskInterruptAction::CANCEL;
  }
  // for obstacle flag, pause and let FSM decide
  if (isrFlags & ISR_FLAG_OBSTACLE) {
    mv.stop();
    return TaskInterruptAction::PAUSE;
  }
  return TaskInterruptAction::IGNORE;
}