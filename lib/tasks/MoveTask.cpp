#include "MoveTask.h"
#include "settings.h"
#include "isr_flags.h"

// Helper: absolute long
static inline long labs_long(long v) { return v < 0 ? -v : v; }

void MoveTask::start(Movement &mv) {
  started = true;
  startMs = millis();
  paused = false;

  mv.resetEncoders();

  if (mode == MoveTaskMode::MOVE_DISTANCE) {
    targetTicks = labs_long(mv.cmToTicks(value));
    uint8_t s = getSpeed() ? getSpeed() : mv.defaultSpeed;
    if (value >= 0.0f) mv.forward(s); else mv.backward(s);
  } else { // ROTATE_ANGLE
    targetTicks = labs_long(mv.degreesToTicks(value));
    uint8_t s = getSpeed() ? getSpeed() : mv.defaultSpeed;
    if (value >= 0.0f) mv.rotateRight(s); else mv.rotateLeft(s);
  }
}

void MoveTask::update(Movement &mv) {
  if (cancelled || finished) return;
  if (paused) return;

  long leftTicks, rightTicks;
  noInterrupts();
  leftTicks  = mv.getLeftTicks();
  rightTicks = mv.getRightTicks();
  interrupts();

  if (mode == MoveTaskMode::MOVE_DISTANCE) {
    long maxTicks = max(labs_long(leftTicks), labs_long(rightTicks));
    if (maxTicks >= targetTicks) {
      mv.stop();
      finished = true;
      return;
    }
  } else { // ROTATE_ANGLE
    // Average absolute ticks gives a reasonable estimate of rotation progress
    long avgTicks = (labs_long(leftTicks) + labs_long(rightTicks)) / 2;
    if (avgTicks >= targetTicks) {
      mv.stop();
      finished = true;
      return;
    }
  }

  if (timeoutMs && (millis() - startMs) > timeoutMs) {
    mv.stop();
    cancelled = true;
    finished = true;
    return;
  }
}

TaskInterruptAction MoveTask::handleInterrupt(Movement &mv, uint8_t isrFlags) {
  // Emergency: immediate cancel
  if (isrFlags & ISR_FLAG_EMERGENCY) {
    mv.stop();
    cancelled = true;
    finished = true;
    return TaskInterruptAction::CANCEL;
  }

  // Obstacle: pause and stop motors; FSM/comms can resume later
  if (isrFlags & ISR_FLAG_OBSTACLE) {
    mv.stop();
    paused = true;
    return TaskInterruptAction::PAUSE;
  }

  // Optional: automatic resume on obstacle cleared
  if (isrFlags & ISR_FLAG_OBSTACLE_CLEARED) {
    if (paused) {
      paused = false;
      uint8_t s = getSpeed() ? getSpeed() : mv.defaultSpeed;
      if (mode == MoveTaskMode::MOVE_DISTANCE) {
        if (value >= 0.0f) mv.forward(s); else mv.backward(s);
      } else {
        if (value >= 0.0f) mv.rotateRight(s); else mv.rotateLeft(s);
      }
      return TaskInterruptAction::HANDLE;
    }
  }

  return TaskInterruptAction::IGNORE;
}

void MoveTask::cancel(Movement &mv) {
  cancelled = true;
  mv.stop();
  finished = true;
}

void MoveTask::resume(Movement &mv) {
  if (!paused || cancelled || finished) return;
  paused = false;
  uint8_t s = getSpeed() ? getSpeed() : mv.defaultSpeed;
  if (mode == MoveTaskMode::MOVE_DISTANCE) {
    if (value >= 0.0f) mv.forward(s); else mv.backward(s);
  } else {
    if (value >= 0.0f) mv.rotateRight(s); else mv.rotateLeft(s);
  }
}




