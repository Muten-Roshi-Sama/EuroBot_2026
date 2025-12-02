#pragma once
#include "Task.h"
#include "../movement/Movement.h"

enum class MoveTaskMode {
  MOVE_DISTANCE,
  ROTATE_ANGLE
};

class MoveTask : public Task {
public:
  // Move distance in cm
  MoveTask(float valueCm, uint8_t speed = 0, unsigned long timeoutMs = 0)
    : Task(speed, timeoutMs), mode(MoveTaskMode::MOVE_DISTANCE),
      value(valueCm), targetTicks(0), paused(false) {}

  // Rotate degrees (positive = clockwise/right)
  MoveTask(float degrees, uint8_t speed, unsigned long timeoutMs, bool rotateTag)
    : Task(speed, timeoutMs), mode(MoveTaskMode::ROTATE_ANGLE),
      value(degrees), targetTicks(0), paused(false) {}

  // Task interface
  void start(Movement &mv) override;
  void update(Movement &mv) override;
  void updateISR(Movement &mv) override {}
  TaskInterruptAction handleInterrupt(Movement &mv, uint8_t isrFlags) override;
  void cancel(Movement &mv) override;

  // Optional helper to resume after pause
  void resume(Movement &mv);

private:
  MoveTaskMode mode;
  float value;       // cm when MOVE_DISTANCE, degrees when ROTATE_ANGLE
  long targetTicks;  // absolute target ticks
  bool paused;

  // Runtime control state (non-blocking controller)
  static int baseSpeed;           // nominal PWM setpoint
  static int loopCounter;         // number of update() iterations since start
  static int warmupIterations;    // number of iterations for warm-up / ramp
  static float Kp;                // proportional gain for differential correction
  static float Ki;                // optional integral gain (small)
  static float integralError;     // integral accumulator for correction
  static int minSpeed;            // minimum motor PWM during motion

};