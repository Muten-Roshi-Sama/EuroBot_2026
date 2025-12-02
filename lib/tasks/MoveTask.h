#pragma once
#include "Task.h"
#include "Movement.h" // Important pour connaître la classe Movement

class MoveTask : public Task {
public:
    enum class MoveTaskMode { MOVE_DISTANCE, ROTATE_ANGLE };

    MoveTask(MoveTaskMode mode, float value, uint8_t speed = 0, unsigned long timeoutMs = 0)
        : Task(speed, timeoutMs), mode(mode), value(value) {}

    void start(Movement &mv) override;
    void update(Movement &mv) override;
    TaskInterruptAction handleInterrupt(Movement &mv, uint8_t isrFlags) override;
    
    void resume(Movement &mv);
    void cancel(Movement &mv) override;

private:
  MoveTaskMode mode;
  float value;       // cm when MOVE_DISTANCE, degrees when ROTATE_ANGLE
  long targetTicks;  // absolute target ticks
  bool paused;
  unsigned long lastPidLoopMs;

  // Runtime control state (non-blocking controller)
  static int baseSpeed;           // nominal PWM setpoint
  static int loopCounter;         // number of update() iterations since start
  static int warmupIterations;    // number of iterations for warm-up / ramp
  static float Kp;                // proportional gain for differential correction
  static float Ki;                // optional integral gain (small)
  static float integralError;     // integral accumulator for correction
  static int minSpeed;            // minimum motor PWM during motion

};