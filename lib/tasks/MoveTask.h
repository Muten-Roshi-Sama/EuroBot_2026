#pragma once
#include "Task.h"
#include "../movement/Movement.h"

enum class MoveTaskMode {
  MOVE_DISTANCE,
  ROTATE_ANGLE
};

/*
    Task : must implement at least (start, update, handleInterrupt, resume and cancel).

    1. MoveTask :
        - distance (+/-)
        - speed    (1-255)

    2. RotateTask :
        - degrees  (90°, -140°, ...)
        - speed 

*/



class MoveTask : public Task {
<<<<<<< Updated upstream
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
};
=======
    public:
        
        MoveTask(float distanceCm, uint8_t speed = 0, unsigned long timeoutMs = 0)
            : Task(speed, timeoutMs), distanceCm(distanceCm) {}

        void start(Movement &mv) override;
        void update(Movement &mv) override;
        TaskInterruptAction handleInterrupt(Movement &mv, uint8_t isrFlags) override;
        
        void resume(Movement &mv);
        void cancel(Movement &mv) override;

    private:
        float distanceCm;     
        long targetTicks; 

        // PID/internal state
        float integralError = 0.0f;
        int loopCounter = 0;
        unsigned long lastPidLoopMs = 0;
        
        // variables
        int baseSpeed = 0;
        int minSpeed = 0;
        int maxSpeed = 255;
        int warmupIterations;
        float Kp = 0.0f;
        float Ki = 0.0f;
        float deadZone = 0;
};
















>>>>>>> Stashed changes
