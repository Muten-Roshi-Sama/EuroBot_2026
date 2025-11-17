// #pragma once
// #include "Task.h"

// class MoveTask : public Task {
// public:
//   MoveTask(long cm, uint8_t speed = 0, unsigned long timeoutMs = 0)
//     : Task(speed, timeoutMs), targetCm(cm), targetTicks(0) {}

//   void start(Movement &mv) override;
//   void update(Movement &mv) override;
//   void updateISR(Movement &mv) override;
//   TaskInterruptAction handleInterrupt(Movement &mv, uint8_t isrFlags) override;
// private:
//   long targetCm;
//   long targetTicks;
//   bool startedDrive = false;
// };