#pragma once
#include <Arduino.h>

// Will have derived Tasks (MoveTask, GrabTask,...)


// Task result for ISR handling decisions
enum class TaskInterruptAction {
  IGNORE,   // ISR irrelevant, continue
  PAUSE,    // pause task (keep state)
  CANCEL,   // cancel task immediately
  HANDLE    // task will actively handle ISR (custom)
};

class Movement; // forward declare

class Task {
public:
  Task(uint8_t speed = 0, unsigned long timeoutMs = 0)
    : speed(speed), timeoutMs(timeoutMs), started(false), finished(false), cancelled(false) {}

  virtual ~Task() {}

  // Called once when the task is started
  virtual void start(Movement &mv) = 0;

  // Called frequently from main loop (fast, non-blocking)
  virtual void update(Movement &mv) = 0;

  // Called every 100 ms by TaskManager::updateISR() for periodic checks
  virtual void updateISR(Movement &mv) { (void)mv; } // optional

  // Called when an ISR event occurs (called by TaskManager::doISR())
  // Return a TaskInterruptAction to instruct manager what to do
  virtual TaskInterruptAction handleInterrupt(Movement &mv, uint8_t isrFlags) {
    (void)mv; (void)isrFlags;
    return TaskInterruptAction::PAUSE;
  }

  // Immediate cancellation/cleanup
  virtual void cancel(Movement &mv) {
    cancelled = true;
    finished = true;
  }

  bool isStarted() const { return started; }
  bool isFinished() const { return finished; }
  bool isCancelled() const { return cancelled; }

  uint8_t getSpeed() const { return speed; }

protected:
  uint8_t speed;
  unsigned long timeoutMs;
  unsigned long startMs = 0;
  bool started;
  bool finished;
  bool cancelled;
};