#pragma once
#include <Arduino.h>

enum class TaskInterruptAction {
  IGNORE,
  PAUSE,
  CANCEL,
  HANDLE
};

class Movement;

class Task {
public:
  // CONSTRUCTOR
  Task(uint8_t speed_ = 0, unsigned long timeoutMs_ = 0)
    : speed(speed_), timeoutMs(timeoutMs_), startMs(0),
      started(false), finished(false), cancelled(false), paused(false) {}

  // Destructor
  virtual ~Task() {}

  // Methods to implement in subclasses
  virtual void start(Movement &mv) = 0;
  virtual void update(Movement &mv) = 0;

  // Default interrupt handler (override when needed)
  virtual TaskInterruptAction handleInterrupt(Movement &mv, uint8_t isrFlags) {
    (void)mv; (void)isrFlags;
    return TaskInterruptAction::PAUSE;
  }

  virtual void cancel(Movement &mv) {
    (void)mv;
    cancelled = true;
    finished = true;
  }

  // Optional: resume a paused task (override if task supports it)
  virtual void resume(Movement &mv) { (void)mv; }

  // Optional small ISR-time update hook
  virtual void updateISR(Movement &mv) { (void)mv; }

  // Getters
  bool isStarted() const { return started; }
  bool isFinished() const { return finished; }
  bool isCancelled() const { return cancelled; }
  bool isPaused() const { return paused; }

  uint8_t getSpeed() const { return speed; }

protected:
  uint8_t speed = 0;
  unsigned long timeoutMs = 0;
  unsigned long startMs = 0;
  bool started = false;
  bool finished = false;
  bool cancelled = false;
  bool paused = false;
};


