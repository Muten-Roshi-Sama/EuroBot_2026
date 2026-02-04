#pragma once
#include <Arduino.h>

// Will have derived Tasks (MoveTask, GrabTask,...)

/*
  Each Action (move, grab, detect,...) is implemented in a Task to be non-blocking.
  Must have :

    1. start() : to start the task
    2. update() : part of the task that is run every 100ms
    3. handleInterrupt() : if ISR arises, save important values
    4. cancel() : cancel your task + cleanup needed values

*/

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
  // CONSTRUCTOR
  // Task(uint8_t speed_ = 0, unsigned long timeoutMs_ = 0)
  //   : speed(speed_), timeoutMs(timeoutMs_), startMs(0),
  //     started(false), finished(false), cancelled(false), paused(false) {}

  // Destructor
  virtual ~Task() {}

  // Methods to implement in subclasses
  virtual void start(void* context) = 0;
  virtual void update(void* context) = 0;   // Called frequently from main loop (fast, non-blocking)
  virtual TaskInterruptAction handleInterrupt(void* context, uint8_t isrFlags) {
    return TaskInterruptAction::PAUSE; }   // Default interrupt handler (override when needed)

  // Immediate cancellation/cleanup
  virtual void cancel(void* context) {cancelled = true; finished = true;}

  // Optional: resume a paused task (override if task supports it)
  virtual void resume(void* context) { (void)context; }
  virtual void updateISR(void* context) { (void)context; } // Optional small ISR-time update hook

  // ----- Getters ------
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


