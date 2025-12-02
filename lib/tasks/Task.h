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
  // Ajout de 'paused(false)' dans l'initialisation
  Task(uint8_t speed = 0, unsigned long timeoutMs = 0)
    : speed(speed), timeoutMs(timeoutMs), 
      started(false), finished(false), cancelled(false), paused(false) {} // <--- AJOUT ICI

  virtual ~Task() {}

  virtual void start(Movement &mv) = 0;
  virtual void update(Movement &mv) = 0;

  virtual TaskInterruptAction handleInterrupt(Movement &mv, uint8_t isrFlags) {
    (void)mv; (void)isrFlags;
    return TaskInterruptAction::PAUSE;
  }

  virtual void cancel(Movement &mv) {
    cancelled = true;
    finished = true;
  }

  virtual void updateISR(Movement &mv) { (void)mv; }

  bool isStarted() const { return started; }
  bool isFinished() const { return finished; }
  bool isCancelled() const { return cancelled; }
  bool isPaused() const { return paused; } // Optionnel : getter pratique

  uint8_t getSpeed() const { return speed; }

protected:
  uint8_t speed;
  unsigned long timeoutMs;
  unsigned long startMs = 0;
  bool started;
  bool finished;
  bool cancelled;
  bool paused; // <--- IL MANQUAIT CETTE VARIABLE
};