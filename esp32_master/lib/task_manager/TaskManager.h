#pragma once
#include <Arduino.h>
#include "../tasks/Task.h"

#define MAX_TASKS 8

// class Movement; // forward

class TaskManager {
public:
  TaskManager();
  void addTask(Task* t);      // Add task pointer (from static pool or heap)
  void tick(void* context);               // Call in main loop frequently
  void updateISR(void* context);          // Called internally every 100 ms by tick()
  void requestISR(uint8_t flags); // ISR-safe: set flags from hardware ISRs
  void doISR(void* context);              // Called from tick() when ISR flags present
  void cancelAll();
  bool isIdle() const;
  Task* currentTask() const;
private:
  // Movement* mv;
  Task* queue[MAX_TASKS];
  uint8_t head, tail, count;
  Task* active;
  unsigned long lastISRupdateMs;
  volatile uint8_t pendingIsrFlags; // set from hardware ISRs via requestISR
  volatile bool isrRequested;
  unsigned long lastTickMs;
  bool obstacleActive = false; // état de l'obstacle

};