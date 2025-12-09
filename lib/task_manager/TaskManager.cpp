#include "TaskManager.h"

// Includes
#include "globals.h"
#include "isr_flags.h"

// Libs
#include "../movement/Movement.h"
#include "../util/Debug.h"


// ==========================================

// Diagnostic helpers (place near top of TaskManager.cpp)
extern "C" char* __brkval;          // avr-libc bookkeeping
extern "C" char __heap_start;
static int freeRam() {
  char stack_dummy = 0;
  char* heap_end = __brkval ? __brkval : &__heap_start;
  return (int)&stack_dummy - (int)heap_end;
}



// TaskManager* TaskManager::instance = nullptr;


TaskManager::TaskManager(Movement* mv)
  : mv(mv), head(0), tail(0), count(0), active(nullptr),
    lastISRupdateMs(0), pendingIsrFlags(0), isrRequested(false), lastTickMs(0) {
  for (int i = 0; i < MAX_TASKS; ++i) queue[i] = nullptr;
}

void TaskManager::addTask(Task* t) {
  debugPrintf(DBG_TASKMANAGER, "addTask called ptr=%p count=%d head=%d tail=%d free=%d",
              (void*)t, count, head, tail, freeRam());
  if (count >= MAX_TASKS) {
    Serial.println("Task queue full");
    return;
  }
  queue[tail] = t;
  tail = (tail + 1) % MAX_TASKS;
  count++;
  debugPrintf(DBG_TASKMANAGER, "AddTask ptr=%p -> queued (count=%d head=%d tail=%d) free=%d",
              (void*)t, count, head, tail, freeRam());}

void TaskManager::tick() {
  /*
    Loop logic :
      1. check ISR -> check if ISR flag is up and manage it.
      2. If no tasks active, start next task.
      3. if a task is active (unfinished) task -> continue (update) this task for 100ms.
      4. clean-up finished tasks.
      5. update ISR timer to run every 100ms.

    Notes :
      - response latency = time to next tick() + time to execute doISR().

    TODO :
      - make 100ms into a variable at taskManager instantiation.
  */

  // 1. fast top-level ISR check
  if (isrRequested) {
    doISR();
  }

  // 2. start next if no active
  if (!active) {
    if (count == 0) return;
    active = queue[head];      // fetch the next task from queue
    queue[head] = nullptr;
    head = (head + 1) % MAX_TASKS;    // ptr management
    count--;
    if (active) active->start(*mv);   // call next task
  }

  // 3. run active task update (non-blocking)
  if (active && !active->isFinished()) {
    active->update(*mv);
  }

  // cleanup finished
  if (active && active->isFinished()) {
    debugPrintf(DBG_TASKMANAGER, "tick: active finished -> deleting task ptr=%p", (void*)active);
    delete active; // if allocated on heap; otherwise return to pool
    active = nullptr;
  }

  // periodic updateISR every 100ms
  unsigned long now = millis();
  if (now - lastISRupdateMs >= 100) {
    lastISRupdateMs = now;
    updateISR();
    // debugPrintf(DBG_TASKMANAGER, "ISR_Update");
  }
}

void TaskManager::updateISR() {
  /* Emergency stop checking HERE
    - called from tick() loop every 100ms
  */

  // BUTTON
  {
    uint8_t btnFlags = emergencyBtn.pollFlags();
    if (btnFlags) {
      requestISR(btnFlags);
      debugPrintf(DBG_TASKMANAGER, "Emergency Button Pressed !");
    }
  }

  // Sensors


  // ==================================
  if (isrRequested) {
    doISR(); // handle any pending ISR first
    return;
  }
  if (active && !active->isFinished()) {
    active->updateISR(*mv);
  }
}



void TaskManager::doISR() {
  // copy and clear flags atomically
  noInterrupts();
  uint8_t flags = pendingIsrFlags;
  pendingIsrFlags = 0;
  isrRequested = false;
  interrupts();

  // If there is an active task, let it handle first
  if (active && !active->isFinished()) {
    TaskInterruptAction act = active->handleInterrupt(*mv, flags);
    if (act == TaskInterruptAction::CANCEL) {
      active->cancel(*mv);
      delete active; active = nullptr;
      return;
    }
    if (act == TaskInterruptAction::PAUSE) {
      // keep active in paused state (we use cancel for simplicity)
      // move it to head of queue or leave as paused (implementation choice)
      mv->stop();
      return;
    }
    if (act == TaskInterruptAction::HANDLE) {
      // active handled it fully, nothing more to do
      return;
    }
  }

  // otherwise, do a global ISR handling policy (e.g., emergency)
  if (flags & ISR_FLAG_EMERGENCY) {
    // immediate emergency handling: stop all tasks
    if (active) active->cancel(*mv);
    cancelAll();
    mv->stop();
    // Optionally set global FSM emergency flag here
  }
  // handle other flags (obstacle etc.) as needed
}



void TaskManager::requestISR(uint8_t flags) {
  // safe to call from ISR:
  pendingIsrFlags |= flags;
  isrRequested = true;
}


void TaskManager::cancelAll() {
  // cancel active
  if (active) {
    active->cancel(*mv);
    delete active;
    active = nullptr;
  }
  // clear queue
  while (count > 0) {
    Task* t = queue[head];
    if (t) delete t;
    queue[head] = nullptr;
    head = (head + 1) % MAX_TASKS;
    count--;
  }
  head = tail = 0;
}

bool TaskManager::isIdle() const {
  return (active == nullptr) && (count == 0);
}



Task* TaskManager::currentTask() const { return active; }