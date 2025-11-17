#include "TaskManager.h"
#include "Movement.h"

TaskManager::TaskManager(Movement* mv)
  : mv(mv), head(0), tail(0), count(0), active(nullptr),
    lastISRupdateMs(0), pendingIsrFlags(0), isrRequested(false), lastTickMs(0) {
  for (int i = 0; i < MAX_TASKS; ++i) queue[i] = nullptr;
}

void TaskManager::addTask(Task* t) {
  if (count >= MAX_TASKS) return; // full - caller must handle
  queue[tail] = t;
  tail = (tail + 1) % MAX_TASKS;
  count++;
}

void TaskManager::tick() {
  // fast top-level ISR check
  if (isrRequested) {
    doISR();
  }

  // start next if no active
  if (!active) {
    if (count == 0) return;
    active = queue[head];
    queue[head] = nullptr;
    head = (head + 1) % MAX_TASKS;
    count--;
    if (active) active->start(*mv);
  }

  // run active update (non-blocking)
  if (active && !active->isFinished()) {
    active->update(*mv);
  }

  // cleanup finished
  if (active && active->isFinished()) {
    delete active; // if allocated on heap; otherwise return to pool
    active = nullptr;
  }

  // periodic updateISR every 100ms
  unsigned long now = millis();
  if (now - lastISRupdateMs >= 100) {
    lastISRupdateMs = now;
    updateISR();
  }
}

void TaskManager::updateISR() {
  // called from main loop every 100ms
  if (isrRequested) {
    doISR(); // handle any pending ISR first
    return;
  }
  if (active && !active->isFinished()) {
    active->updateISR(*mv);
  }
}

void TaskManager::requestISR(uint8_t flags) {
  // safe to call from ISR:
  pendingIsrFlags |= flags;
  isrRequested = true;
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