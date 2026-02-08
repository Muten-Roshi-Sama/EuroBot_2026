#include "TaskManager.h"

// Includes
#include "globals.h"
#include "isr_flags.h"

// Libs
#include "../movement/Movement.h"
#include "../util/Debug.h"

// ==========================================

// Correction pour ESP32 : On utilise l'API native pour la mémoire
static int freeRam() {
  return (int)ESP.getFreeHeap();
}

// ==========================================

TaskManager::TaskManager(Movement* mv)
  : mv(mv), head(0), tail(0), count(0), active(nullptr),
    lastISRupdateMs(0), pendingIsrFlags(0), isrRequested(false), lastTickMs(0) {
  // Initialisation du tableau à null
  for (int i = 0; i < MAX_TASKS; ++i) queue[i] = nullptr;
}

void TaskManager::addTask(Task* t) {
  // Affichage debug avec la mémoire libre ESP32
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
              (void*)t, count, head, tail, freeRam());
}

void TaskManager::tick() {
  /*
    Loop logic :
      1. check ISR -> check if ISR flag is up and manage it.
      2. If no tasks active, start next task.
      3. if a task is active (unfinished) task -> continue (update) this task.
      4. clean-up finished tasks.
      5. update ISR timer to run every 100ms.
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

  // 4. cleanup finished
  if (active && active->isFinished()) {
    debugPrintf(DBG_TASKMANAGER, "tick: active finished -> deleting task ptr=%p", (void*)active);
    delete active; // Important: on libère la mémoire
    active = nullptr;
  }

  // 5. periodic updateISR every 100ms
  unsigned long now = millis();
  if (now - lastISRupdateMs >= 100) {
    lastISRupdateMs = now;
    updateISR();
  }
}

void TaskManager::updateISR() {
  /* Emergency stop checking HERE
    - called from tick() loop every 100ms
  */

  // --- GESTION BOUTON D'URGENCE ---
  // Assurez-vous que 'emergencyBtn' est bien déclaré dans globals.h
  // Si ce n'est pas le cas, commentez ce bloc pour l'instant
  /*
  {
    uint8_t btnFlags = emergencyBtn.pollFlags();
    if (btnFlags) {
      requestISR(btnFlags);
      debugPrintf(DBG_TASKMANAGER, "Emergency Button Pressed !");
    }
  }
  */

  // ==================================
  if (isrRequested) {
    doISR(); // handle any pending ISR first
    return;
  }
  
  // Si une tâche a besoin d'updates périodiques
  if (active && !active->isFinished()) {
    active->updateISR(*mv);
  }
}

void TaskManager::doISR() {
  // copy and clear flags atomically
  // Sur ESP32, noInterrupts fonctionne mais attention aux cœurs multiples
  // Pour l'instant c'est ok.
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
      delete active; 
      active = nullptr;
      return;
    }
    if (act == TaskInterruptAction::PAUSE) {
      mv->stop();
      return;
    }
    if (act == TaskInterruptAction::HANDLE) {
      return;
    }
  }

  // otherwise, do a global ISR handling policy
  if (flags & ISR_FLAG_EMERGENCY) {
    if (active) active->cancel(*mv);
    cancelAll();
    mv->stop();
  }
}

void TaskManager::requestISR(uint8_t flags) {
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