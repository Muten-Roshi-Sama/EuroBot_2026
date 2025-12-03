#include "TaskManager.h"

// Includes
#include "globals.h"
#include "isr_flags.h"
#include "../detection/capteur_lidar.h"

// Libs
#include "../movement/Movement.h"
#include "../util/Debug.h"


// ==========================================

// TaskManager* TaskManager::instance = nullptr;


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
  debugPrintf(DBG_TASKMANAGER, "[TaskManager.cpp] : AddTask ptr=0x%08lX", (unsigned long)t);
}

void TaskManager::tick() {
  /*
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
    delete active; // if allocated on heap; otherwise return to pool
    active = nullptr;
  }

  // periodic updateISR every 100ms
  unsigned long now = millis();
  if (now - lastISRupdateMs >= 500) {
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
  {
    int distance = lireDistance();  // lecture non bloquante
    // DEBUG : toujours afficher la distance
    static unsigned long lastLidarDebug = 0;
    if (millis() - lastLidarDebug > 500) {
        debugPrintf(DBG_TASKMANAGER, "LIDAR distance = %d mm (obstacleActive=%d)", 
                    distance, obstacleActive);
        lastLidarDebug = millis();
    }
    
    if (distance > 0 && distance <= 50 && !obstacleActive) {
        requestISR(ISR_FLAG_OBSTACLE);
        obstacleActive = true;
        debugPrintf(DBG_TASKMANAGER, "ISR: Obstacle detecté à %d mm", distance);
    }
    else if ((distance == 0 || distance > 50) && obstacleActive) {
        requestISR(ISR_FLAG_OBSTACLE_CLEARED);
        obstacleActive = false;
        debugPrintf(DBG_TASKMANAGER, "✓ Obstacle dégagé");
    }
  }
  // ==================================
  if (isrRequested) {
    doISR(); // handle any pending ISR first
    //return;
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
  // gestion du flag obstacle
  if (flags & ISR_FLAG_OBSTACLE) {
      debugPrintf(DBG_TASKMANAGER, "Obstacle très proche, arrêt du robot");
      
      // Arrêt immédiat du robot
      if (mv) mv->stop();    

  }
  if (flags & ISR_FLAG_OBSTACLE_CLEARED) {
      debugPrintf(DBG_TASKMANAGER, "Obstacle dégagé, reprise TASK possible");
  }
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