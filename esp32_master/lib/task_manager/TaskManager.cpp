#include "TaskManager.h"

// Includes
#include <globals.h>
#include <isr_flags.h>
// #include <i2c_addr.h>
// #include "../detection/capteur_lidar.h"

// Libs
// #include "../movement/Movement.h"
#include "../util/Debug.h"

// ==========================================

// Diagnostic helpers (place near top of TaskManager.cpp)
extern "C" char *__brkval; // avr-libc bookkeeping
extern "C" char __heap_start;
static int freeRam()
{
  char stack_dummy = 0;
  char *heap_end = __brkval ? __brkval : &__heap_start;
  return (int)&stack_dummy - (int)heap_end;
}

// TaskManager* TaskManager::instance = nullptr;   // DECLARED IN /src/GLOBAL.CPP

TaskManager::TaskManager()
    : head(0), tail(0), count(0), active(nullptr),
      lastISRupdateMs(0), pendingIsrFlags(0), isrRequested(false), lastTickMs(0)
{
  for (int i = 0; i < MAX_TASKS; ++i)
    queue[i] = nullptr;
}

void TaskManager::addTask(Task *t)
{
  debugPrintf(DBG_TASKMANAGER, "addTask called ptr=%p count=%d",(void *)t, count);

  // if (!t) {
  //   Serial.println("ERROR: Task pointer is NULL!");
  //   return;
  // }

  if (count >= MAX_TASKS)
  {
    Serial.println("Task queue full");
    return;
  }
  queue[tail] = t;
  tail = (tail + 1) % MAX_TASKS;
  count++;
}

void TaskManager::tick(void* context)
{
  // 1. fast top-level ISR check
  if (isrRequested) {doISR(context); }

  // 2. start next if no active
  if (!active)
    {
      if (count == 0) return;
      active = queue[head];     // fetch next task from queue
      queue[head] = nullptr;
      head = (head + 1) % MAX_TASKS; // ptr management
      count--;
      if (active) {active->start(context);} // call next task
    }

  // 3. run active task update (non-blocking)
  if (active && !active->isFinished()){ active->update(context);}

  // 4. cleanup finished Tasks
  if (active && active->isFinished())
  {
    debugPrintf(DBG_TASKMANAGER, "tick: active finished -> deleting task ptr=%p", (void *)active);
    delete active; // if allocated on heap; otherwise return to pool
    active = nullptr;
  }

  // 5. periodic updateISR every 100ms
  unsigned long now = millis();
  if (now - lastISRupdateMs >= 500) {
    lastISRupdateMs = now;
    updateISR(context);
    // debugPrintf(DBG_TASKMANAGER, "ISR_Update");
  }
}

void TaskManager::updateISR(void* context)
{
  /* Emergency stop checking HERE
    - called from tick() loop every 100ms
  */

  // BUTTON
  // {
  //   uint8_t btnFlags = emergencyBtn.pollFlags();
  //   debugPrintf(DBG_TASKMANAGER, "Emergency btn flags: 0x%02X", btnFlags);  // NEW
    
  //   if (btnFlags) {
  //     requestISR(btnFlags);
  //     debugPrintf(DBG_TASKMANAGER, "Emergency Button Pressed!");
  //   }
  // }

  // Lecture des 3 LIDARs
  // int distance1 = lireDistanceLidar1();
  // int distance2 = lireDistanceLidar2();
  // int distance3 = lireDistanceLidar3();

  // Affichage condensé sur une seule ligne
  // debugPrintf(DBG_TASKMANAGER, "L1:%dmm L2:%dmm L3:%dmm", distance1, distance2, distance3);

  // // Gestion ISR LIDAR1 (seuil 80mm)
  // if (distance1 > 0 && distance1 <= LIDAR1_THRESHOLD) {
  //   requestISR(ISR_FLAG_OBSTACLE);
  //   debugPrintf(DBG_TASKMANAGER, "LIDAR1 OBSTACLE!");
  // }
  // else if (distance1 > LIDAR1_THRESHOLD && (pendingIsrFlags & ISR_FLAG_OBSTACLE)) {
  //   requestISR(ISR_FLAG_OBSTACLE_CLEARED);
  // }

  // // Gestion ISR LIDAR2 (seuil 80mm)
  // if (distance2 > 0 && distance2 <= LIDAR2_THRESHOLD) {
  //   requestISR(ISR_FLAG_OBSTACLE);
  //   debugPrintf(DBG_TASKMANAGER, "LIDAR2 OBSTACLE!");
  // }
  // else if (distance2 > LIDAR2_THRESHOLD && (pendingIsrFlags & ISR_FLAG_OBSTACLE)) {
  //   requestISR(ISR_FLAG_OBSTACLE_CLEARED);
  // }

  // // Gestion ISR LIDAR3 (seuil 100mm)
  // if (distance3 > 0 && distance3 <= LIDAR3_THRESHOLD) {
  //   requestISR(ISR_FLAG_OBSTACLE);
  //   debugPrintf(DBG_TASKMANAGER, "LIDAR3 OBSTACLE!");
  // }
  // else if (distance3 > LIDAR3_THRESHOLD && (pendingIsrFlags & ISR_FLAG_OBSTACLE)) {
  //   requestISR(ISR_FLAG_OBSTACLE_CLEARED);
  // }

  // ==================================
  if (isrRequested)                    {doISR(context);}
  if (active && !active->isFinished()) {active->updateISR(context);}

}


void TaskManager::doISR(void* context) {
  // copy and clear flags atomically
  // noInterrupts();
  // uint8_t flags = pendingIsrFlags;
  // pendingIsrFlags = 0;
  // isrRequested = false;
  // interrupts();

  // // If there is an active task, let it handle first
  // if (active && !active->isFinished())
  // {
  //   TaskInterruptAction act = active->handleInterrupt(context, flags);
  //   if (act == TaskInterruptAction::CANCEL)
  //   {
  //     active->cancel(context);
  //     delete active;
  //     active = nullptr;
  //     return;
  //   }
  //   if (act == TaskInterruptAction::PAUSE)
  //   {
  //     // keep active in paused state (we use cancel for simplicity)
  //     // move it to head of queue or leave as paused (implementation choice)
  //     mv->stop();
  //     return;
  //   }
  //   if (act == TaskInterruptAction::HANDLE)
  //   {
  //     // active handled it fully, nothing more to do
  //     return;
  //   }
  // }

  // // otherwise, do a global ISR handling policy (e.g., emergency)
  // if (flags & ISR_FLAG_EMERGENCY)
  // {
  //   // immediate emergency handling: stop all tasks
  //   if (active)
  //     active->cancel(context);
  //   cancelAll();
  //   mv->stop();
  //   // Optionally set global FSM emergency flag here
  // }
  // // gestion du flag obstacle
  // if (flags & ISR_FLAG_OBSTACLE) {
  //     debugPrintf(DBG_TASKMANAGER, "Obstacle très proche, arrêt du robot");
      
  //     // Arrêt immédiat du robot
  //     if (mv) mv->stop();    

  // }
  // if (flags & ISR_FLAG_OBSTACLE_CLEARED) {
  //     debugPrintf(DBG_TASKMANAGER, "Obstacle dégagé, reprise TASK possible");
  // }
}

void TaskManager::requestISR(uint8_t flags)
{
  // safe to call from ISR:
  pendingIsrFlags |= flags;
  isrRequested = true;
}

void TaskManager::cancelAll()
{
  // // cancel active
  // if (active)
  // {
  //   active->cancel(context);
  //   delete active;
  //   active = nullptr;
  // }
  // // clear queue
  // while (count > 0)
  // {
  //   Task *t = queue[head];
  //   if (t)
  //     delete t;
  //   queue[head] = nullptr;
  //   head = (head + 1) % MAX_TASKS;
  //   count--;
  // }
  // head = tail = 0;
}

Task *TaskManager::currentTask() const { return active; }
bool TaskManager::isIdle() const {return (active == nullptr) && (count == 0);}
