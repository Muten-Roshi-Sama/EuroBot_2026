
#include <Arduino.h>
#include "settings.h"
#include "TaskManager.h"

class TaskManager {
public:
  enum State { IDLE, RUNNING, WAITING };
  TaskManager(Movement* mv): mv(mv), idx(0), state(IDLE) {}

  void addTask(const Task& t) { tasks.push_back(t); }

  void tick() {
    if (idx >= tasks.size()) { state = IDLE; return; }
    Task &cur = tasks[idx];
    if (state == IDLE) {
      // start task
      if (cur.type == TaskType::MOVE_DISTANCE) {
         targetTicks = mv->cmToTicks(cur.value);
         mv->resetEncoders();
         mv->forward(cur.speed == 0 ? DEFAULT_SPEED : cur.speed);
         state = RUNNING;
      } else if (cur.type == TaskType::ROTATE) {
         targetTicks = mv->degreesToTicks(cur.value);
         mv->resetEncoders();
         // start rotation: left backward right forward OR vice versa
         // choose direction depending on sign
         if (cur.value > 0) { mv->rotateRight(cur.speed); } else { mv->rotateLeft(cur.speed); }
         state = RUNNING;
      } else if (cur.type == TaskType::WAIT_MS) {
         waitUntil = millis() + (unsigned long)cur.value;
         state = WAITING;
      }
    } else if (state == RUNNING) {
      long left = abs(mv->getLeftTicks());
      long right = abs(mv->getRightTicks());
      if (cur.type == TaskType::MOVE_DISTANCE) {
         if (left >= targetTicks || right >= targetTicks) {
           mv->stop();
           idx++; state = IDLE;
         }
      } else if (cur.type == TaskType::ROTATE) {
         // compute angle from both wheels (better than a single)
         float angle = mv->ticksToDegrees((left + right)/2);
         if (abs(angle) >= abs(cur.value)) { mv->stop(); idx++; state = IDLE; }
      }
    } else if (state == WAITING) {
      if (millis() >= waitUntil) { idx++; state = IDLE; }
    }
  }

  bool isIdle() const { return state == IDLE && idx >= tasks.size(); }
private:
  Movement* mv;
  std::vector<Task> tasks;
  size_t idx;
  State state;
  long targetTicks;
  unsigned long waitUntil;
};