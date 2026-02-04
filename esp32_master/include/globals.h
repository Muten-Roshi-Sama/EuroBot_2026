#pragma once

// Global shared objects between modules
// Button driver included from lib path; PlatformIO adds lib/ to include paths.


#include <stdint.h>


// #include "../lib/drivers/button/Button.h"
// extern Button emergencyBtn;


#include "../lib/task_manager/TaskManager.h"
class TaskManager;
extern TaskManager* taskManager;


