#include "FSMFlags.h"

FSMFlags::FSMFlags()
    : flag_ready(false),
      flag_trigger_launch(false),
      flag_emergency_stop(false)
{
    flags_mutex = xSemaphoreCreateMutex();
}

// === SETTERS ===
void FSMFlags::set_ready(bool value) {
    xSemaphoreTake(flags_mutex, portMAX_DELAY);
    flag_ready = value;
    xSemaphoreGive(flags_mutex);
}

void FSMFlags::set_trigger_launch(bool value) {
    xSemaphoreTake(flags_mutex, portMAX_DELAY);
    flag_trigger_launch = value;
    xSemaphoreGive(flags_mutex);
}

void FSMFlags::set_emergency_stop(bool value) {
    xSemaphoreTake(flags_mutex, portMAX_DELAY);
    flag_emergency_stop = value;
    xSemaphoreGive(flags_mutex);
}

// === GETTERS ===
bool FSMFlags::get_ready() const {
    xSemaphoreTake(flags_mutex, portMAX_DELAY);
    bool value = flag_ready;
    xSemaphoreGive(flags_mutex);
    return value;
}

bool FSMFlags::get_trigger_launch() const {
    xSemaphoreTake(flags_mutex, portMAX_DELAY);
    bool value = flag_trigger_launch;
    xSemaphoreGive(flags_mutex);
    return value;
}

bool FSMFlags::get_emergency_stop() const {
    xSemaphoreTake(flags_mutex, portMAX_DELAY);
    bool value = flag_emergency_stop;
    xSemaphoreGive(flags_mutex);
    return value;
}

// === RESET ===
void FSMFlags::clear_trigger_launch() {
    xSemaphoreTake(flags_mutex, portMAX_DELAY);
    flag_trigger_launch = false;
    xSemaphoreGive(flags_mutex);
}

void FSMFlags::clear_emergency_stop() {
    xSemaphoreTake(flags_mutex, portMAX_DELAY);
    flag_emergency_stop = false;
    xSemaphoreGive(flags_mutex);
}
