#pragma once
#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>

namespace espnow_receiver {
    void begin();
    void detection_loop();
}
