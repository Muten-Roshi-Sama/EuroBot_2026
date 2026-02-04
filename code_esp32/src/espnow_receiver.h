#pragma once
#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>

// CONFIGURATION :
#define ESPNOW_ROLE_INITIATOR 1 // 1 = INITIATOR, 0 = RESPONDER
namespace espnow_receiver {
    void begin();
    void detection_loop();
}
