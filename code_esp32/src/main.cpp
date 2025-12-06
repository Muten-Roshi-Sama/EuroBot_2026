#include <Arduino.h>
#include "espnow_receiver.h"
// #include "protocol_esp32.h" // Inutile pour test ESP-NOW pur

//using namespace protocol_esp32; // Inutile pour test ESP-NOW pur

void setup() {
    Serial.begin(115200);
    // UART2: RX=9, TX=10, baud=115200
    // RX=20, TX=21, baud=115200 pour le C3
    begin_wifi_and_uart(Serial2, 9, 10, 115200);
    Serial.println("[BOOT] ESP32 WiFi↔UART bridge + ESP-NOW ping-pong (broadcast)");
    espnow_receiver::begin(); // Initialisation ESP-NOW
}

void loop() {
    bridge_uart_wifi_loop(); // Inutile pour test ESP-NOW pur
    espnow_receiver::detection_loop(); // Ping toutes les secondes
    // Pas de delay ici : tout est non-bloquant
}
