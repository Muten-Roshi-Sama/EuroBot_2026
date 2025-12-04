
#include <Arduino.h>
#include "espnow_receiver.h"
#include "protocol_esp32.h"

using namespace protocol_esp32;

void setup() {
    Serial.begin(115200);
    // UART2: RX=9, TX=10, baud=115200
    // RX=20, TX=21, baud=115200 pour le C3
    begin_wifi_and_uart(Serial2, 9, 10, 115200);
    Serial.println("[BOOT] ESP32 WiFi↔UART bridge + ping-pong démarré");
    espnow_receiver::begin();
}

void loop() {
    bridge_uart_wifi_loop();
    espnow_receiver::detection_loop();
    // Pas de delay ici : tout est non-bloquant
}
