// code de l'esp32 pour faire le pont UART <-> WiFi
#include <Arduino.h>
#include "protocol_esp32.h"

using namespace protocol_esp32;

void setup() {
    Serial.begin(115200);
    // UART2: RX=9, TX=10, baud=115200
    // RX=20, TX=21, baud=115200 pour le C3

    begin_wifi_and_uart(Serial2, 9, 10, 115200);
    Serial.println("[BOOT] ESP32 WiFi↔UART bridge démarré");
}

void loop() {
    // Boucle principale : tout est géré dans bridge_uart_wifi_loop
    bridge_uart_wifi_loop();
    // Pas de delay ici : tout est non-bloquant
}
