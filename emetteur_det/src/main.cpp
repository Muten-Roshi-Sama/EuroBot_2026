#include <Arduino.h>
#include "bluetooth_dstrwr.h"

#define RXD2 20
#define TXD2 21

// Garder UART1 pour logs
void setup() {
    Serial1.begin(115200, SERIAL_8N1, RXD2, TXD2);
    delay(100);
    Serial1.println("[BOOT] DS-TWR ESP32C3 (UART1, 240 MHz, BLUETOOTH)");
    bluetooth_dstrwr_setup();
}

void loop() {
    // Rediriger les prints Bluetooth sur UART1
    bluetooth_dstrwr_loop();
}
