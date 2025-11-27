#include <Arduino.h>
#include "protocol_esp32.h"

using namespace protocol_esp32;

void setup() {
    Serial.begin(115200);
    begin_wifi_and_uart(Serial2, 9, 10, 115200);
}

void loop() {
    bridge_uart_wifi_loop();
}
