#ifndef PROTOCOL_ESP32_H
#define PROTOCOL_ESP32_H

#include <Arduino.h>

namespace protocol_esp32 {

void begin_wifi_and_uart(HardwareSerial& uart, int rx, int tx, unsigned long baud);
void bridge_uart_wifi_loop();

} // namespace protocol_esp32

#endif // PROTOCOL_ESP32_H
