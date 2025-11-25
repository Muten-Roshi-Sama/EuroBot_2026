#pragma once
#include <Arduino.h>

namespace protocol_esp32 {

// Initialise WiFi et UART (à appeler dans setup)
void begin_wifi_and_uart(HardwareSerial& uart = Serial2, int rx = 9, int tx = 10, unsigned long baud = 115200);

// Pont UART <-> WiFi (à appeler dans loop)
void bridge_uart_wifi_loop();

// Accès à la configuration centralisée (optionnel)
extern const char* ssid;
extern const char* password;
extern const char* server_ip;
extern const uint16_t server_port;

} // namespace protocol_esp32
