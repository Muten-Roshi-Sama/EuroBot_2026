
#ifndef PROTOCOL_H
#define PROTOCOL_H
#include <Arduino.h>
#include <ArduinoJson.h>

namespace protocol {


void init(HardwareSerial& serial = Serial, unsigned long baudrate = 115200);
void log(const char* msg);
void check_button(uint8_t button_pin = 4);
bool receive(JsonDocument& doc, HardwareSerial& serial = Serial);

} // namespace protocol
#endif  // PROTOCOL
