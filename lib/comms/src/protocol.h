
#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <Arduino.h>
#include <ArduinoJson.h>

namespace protocol {

void init(HardwareSerial& serial, unsigned long baudrate);
void log(const char* msg);
bool send(const JsonDocument& doc, HardwareSerial& serial);
bool send_event(const char* key, const char* value, HardwareSerial& serial);
bool send_int(const char* key, int value, HardwareSerial& serial);
bool receive(JsonDocument& doc, HardwareSerial& serial);

} // namespace protocol

#endif  // PROTOCOL
