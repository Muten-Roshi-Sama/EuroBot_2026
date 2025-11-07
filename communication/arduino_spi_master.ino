/*
README (Arduino UNO SPI Master → ESP32-C3 SPI Slave, NDJSON bridge)
-------------------------------------------------------------------
Purpose: The Arduino UNO is the SPI master. It periodically sends telemetry
NDJSON to the ESP32-C3 and receives NDJSON commands back via SPI.

Framing over SPI:
- Fixed frame size: 2 + 512 bytes (LEN_L, LEN_H, PAYLOAD[0..511])
- LEN=0..512 indicates number of valid payload bytes in this frame.
- Payload contains a single NDJSON line (UTF-8, ending with '\n').
- Each SPI transaction transfers one full frame in both directions.

Wiring (UNO master):
- SCK  -> ESP32-C3 SCLK
- MOSI -> ESP32-C3 MOSI
- MISO <- ESP32-C3 MISO
- SS   -> ESP32-C3 SS (slave select)
- GND shared between boards.

Build:
- Install ArduinoJson (v6+). Upload to UNO.
- Open Serial Monitor at 115200 for debug (#-prefixed lines).

Notes:
- We keep the JSON stream clean for ESP side; debug is on USB Serial only.
*/

#include <Arduino.h>
#include <SPI.h>
#include <ArduinoJson.h>

// CONFIG --------------------------------------------------------------
static const uint8_t PIN_SS = 10; // UNO SS
static const uint32_t SPI_CLOCK_HZ = 1000000UL; // 1 MHz (safe)
static const uint8_t SPI_MODE = SPI_MODE0;
static const size_t MAX_JSON_SIZE = 512;
static const uint32_t TELEMETRY_PERIOD_MS = 1000;

// Frame constants
static const size_t FRAME_DATA_LEN = 512;
static const size_t FRAME_LEN = 2 + FRAME_DATA_LEN; // 514 bytes

// State ---------------------------------------------------------------
static uint32_t lastTelemetryMs = 0;
static int currentSpeed = 0;

// Helpers -------------------------------------------------------------
static void buildTelemetry(char *out, size_t &outLen) {
  StaticJsonDocument<MAX_JSON_SIZE> doc;
  doc["type"] = "telemetry";
  doc["ts"] = millis();
  doc["battery"] = 92;
  doc["speed"] = currentSpeed;
  // Serialize with trailing newline (NDJSON)
  String s;
  serializeJson(doc, s);
  s += '\n';
  outLen = min((size_t)s.length(), (size_t)MAX_JSON_SIZE);
  memcpy(out, s.c_str(), outLen);
}

static size_t encodeFrame(uint8_t *frame, const uint8_t *payload, size_t payloadLen) {
  if (payloadLen > FRAME_DATA_LEN) payloadLen = FRAME_DATA_LEN;
  frame[0] = (uint8_t)(payloadLen & 0xFF);
  frame[1] = (uint8_t)((payloadLen >> 8) & 0xFF);
  if (payloadLen) memcpy(frame + 2, payload, payloadLen);
  if (FRAME_DATA_LEN > payloadLen) memset(frame + 2 + payloadLen, 0, FRAME_DATA_LEN - payloadLen);
  return FRAME_LEN;
}

static size_t decodeFrame(const uint8_t *frame, uint8_t *payloadOut) {
  size_t len = (size_t)frame[0] | ((size_t)frame[1] << 8);
  if (len > FRAME_DATA_LEN) len = FRAME_DATA_LEN;
  if (len) memcpy(payloadOut, frame + 2, len);
  return len;
}

static void spiTransfer514(const uint8_t *tx514, uint8_t *rx514) {
  // Assert SS and transfer fixed 514 bytes
  digitalWrite(PIN_SS, LOW);
  for (size_t i = 0; i < FRAME_LEN; ++i) {
    rx514[i] = SPI.transfer(tx514[i]);
  }
  digitalWrite(PIN_SS, HIGH);
}

static void sendLineIfAnyAndReceive(char *rxLineBuf, size_t &rxLen) {
  // Prepare TX frame (possibly empty)
  uint8_t txFrame[FRAME_LEN];
  uint8_t rxFrame[FRAME_LEN];
  char line[MAX_JSON_SIZE];
  size_t lineLen = 0;

  // Telemetry every TELEMETRY_PERIOD_MS
  if (millis() - lastTelemetryMs >= TELEMETRY_PERIOD_MS) {
    lastTelemetryMs = millis();
    buildTelemetry(line, lineLen);
  }

  size_t toSend = encodeFrame(txFrame, (const uint8_t *)line, lineLen);
  (void)toSend; // always FRAME_LEN
  spiTransfer514(txFrame, rxFrame);

  // Decode response frame
  uint8_t payload[FRAME_DATA_LEN];
  size_t got = decodeFrame(rxFrame, payload);
  if (got > 0 && got <= MAX_JSON_SIZE) {
    memcpy(rxLineBuf, payload, got);
    rxLen = got;
  } else {
    rxLen = 0;
  }
}

static void handleIncomingLine(const char *line, size_t len) {
  // Expect NDJSON line
  StaticJsonDocument<MAX_JSON_SIZE> doc;
  DeserializationError err = deserializeJson(doc, line, len);
  if (err) {
    Serial.print('#'); Serial.print("Invalid JSON from SPI: "); Serial.write(line, len); Serial.println();
    return;
  }
  const char *type = doc["type"] | "command";
  if (strcmp(type, "command") == 0) {
    const char *cmd = doc["command"] | "";
    if (strcmp(cmd, "set_speed") == 0) {
      currentSpeed = (int)doc["value"] | currentSpeed;
      Serial.print('#'); Serial.print("Set speed to "); Serial.println(currentSpeed);
    }
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }
  Serial.println("# UNO SPI master NDJSON starting...");

  pinMode(PIN_SS, OUTPUT);
  digitalWrite(PIN_SS, HIGH);
  SPI.begin();
  SPI.beginTransaction(SPISettings(SPI_CLOCK_HZ, MSBFIRST, SPI_MODE));
}

void loop() {
  char rxLine[MAX_JSON_SIZE];
  size_t rxLen = 0;
  sendLineIfAnyAndReceive(rxLine, rxLen);
  if (rxLen > 0) {
    handleIncomingLine(rxLine, rxLen);
  }
  delay(5); // pacing
}


