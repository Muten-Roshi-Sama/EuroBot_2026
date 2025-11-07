/*
README (ESP32-C3 SPI Slave <-> Wi-Fi TCP NDJSON Bridge)
-------------------------------------------------------
Purpose: ESP32-C3 acts as SPI slave to an Arduino UNO master. It bridges
NDJSON messages between SPI and a single TCP client over 2.4 GHz Wi‑Fi.

SPI Frame format (per transaction, full-duplex):
- 514 bytes: [LEN_L, LEN_H, PAYLOAD[0..511]]
- LEN=0..512 indicates number of valid payload bytes. Payload is NDJSON line
  (UTF-8, ending with '\n'). Each transaction can carry at most one line.
- Both directions use the same frame size.

Wi‑Fi: 2.4 GHz only (ESP32‑C3). Starts TCP server on port 3333, single client.
JSON: Validates with ArduinoJson. Optional pre‑shared key filtering on commands.
*/

#include <WiFi.h>
#include <ArduinoJson.h>
#include <esp_task_wdt.h>
#include <ESP32SPISlave.h>

// CONFIG --------------------------------------------------------------
// Wi‑Fi credentials
static const char *WIFI_SSID = "YOUR_SSID";
static const char *WIFI_PASS = "YOUR_PASSWORD";
static const char *HOSTNAME  = "esp32c3-spi-bridge";
static const uint16_t TCP_PORT = 3333;

// SPI pins (adjust for your ESP32‑C3 board)
// These are examples; verify your board's available GPIOs.
static const int PIN_MOSI = 6;   // connect to UNO MOSI
static const int PIN_MISO = 7;   // connect to UNO MISO
static const int PIN_SCLK = 4;   // connect to UNO SCK
static const int PIN_SS   = 5;   // connect to UNO SS

// JSON & framing
static const size_t MAX_JSON_SIZE = 512;  // bytes
static const size_t FRAME_DATA_LEN = 512;
static const size_t FRAME_LEN = 2 + FRAME_DATA_LEN; // 514 bytes

// Security (optional pre‑shared key). Empty string disables check.
static const char *PRE_SHARED_KEY = ""; // e.g. "secret123"

// Behavior
#define DEBUG 1
static const bool SEND_RELAY_ACK = true;

// Wi‑Fi backoff
static const uint32_t WIFI_RETRY_BASE_MS = 500;
static const uint32_t WIFI_RETRY_MAX_MS  = 8000;

// Watchdog
static const uint32_t WDT_TIMEOUT_S = 2;

// Globals --------------------------------------------------------------
WiFiServer server(TCP_PORT);
WiFiClient client;
ESP32SPISlave spiSlave;

static uint8_t spiRxBuf[FRAME_LEN];
static uint8_t spiTxBuf[FRAME_LEN];

static uint32_t seqCounter = 1;

// Utils ----------------------------------------------------------------
static void debugln(const String &s) {
#if DEBUG
  Serial.print('#');
  Serial.println(s);
#endif
}

static void debug(const String &s) {
#if DEBUG
  Serial.print('#');
  Serial.print(s);
#endif
}

static void frameClearTx() {
  memset(spiTxBuf, 0, FRAME_LEN);
}

static void frameFromLine(const uint8_t *line, size_t len) {
  if (len > FRAME_DATA_LEN) len = FRAME_DATA_LEN;
  spiTxBuf[0] = (uint8_t)(len & 0xFF);
  spiTxBuf[1] = (uint8_t)((len >> 8) & 0xFF);
  if (len) memcpy(spiTxBuf + 2, line, len);
  if (FRAME_DATA_LEN > len) memset(spiTxBuf + 2 + len, 0, FRAME_DATA_LEN - len);
}

static size_t frameToLine(uint8_t *out) {
  size_t len = (size_t)spiRxBuf[0] | ((size_t)spiRxBuf[1] << 8);
  if (len > FRAME_DATA_LEN) len = FRAME_DATA_LEN;
  if (len) memcpy(out, spiRxBuf + 2, len);
  return len;
}

static void sendJsonLine(WiFiClient &c, const JsonDocument &doc) {
  if (!c) return;
  String out; serializeJson(doc, out); out += '\n';
  c.print(out);
}

static void sendErrorJson(WiFiClient &c, const char *reason) {
  StaticJsonDocument<64> err;
  err["type"] = "error";
  err["reason"] = reason;
  sendJsonLine(c, err);
}

static bool requireKeyOk(const JsonDocument &doc) {
  if (PRE_SHARED_KEY == nullptr || PRE_SHARED_KEY[0] == '\0') return true;
  const char *key = doc["key"] | "";
  return strcmp(key, PRE_SHARED_KEY) == 0;
}

static void ensureWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;
  debugln("Wi‑Fi connecting...");
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(HOSTNAME);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  uint32_t backoff = WIFI_RETRY_BASE_MS;
  while (WiFi.status() != WL_CONNECTED) {
    delay(backoff);
    backoff = min(backoff * 2, WIFI_RETRY_MAX_MS);
    debug("Connecting... status="); debugln(String(WiFi.status()));
  }
  debug("Wi‑Fi connected, IP: "); debugln(WiFi.localIP().toString());
}

static void handleNewClient() {
  if (!server.hasClient()) return;
  WiFiClient incoming = server.available();
  if (!incoming) return;
  if (client && client.connected()) { incoming.stop(); debugln("Second client refused"); return; }
  client = incoming; debugln("Client connected: " + client.remoteIP().toString());
}

static void dropClient() { if (client) { client.stop(); debugln("Client disconnected"); } }

void setup() {
  Serial.begin(115200);
  delay(100);
  debugln("ESP32‑C3 SPI slave NDJSON bridge (2.4 GHz)");

  // SPI slave setup
  spiSlave.setDataMode(SPI_MODE0);
  spiSlave.setQueueSize(1);
  spiSlave.begin(PIN_SCLK, PIN_MISO, PIN_MOSI, PIN_SS);
  frameClearTx();

  // Wi‑Fi and server
  ensureWiFi();
  server.begin();
  server.setNoDelay(true);
  debug("TCP server on port "); debugln(String(TCP_PORT));

  // WDT
  esp_task_wdt_init(WDT_TIMEOUT_S, true);
  esp_task_wdt_add(NULL);
}

void loop() {
  ensureWiFi();
  handleNewClient();

  // Prepare a response for SPI (if there's a pending line from TCP)
  bool prepared = false;
  if (client && client.connected() && client.available()) {
    // Read one line from TCP and validate, then place to SPI TX buffer
    static char tcpLine[MAX_JSON_SIZE];
    size_t idx = 0;
    while (client.available() && idx < MAX_JSON_SIZE) {
      int c = client.read();
      if (c < 0) break;
      if (c == '\r') continue;
      tcpLine[idx++] = (char)c;
      if (c == '\n') break;
    }
    if (idx > 0) {
      StaticJsonDocument<MAX_JSON_SIZE> doc;
      DeserializationError err = deserializeJson(doc, tcpLine, idx);
      if (err) { sendErrorJson(client, "invalid_json"); }
      else if (requireKeyOk(doc)) {
        frameFromLine((const uint8_t*)tcpLine, idx);
        prepared = true;
      } else {
        debugln("Command rejected: bad key");
      }
    }
  }
  if (!prepared) { frameClearTx(); }

  // Execute one SPI transaction (514 bytes)
  spiSlave.queue(spiTxBuf, spiRxBuf, FRAME_LEN);
  spiSlave.yield(); // wait for master transaction

  // Process received SPI line (if any)
  {
    static uint8_t rxPayload[FRAME_DATA_LEN];
    size_t rxLen = frameToLine(rxPayload);
    if (rxLen > 0) {
      StaticJsonDocument<MAX_JSON_SIZE> doc;
      DeserializationError err = deserializeJson(doc, rxPayload, rxLen);
      if (err) {
        if (client && client.connected()) sendErrorJson(client, "invalid_json");
      } else {
        // Ensure seq exists
        if (!doc.containsKey("seq")) doc["seq"] = seqCounter++;
        if (client && client.connected()) {
          sendJsonLine(client, doc);
          if (SEND_RELAY_ACK) {
            StaticJsonDocument<64> ack; ack["type"] = "relay_ack"; ack["seq"] = (int)doc["seq"]; sendJsonLine(client, ack);
          }
        }
      }
    }
  }

  if (client && !client.connected()) dropClient();
  esp_task_wdt_reset();
}


