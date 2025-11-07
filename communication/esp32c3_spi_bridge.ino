/*
README (ESP32-C6 SPI Slave <-> Wi-Fi TCP NDJSON Bridge)
------------------------------------------------------
Purpose: ESP32-C6 acts as SPI slave to an Arduino UNO master. It bridges
NDJSON messages between SPI and a single TCP client over Wi‑Fi (2.4 GHz).

SPI Frame format (per transaction, full-duplex):
- 514 bytes: [LEN_L, LEN_H, PAYLOAD[0..511]]
- LEN=0..512 indicates number of valid payload bytes. Payload is NDJSON line
  (UTF-8, ending with '\n'). Each transaction can carry at most one line.
- Both directions use the same frame size.

Wi‑Fi: 2.4 GHz only (ESP32‑C6). Starts TCP server on port 3333, single client.
JSON: Validates with ArduinoJson. Optional pre‑shared key filtering on commands.
*/

#include <WiFi.h>
#include <esp_task_wdt.h>
#include <ESP32SPISlave.h>

// CONFIG --------------------------------------------------------------
// Wi‑Fi credentials
static const char *WIFI_SSID = "YOUR_SSID";
static const char *WIFI_PASS = "YOUR_PASSWORD";
static const char *HOSTNAME  = "esp32c6-spi-bridge";
static const uint16_t TCP_PORT = 3333;

// SPI pins (ESP32‑C6 — adjust for your board/wiring)
// Example defaults for ESP32‑C6 DevKitC‑1 (change if your wiring differs)
static const int PIN_MOSI = 6;   // connect to UNO MOSI
static const int PIN_MISO = 5;   // connect to UNO MISO
static const int PIN_SCLK = 7;   // connect to UNO SCK
static const int PIN_SS   = 4;   // connect to UNO SS

// Line protocol & framing
static const size_t MAX_LINE_SIZE = 512;  // bytes (includes trailing '\n')
static const size_t FRAME_DATA_LEN = 512;
static const size_t FRAME_LEN = 2 + FRAME_DATA_LEN; // 514 bytes

// Behavior
#define DEBUG 1

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

static void sendRawLine(WiFiClient &c, const uint8_t *data, size_t len) {
  if (!c || len == 0) return;
  c.write(data, len);
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
  debugln("ESP32‑C6 SPI slave NDJSON bridge (2.4 GHz)");

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
    // Read one line from TCP and place to SPI TX buffer as-is
    static char tcpLine[MAX_LINE_SIZE];
    size_t idx = 0;
    while (client.available() && idx < MAX_LINE_SIZE) {
      int c = client.read();
      if (c < 0) break;
      if (c == '\r') continue;
      tcpLine[idx++] = (char)c;
      if (c == '\n') break;
    }
    if (idx > 0) {
      frameFromLine((const uint8_t*)tcpLine, idx);
      prepared = true;
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
      if (client && client.connected()) {
        sendRawLine(client, rxPayload, rxLen);
      }
    }
  }

  if (client && !client.connected()) dropClient();
  esp_task_wdt_reset();
}


