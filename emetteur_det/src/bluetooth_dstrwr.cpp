// Bluetooth DS-TWR (squelette)

#include <Arduino.h>
#include <NimBLEDevice.h>

// CONFIGURATION :
#define ROLE_INITIATOR 0 // 1 = INITIATOR, 0 = RESPONDER

// Structure DS-TWR
struct RangingPacket {
    uint8_t  type;  // 0 = PING, 1 = PONG
    uint64_t t1;
    uint64_t t2;
    uint64_t t3;
};

static inline uint64_t get_cycles() {
    return (uint64_t)esp_cpu_get_ccount();
}
static constexpr double CYCLES_TO_SEC = 1.0 / 240000000.0;
static constexpr double SPEED_OF_LIGHT = 299792458.0;
static inline uint64_t cycles_diff(uint64_t a, uint64_t b) {
    return (a >= b) ? (a - b) : (UINT64_MAX - b + a + 1);
}

// BLE UUIDs
static NimBLEUUID SERVICE_UUID("6e400001-b5a3-f393-e0a9-e50e24dcca9e");
static NimBLEUUID CHAR_UUID_RX("6e400002-b5a3-f393-e0a9-e50e24dcca9e");
static NimBLEUUID CHAR_UUID_TX("6e400003-b5a3-f393-e0a9-e50e24dcca9e");

NimBLEServer* pServer = nullptr;
NimBLECharacteristic* pTxCharacteristic = nullptr;
NimBLECharacteristic* pRxCharacteristic = nullptr;

class RangingCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* pCharacteristic) {
        std::string value = pCharacteristic->getValue();
        Serial1.print("[BLE][RX] Write received, size: ");
        Serial1.println(value.length());
        if (value.length() == sizeof(RangingPacket)) {
            RangingPacket pkt;
            memcpy(&pkt, value.data(), sizeof(pkt));
            Serial1.print("[BLE][RX] Packet type: ");
            Serial1.println(pkt.type);
            if (pkt.type == 0) {
                uint64_t t2 = get_cycles();
                RangingPacket resp;
                resp.type = 1;
                resp.t1 = pkt.t1;
                resp.t2 = t2;
                resp.t3 = 0;
                delayMicroseconds(50);
                resp.t3 = get_cycles();
                pTxCharacteristic->setValue((uint8_t*)&resp, sizeof(resp));
                pTxCharacteristic->notify();
                Serial1.println("[PONG][BLE] Répondu");
            }
        } else {
            Serial1.println("[BLE][RX] Invalid packet size, ignored.");
        }
    }
};

void bluetooth_dstrwr_setup() {
    Serial1.begin(115200, SERIAL_8N1, 20, 21);
    Serial1.println("[BOOT] DS-TWR BLE (RESPONDER)");
    NimBLEDevice::init("ESP32C3_BLE_DSTRWR");
    pServer = NimBLEDevice::createServer();
    NimBLEService* pService = pServer->createService(SERVICE_UUID);
    pTxCharacteristic = pService->createCharacteristic(
        CHAR_UUID_TX, NIMBLE_PROPERTY::NOTIFY
    );
    pRxCharacteristic = pService->createCharacteristic(
        CHAR_UUID_RX, NIMBLE_PROPERTY::WRITE
    );
    pRxCharacteristic->setCallbacks(new RangingCallbacks());
    pService->start();

    NimBLEAdvertising* pAdvertising = NimBLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    // If you want to set scan response data, use setScanResponseData().
    // Example: pAdvertising->setScanResponseData("\x02\x01\x06");
    // Remove setScanResponse(true) as it's not available.
    pAdvertising->setMinInterval(0x20);  // 32 * 0.625ms = 20ms (example value)
    pAdvertising->setMaxInterval(0x40);  // 64 * 0.625ms = 40ms (example value)
    NimBLEDevice::startAdvertising();

    Serial1.print("[BLE] Service UUID: ");
    Serial1.println(SERVICE_UUID.toString().c_str());
    Serial1.print("[BLE] Char UUID TX: ");
    Serial1.println(CHAR_UUID_TX.toString().c_str());
    Serial1.print("[BLE] Char UUID RX: ");
    Serial1.println(CHAR_UUID_RX.toString().c_str());
    Serial1.println("[BLE] Advertising started (with service UUID)");
}

void bluetooth_dstrwr_loop() {
    static uint32_t lastLog = 0;
    if (millis() - lastLog > 1000) {
        Serial1.print("[BLE][STATE] millis: ");
        Serial1.print(millis());
        Serial1.print(" | Free heap: ");
        Serial1.println(ESP.getFreeHeap());
        Serial1.print("[BLE][STATE] Advertising: ");
        Serial1.println(NimBLEDevice::getAdvertising()->isAdvertising() ? "YES" : "NO");
        lastLog = millis();
    }
}
