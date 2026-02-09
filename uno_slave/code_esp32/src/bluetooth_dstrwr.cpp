// Bluetooth DS-TWR (librairie)

#include "bluetooth_dstrwr.h"
#include <NimBLEDevice.h>

#define ROLE_INITIATOR 1 // 1 = INITIATOR, 0 = RESPONDER

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


// BLE UUIDs (doivent matcher le serveur)
static NimBLEUUID SERVICE_UUID("6e400001-b5a3-f393-e0a9-e50e24dcca9e");
static NimBLEUUID CHAR_UUID_RX("6e400002-b5a3-f393-e0a9-e50e24dcca9e");
static NimBLEUUID CHAR_UUID_TX("6e400003-b5a3-f393-e0a9-e50e24dcca9e");

NimBLEAdvertisedDevice* pAdvertisedDevice = nullptr;
NimBLEClient* pClient = nullptr;
NimBLERemoteCharacteristic* pTxCharacteristic = nullptr;
NimBLERemoteCharacteristic* pRxCharacteristic = nullptr;

volatile bool pongReceived = false;
RangingPacket lastPong;
uint64_t t1, t4;

void notifyCallback(NimBLERemoteCharacteristic* pCharacteristic, uint8_t* data, size_t length, bool isNotify) {
    if (length == sizeof(RangingPacket)) {
        memcpy(&lastPong, data, sizeof(RangingPacket));
        t4 = get_cycles();
        pongReceived = true;
    }
}

class MyScanCallbacks : public NimBLEScanCallbacks {
    void onResult(NimBLEAdvertisedDevice* advertisedDevice) {
        Serial.print("[BLE][SCAN] Device found: ");
        Serial.println(advertisedDevice->getName().c_str());
        if (advertisedDevice->haveServiceUUID()) {
            Serial.print("[BLE][SCAN] Service UUID: ");
            Serial.println(advertisedDevice->getServiceUUID().toString().c_str());
        }
        if (advertisedDevice->isAdvertisingService(SERVICE_UUID)) {
            Serial.println("[BLE][SCAN] DS-TWR service found, stopping scan.");
            pAdvertisedDevice = advertisedDevice;
            NimBLEDevice::getScan()->stop();
        }
    }
    void onScanEnd(NimBLEScanResults results) {
        Serial.print("[BLE][SCAN] Scan ended, devices found: ");
        Serial.println(results.getCount());
    }
};

void bluetooth_dstrwr_setup() {
    Serial.begin(115200);
    Serial.println("[BOOT] DS-TWR BLE (INITIATOR)");
    NimBLEDevice::init("");
}

void bluetooth_dstrwr_loop() {
    static bool connected = false;
    static uint32_t lastPing = 0;
    if (!connected) {
        NimBLEScan* pScan = NimBLEDevice::getScan();
        Serial.println("[BLE][SCAN] Starting scan (10s)...");
        pScan->setScanCallbacks(new MyScanCallbacks());
        pScan->setActiveScan(true);
        pScan->start(10, false); // Increase scan duration to 10 seconds
        NimBLEScanResults results = pScan->getResults();
        bool found = false;
        for (int i = 0; i < results.getCount(); ++i) {
            const NimBLEAdvertisedDevice* dev = results.getDevice(i);
            if (dev->isAdvertisingService(SERVICE_UUID)) {
                Serial.print("[BLE][SCAN] DS-TWR Device ");
                Serial.print(i);
                Serial.print(": Name: ");
                Serial.print(dev->getName().c_str());
                Serial.print(" | Service UUID: ");
                Serial.print(dev->getServiceUUID().toString().c_str());
                Serial.println("");
                pAdvertisedDevice = (NimBLEAdvertisedDevice*)dev; // Cast away const for connect
                found = true;
                break;
            }
        }
        if (found && pAdvertisedDevice) {
            Serial.println("[BLE][CONNECT] Trying to connect to server...");
            pClient = NimBLEDevice::createClient();
            if (pClient->connect(pAdvertisedDevice)) {
                Serial.println("[BLE][CONNECT] Connected!");
                NimBLERemoteService* pService = pClient->getService(SERVICE_UUID);
                if (pService) {
                    Serial.print("[BLE][CONNECT] Service found! UUID: ");
                    Serial.println(pService->getUUID().toString().c_str());
                    pTxCharacteristic = pService->getCharacteristic(CHAR_UUID_TX);
                    pRxCharacteristic = pService->getCharacteristic(CHAR_UUID_RX);
                    if (pTxCharacteristic) {
                        Serial.print("[BLE][CONNECT] TX Char found! UUID: ");
                        Serial.println(pTxCharacteristic->getUUID().toString().c_str());
                    } else {
                        Serial.println("[BLE][ERROR] TX Characteristic not found!");
                    }
                    if (pRxCharacteristic) {
                        Serial.print("[BLE][CONNECT] RX Char found! UUID: ");
                        Serial.println(pRxCharacteristic->getUUID().toString().c_str());
                    } else {
                        Serial.println("[BLE][ERROR] RX Characteristic not found!");
                    }
                    if (pTxCharacteristic && pRxCharacteristic) {
                        pTxCharacteristic->subscribe(true, notifyCallback);
                        connected = true;
                        Serial.println("[BLE] Connecté au serveur BLE");
                    } else {
                        Serial.println("[BLE][ERROR] Characteristics not found!");
                    }
                } else {
                    Serial.println("[BLE][ERROR] Service not found!");
                }
            } else {
                Serial.println("[BLE][ERROR] Connection failed!");
            }
        } else {
            Serial.println("[BLE][SCAN] No DS-TWR server found yet.");
        }
    } else {
        if (millis() - lastPing > 1000) {
            RangingPacket pkt = {0, 0, 0, 0};
            t1 = get_cycles();
            pkt.type = 0;
            pkt.t1 = t1;
            bool writeOk = false;
            if (pRxCharacteristic) {
                writeOk = pRxCharacteristic->writeValue((uint8_t*)&pkt, sizeof(pkt));
                Serial.print("[PING][BLE] Write to RX Char: ");
                Serial.println(writeOk ? "OK" : "FAILED");
            } else {
                Serial.println("[PING][BLE] RX Characteristic is NULL!");
            }
            lastPing = millis();
        }
        if (pongReceived) {
            uint64_t T1 = lastPong.t1;
            uint64_t T2 = lastPong.t2;
            uint64_t T3 = lastPong.t3;
            uint64_t T4 = t4;
            uint64_t delay = cycles_diff(T4, T1) - cycles_diff(T3, T2);
            double tof = delay * CYCLES_TO_SEC / 2.0;
            double distance = tof * SPEED_OF_LIGHT;
            Serial.print("[RESULT][BLE] T1="); Serial.print(T1);
            Serial.print(" T2="); Serial.print(T2);
            Serial.print(" T3="); Serial.print(T3);
            Serial.print(" T4="); Serial.print(T4);
            Serial.print(" | ToF="); Serial.print(tof * 1e9, 1); Serial.print(" ns");
            Serial.print(" | Distance="); Serial.print(distance, 3); Serial.println(" m");
            pongReceived = false;
        }
    }
}
