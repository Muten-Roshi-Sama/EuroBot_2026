/*
 * BluetoothProtocol - Implémentation
 */

#include "bluetooth.h"

// UUID du service Nordic UART Service
#define SERVICE_UUID    "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define RX_CHAR_UUID    "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"  // client → ESP32 (write)
#define TX_CHAR_UUID    "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"  // ESP32 → client (notify)

// Instance globale pour les callbacks
BluetoothProtocol* g_bleProto = nullptr;

BluetoothProtocol::BluetoothProtocol(const char* deviceName)
    : deviceName(deviceName), bleServer(nullptr), txCharacteristic(nullptr),
      rxCharacteristic(nullptr), command_callback(nullptr), connection_callback(nullptr),
      ble_connected(false), boot_time_ms(0) {
    g_bleProto = this;
}

boolean BluetoothProtocol::setup() {
    Serial.println("\n=== BluetoothProtocol Setup ===");

    // Initialiser BLE
    BLEDevice::init(deviceName);
    Serial.printf("[BLE] Device name: %s\n", deviceName);

    // Créer le serveur BLE
    bleServer = BLEDevice::createServer();
    bleServer->setCallbacks(new MyBLEServerCallbacks(this));

    // Créer le service
    BLEService* pService = bleServer->createService(SERVICE_UUID);

    // TX : notify (ESP32 → client)
    txCharacteristic = pService->createCharacteristic(
        TX_CHAR_UUID,
        BLECharacteristic::PROPERTY_NOTIFY
    );
    txCharacteristic->addDescriptor(new BLE2902());

    // RX : write (client → ESP32)
    rxCharacteristic = pService->createCharacteristic(
        RX_CHAR_UUID,
        BLECharacteristic::PROPERTY_WRITE
    );
    rxCharacteristic->setCallbacks(new MyBLECharacteristicCallbacks(this));

    // Démarrer le service
    pService->start();

    // Démarrer la publicité
    BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);
    pAdvertising->setMinPreferred(0x12);
    BLEDevice::startAdvertising();

    Serial.println("[BLE] ✓ Setup complet, en attente de connexion...");
    Serial.println("=== BluetoothProtocol Ready ===\n");

    boot_time_ms = millis();
    return true;
}

void BluetoothProtocol::update() {
    // Les callbacks BLE gèrent tout de façon asynchrone
}

void BluetoothProtocol::on_command(OnBLECommandCallback callback) {
    command_callback = callback;
}

void BluetoothProtocol::on_connection(OnBLEConnectionCallback callback) {
    connection_callback = callback;
}

void BluetoothProtocol::notify(const char* message) {
    if (is_connected() && txCharacteristic) {
        txCharacteristic->setValue((uint8_t*)message, strlen(message));
        txCharacteristic->notify();
    }
}

boolean BluetoothProtocol::is_connected() {
    return ble_connected;
}

int BluetoothProtocol::get_connected_clients() {
    return ble_connected ? 1 : 0;
}

void BluetoothProtocol::process_command(const String& json_str) {
    Serial.printf("[BLE_Command] Reçu: %s", json_str.c_str());

    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, json_str);

    if (error) {
        Serial.printf("[BLE_Command] ✗ JSON Error: %s\n", error.c_str());
        return;
    }

    const char* cmd = doc["cmd"];
    JsonObject params = doc["params"].as<JsonObject>();

    Serial.printf("[BLE_Command] Commande: %s\n", cmd ? cmd : "N/A");

    JsonDocument response;

    // Commandes built-in
    if (strcmp(cmd, "LED_ON") == 0) {
        handle_led_on(params, response);
    }
    else if (strcmp(cmd, "LED_OFF") == 0) {
        handle_led_off(params, response);
    }
    else if (strcmp(cmd, "LED_BLINK") == 0) {
        handle_led_blink(params, response);
    }
    else if (strcmp(cmd, "GET_STATUS") == 0) {
        handle_get_status(response);
    }
    else if (strcmp(cmd, "RESET") == 0) {
        handle_reset(response);
    }
    // Callback custom (commandes FSM, etc.)
    else if (command_callback) {
        command_callback(cmd, params, response);
    }
    else {
        response["status"] = "error";
        response["message"] = "Commande inconnue";
    }

    // Envoyer la réponse
    String responseStr;
    serializeJson(response, responseStr);
    notify((responseStr + "\n").c_str());
    Serial.printf("[BLE_Response] %s\n", responseStr.c_str());
}

void BluetoothProtocol::send_response(const char* status, const char* message, JsonObject* data) {
    JsonDocument response;
    response["status"] = status;
    response["message"] = message;
    if (data) {
        response["data"] = *data;
    }
    String responseStr;
    serializeJson(response, responseStr);
    notify((responseStr + "\n").c_str());
}

void BluetoothProtocol::handle_led_on(const JsonObject& params, JsonDocument& response) {
    response["status"] = "ok";
    response["message"] = "LED ON";
    response["data"]["cmd"] = "LED_ON";
    Serial.println("[BLE_LED] ON");
}

void BluetoothProtocol::handle_led_off(const JsonObject& params, JsonDocument& response) {
    response["status"] = "ok";
    response["message"] = "LED OFF";
    response["data"]["cmd"] = "LED_OFF";
    Serial.println("[BLE_LED] OFF");
}

void BluetoothProtocol::handle_led_blink(const JsonObject& params, JsonDocument& response) {
    int times    = params["times"] | 1;
    int delay_ms = params["delay"] | 500;

    response["status"] = "ok";
    response["message"] = "LED BLINK";
    response["data"]["cmd"]   = "LED_BLINK";
    response["data"]["times"] = times;
    response["data"]["delay"] = delay_ms;

    Serial.printf("[BLE_LED] BLINK x%d, delay=%d\n", times, delay_ms);
}

void BluetoothProtocol::handle_get_status(JsonDocument& response) {
    uint32_t uptime = millis() - boot_time_ms;

    response["status"] = "ok";
    response["message"] = "Status";
    response["data"]["uptime_ms"]    = uptime;
    response["data"]["ble_connected"] = ble_connected;
    response["data"]["protocol"]     = "BLE";

    Serial.printf("[BLE_STATUS] Uptime: %u ms\n", uptime);
}

void BluetoothProtocol::handle_reset(JsonDocument& response) {
    response["status"] = "ok";
    response["message"] = "Redémarrage...";
    response["data"]["cmd"] = "RESET";

    Serial.println("[BLE_RESET] Redémarrage demandé");
    delay(1000);
    ESP.restart();
}

// ====== BLE Callbacks ======

void MyBLEServerCallbacks::onConnect(BLEServer* pServer) {
    Serial.println("[BLE] ✓ Client connecté");
    if (bleProto) {
        bleProto->set_connected(true);
        if (bleProto->connection_callback) bleProto->connection_callback(true);
    }
}

void MyBLEServerCallbacks::onDisconnect(BLEServer* pServer) {
    Serial.println("[BLE] ✗ Client déconnecté");
    if (bleProto) {
        bleProto->set_connected(false);
        if (bleProto->connection_callback) bleProto->connection_callback(false);
    }
    // Redémarrer l'annonce pour permettre une nouvelle connexion
    BLEDevice::startAdvertising();
}

void MyBLECharacteristicCallbacks::onWrite(BLECharacteristic* pCharacteristic) {
    std::string rxValue = pCharacteristic->getValue();

    if (rxValue.length() > 0) {
        Serial.print("[BLE_RX] ");
        for (size_t i = 0; i < rxValue.length(); i++) {
            Serial.print(rxValue[i]);
        }
        Serial.println();

        if (bleProto) {
            bleProto->process_command(String(rxValue.c_str()));
        }
    }
}
