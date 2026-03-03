/*
 * BluetoothProtocol - Gestion BLE pour ESP32
 * Gère les mêmes commandes que WiFiProtocol (LED, FSM, STATUS)
 *
 * Nordic UART Service UUIDs:
 *   SERVICE_UUID : 6E400001-B5A3-F393-E0A9-E50E24DCCA9E
 *   RX_CHAR_UUID : 6E400002-B5A3-F393-E0A9-E50E24DCCA9E  (client → ESP32, write)
 *   TX_CHAR_UUID : 6E400003-B5A3-F393-E0A9-E50E24DCCA9E  (ESP32 → client, notify)
 */

#ifndef BLUETOOTH_PROTOCOL_H
#define BLUETOOTH_PROTOCOL_H

#include <Arduino.h>
#include <ArduinoJson.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// Callback pour event externe (commandes)
typedef void (*OnBLECommandCallback)(const char* cmd, const JsonObject& params, JsonDocument& response);

// Callback pour connexion/déconnexion BLE
typedef void (*OnBLEConnectionCallback)(bool connected);

class BluetoothProtocol {
public:
    BluetoothProtocol(const char* deviceName = "EuroBot");

    // Initialiser BLE et serveur
    boolean setup();

    // Mettre à jour (traiter messages reçus)
    void update();

    // Définir le callback pour traiter les commandes custom
    void on_command(OnBLECommandCallback callback);

    // Définir le callback pour événements connexion/déconnexion
    void on_connection(OnBLEConnectionCallback callback);

    // Envoyer une notification au client
    void notify(const char* message);

    // Vérifier si un client est connecté
    boolean is_connected();

    // Obtenir le nombre de clients connectés
    int get_connected_clients();

    // Setter pour l'état de connexion (utilisé par les callbacks)
    void set_connected(bool connected) { ble_connected = connected; }

    // Friend classes pour accéder aux méthodes privées
    friend class MyBLEServerCallbacks;
    friend class MyBLECharacteristicCallbacks;

private:
    const char* deviceName;
    BLEServer* bleServer;
    BLECharacteristic* txCharacteristic;
    BLECharacteristic* rxCharacteristic;

    String buffer;
    OnBLECommandCallback command_callback;
    OnBLEConnectionCallback connection_callback;
    bool ble_connected;

    uint32_t boot_time_ms;

    // Traiter une commande JSON reçue
    void process_command(const String& json_str);

    // Construire une réponse JSON et l'envoyer via notify
    void send_response(const char* status, const char* message, JsonObject* data = nullptr);

    // Handlers pour les commandes built-in
    void handle_led_on(const JsonObject& params, JsonDocument& response);
    void handle_led_off(const JsonObject& params, JsonDocument& response);
    void handle_led_blink(const JsonObject& params, JsonDocument& response);
    void handle_get_status(JsonDocument& response);
    void handle_reset(JsonDocument& response);
};

// Callback événements serveur BLE
class MyBLEServerCallbacks : public BLEServerCallbacks {
public:
    MyBLEServerCallbacks(BluetoothProtocol* bleProto) : bleProto(bleProto) {}
    void onConnect(BLEServer* pServer) override;
    void onDisconnect(BLEServer* pServer) override;

private:
    BluetoothProtocol* bleProto;
};

// Callback écriture caractéristique RX
class MyBLECharacteristicCallbacks : public BLECharacteristicCallbacks {
public:
    MyBLECharacteristicCallbacks(BluetoothProtocol* bleProto) : bleProto(bleProto) {}
    void onWrite(BLECharacteristic* pCharacteristic) override;

private:
    BluetoothProtocol* bleProto;
};

#endif // BLUETOOTH_PROTOCOL_H
