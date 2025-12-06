#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

#define RXD2 20 // RX du C3 (à connecter au TX de l'Arduino/USB-UART)
#define TXD2 21 // TX du C3 (à connecter au RX de l'Arduino/USB-UART)

// Adresse MAC réelle de l'ESP32S
uint8_t peerS[] = {0xE0, 0x5A, 0x1B, 0xA2, 0x4C, 0xD0};

// --- ESP32C3 : Responder ---
// Ce code répond "Pong" à chaque "Ping" reçu via ESP-NOW, toujours en broadcast

void onReceive(const uint8_t *mac, const uint8_t *data, int len) {
    if (len >= 4 && strncmp((const char*)data, "Ping", 4) == 0) {
        const char pong[] = "Pong";
        esp_err_t pongResult = esp_now_send(peerS, (const uint8_t*)pong, sizeof(pong));
        Serial1.print("Ping reçu, tentative envoi Pong à ESP32S : ");
        for (int i = 0; i < 6; i++) {
            Serial1.printf("%02X", peerS[i]);
            if (i < 5) Serial1.print(":");
        }
        Serial1.print(" | Résultat envoi: ");
        Serial1.println(pongResult == ESP_OK ? "OK" : String(pongResult));
    }
}

void setup() {
    Serial1.begin(115200, SERIAL_8N1, RXD2, TXD2); // UART1 sur broches physiques
    Serial1.println("[BOOT] ESP32C3 Pong responder prêt (ESP-NOW)");
    Serial1.print("[MAC] ");
    Serial1.println(WiFi.macAddress());
    WiFi.mode(WIFI_STA); // Mode station obligatoire pour ESP-NOW
    esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
    wifi_second_chan_t second;
    uint8_t chan;
    esp_wifi_get_channel(&chan, &second);
    Serial1.print("[ESP32C3] Canal WiFi forcé sur ");
    Serial1.println(chan);

    if (esp_now_init() != ESP_OK) {
        Serial1.println("Erreur init ESP-NOW");
        return;
    }

    // Ajout du peer ESP32S
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, peerS, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    esp_err_t peerResult = esp_now_add_peer(&peerInfo);
    Serial1.print("Ajout peer ESP32S : ");
    Serial1.println(peerResult == ESP_OK ? "OK" : String(peerResult));

    esp_now_register_recv_cb(onReceive);
}

void loop() {
    // Rien à faire ici, tout est dans le callback
}