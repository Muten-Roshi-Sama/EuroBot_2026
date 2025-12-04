#include "espnow_receiver.h"

namespace {
    // MAC de l'autre ESP (à adapter)
    uint8_t peerAddress[] = {0x24, 0x6F, 0x28, 0xAE, 0x12, 0x34};
    volatile bool pongReceived = false;
    volatile unsigned long pongTimestamp = 0;
    unsigned long pingTimestamp = 0;
    bool waitingPong = false;
    unsigned long lastPing = 0;

    void onReceive(const uint8_t *mac, const uint8_t *data, int len) {
        if (len >= 4 && strncmp((const char*)data, "Pong", 4) == 0) {
            pongTimestamp = micros();
            pongReceived = true;
        }
    }
}

namespace espnow_receiver {
    void begin() {
        WiFi.mode(WIFI_STA);
        if (esp_now_init() == ESP_OK) {
            esp_now_peer_info_t peerInfo = {};
            memcpy(peerInfo.peer_addr, peerAddress, 6);
            peerInfo.channel = 0;
            peerInfo.encrypt = false;
            esp_now_add_peer(&peerInfo);
            esp_now_register_recv_cb(onReceive);
            Serial.println("[espnow_receiver] ESP-NOW initialisé et peer ajouté");
        } else {
            Serial.println("[espnow_receiver] Erreur init ESP-NOW");
        }
    }

    void detection_loop() {
        Serial.println("loop");
        if (!waitingPong && (millis() - lastPing > 1000)) {
            const char msg[] = "Ping";
            Serial.println("[espnow_receiver] Tentative d'envoi de Ping...");
            esp_err_t result = esp_now_send(peerAddress, (uint8_t *)msg, sizeof(msg));
            if (result == ESP_OK) {
                Serial.println("Ping envoyé");
            } else {
                Serial.print("Erreur esp_now_send: ");
                Serial.println(result);
            }
            pingTimestamp = micros();
            waitingPong = true;
            lastPing = millis();
        }

        if (waitingPong && pongReceived) {
            unsigned long rtt = pongTimestamp - pingTimestamp; // en microsecondes
            float distance = (rtt / 2.0) * 0.000299792458; // vitesse lumière ~0.3 m/µs
            Serial.print("Pong reçu ! RTT: ");
            Serial.print(rtt);
            Serial.print(" us | Distance estimée: ");
            Serial.print(distance, 3);
            Serial.println(" m");
            pongReceived = false;
            waitingPong = false;
        }
    }
}