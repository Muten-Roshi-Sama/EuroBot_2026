#include "espnow_receiver.h"

#include <WiFi.h>
#include <esp_wifi.h>

namespace {
    // Adresse MAC réelle du C3 (à garder)
    uint8_t peerAddress[] = {0x58, 0x8C, 0x81, 0x94, 0x22, 0x1C};
    volatile bool pongReceived = false;
    volatile unsigned long pongTimestamp = 0;
    unsigned long pingTimestamp = 0;
    bool waitingPong = false;
    unsigned long lastPing = 0;

    // Callback natif ESP-IDF pour RSSI précis
    void onReceive(const uint8_t *mac, const uint8_t *data, int len, void *rx_ctrl) {
        Serial.print("[ESP-NOW RX] Message reçu : ");
        for (int i = 0; i < len; i++) Serial.print((char)data[i]);
        Serial.print(" | len="); Serial.print(len);
        int rssi = 0;
        if (rx_ctrl != nullptr) {
            rssi = ((wifi_pkt_rx_ctrl_t*)rx_ctrl)->rssi;
            Serial.print(" | RSSI précis: ");
            Serial.print(rssi);
            Serial.print(" dBm");
        }
        Serial.println();
        if (len >= 4 && strncmp((const char*)data, "Pong", 4) == 0) {
            pongTimestamp = micros();
            pongReceived = true;
        }
    }
}

namespace espnow_receiver {
    void begin() {
        Serial.println("[espnow_receiver] Début initialisation ESP-NOW");
        WiFi.mode(WIFI_STA);
        Serial.println("[espnow_receiver] WiFi.mode(WIFI_STA) OK");
        esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
        wifi_second_chan_t second;
        uint8_t chan;
        esp_wifi_get_channel(&chan, &second);
        Serial.print("[espnow_receiver] Canal WiFi forcé sur ");
        Serial.println(chan);
        Serial.print("[MAC] ");
        Serial.println(WiFi.macAddress());
        esp_err_t initResult = esp_now_init();
        if (initResult == ESP_OK) {
            Serial.println("[espnow_receiver] esp_now_init OK");
            esp_now_peer_info_t peerInfo = {};
            memcpy(peerInfo.peer_addr, peerAddress, 6);
            peerInfo.channel = 0;
            peerInfo.encrypt = false;
            esp_err_t peerResult = esp_now_add_peer(&peerInfo);
            if (peerResult == ESP_OK) {
                Serial.print("[espnow_receiver] Peer C3 ajouté OK : ");
                for (int i = 0; i < 6; i++) {
                    Serial.printf("%02X", peerAddress[i]);
                    if (i < 5) Serial.print(":");
                }
                Serial.println();
            } else {
                Serial.print("[espnow_receiver] Erreur ajout peer: ");
                Serial.println(peerResult);
            }
            // Enregistrement du callback natif avec RSSI
            esp_now_register_recv_cb((esp_now_recv_cb_t)onReceive);
            Serial.println("[espnow_receiver] Callback enregistré");
        } else {
            Serial.print("[espnow_receiver] Erreur init ESP-NOW: ");
            Serial.println(initResult);
        }
    }

    void detection_loop() {
        if (!waitingPong && (millis() - lastPing > 1000)) {
            const char msg[] = "Ping";
            Serial.println("Ping envoyé");
            esp_err_t result = esp_now_send(peerAddress, (uint8_t *)msg, sizeof(msg));
            if (result != ESP_OK) {
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
            // Correction d'étalonnage (modifiable)
            static float facteur_etalon = 5.0 / 0.7; // À ajuster selon ton étalonnage
            float distance_corrigee = distance * facteur_etalon;
            Serial.print("Pong reçu ! RTT: ");
            Serial.print(rtt);
            Serial.print(" us | Distance brute: ");
            Serial.print(distance, 3);
            Serial.print(" m | Distance corrigée: ");
            Serial.print(distance_corrigee, 3);
            Serial.println(" m");
            pongReceived = false;
            waitingPong = false;
        }
    }
}