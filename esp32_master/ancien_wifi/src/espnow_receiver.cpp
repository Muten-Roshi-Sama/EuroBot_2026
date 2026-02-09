#include "espnow_receiver.h"

#include <WiFi.h>
#include <esp_wifi.h>


// Adresse MAC du peer (à adapter)
static uint8_t peerAddress[] = {0x58, 0x8C, 0x81, 0x94, 0x22, 0x1C};

// STRUCTURE DE PACKET
struct RangingPacket {
    uint8_t  type;  // 0 = PING, 1 = PONG
    uint64_t t1;
    uint64_t t2;
    uint64_t t3;
};

// UTILITAIRES HORODATAGE
static inline uint64_t get_cycles() {
    return (uint64_t)esp_cpu_get_ccount();
}
static constexpr double CYCLES_TO_SEC = 1.0 / 240000000.0; // 240 MHz
static constexpr double SPEED_OF_LIGHT = 299792458.0;

// GESTION WRAPAROUND (soustraction modulo 2^64)
uint64_t cycles_diff(uint64_t a, uint64_t b) {
    return (a >= b) ? (a - b) : (UINT64_MAX - b + a + 1);
}

#if ESPNOW_ROLE_INITIATOR
static volatile bool pongReceived = false;
static RangingPacket lastPong;
static uint64_t t1, t4;
#else
static volatile bool pingReceived = false;
static RangingPacket lastPing;
#endif

// CALLBACK RX
static void onReceive(const uint8_t *mac, const uint8_t *data, int len) {
    if (len != sizeof(RangingPacket)) return;
    RangingPacket pkt;
    memcpy(&pkt, data, sizeof(pkt));
    Serial.println("[DEBUG] RX callback called");
#if ESPNOW_ROLE_INITIATOR
    if (pkt.type == 1) { // PONG
        Serial.println("[DEBUG] PONG received");
        lastPong = pkt;
        t4 = get_cycles();
        pongReceived = true;
    }
#else
    if (pkt.type == 0) { // PING
        lastPing = pkt;
        pingReceived = true;
    }
#endif
}

namespace espnow_receiver {
    void begin() {
        Serial.println("[espnow_receiver] Début initialisation ESP-NOW DS-TWR");
        WiFi.mode(WIFI_STA);
        esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
        Serial.print("[MAC] "); Serial.println(WiFi.macAddress());
        if (esp_now_init() != ESP_OK) {
            Serial.println("[ERR] esp_now_init");
            while (1) delay(1000);
        }
        esp_now_peer_info_t peerInfo = {};
        memcpy(peerInfo.peer_addr, peerAddress, 6);
        peerInfo.channel = 0;
        peerInfo.encrypt = false;
        esp_now_add_peer(&peerInfo);
        esp_now_register_recv_cb(onReceive);
#if ESPNOW_ROLE_INITIATOR
        Serial.println("[ROLE] INITIATOR");
#else
        Serial.println("[ROLE] RESPONDER");
#endif
    }

    void detection_loop() {
#if ESPNOW_ROLE_INITIATOR
        static uint32_t lastPing = 0;
        static bool waitingPong = false;
        if (!waitingPong && (millis() - lastPing > 1000)) {
            // Envoi PING
            RangingPacket pkt = {0, 0, 0, 0};
            t1 = get_cycles();
            pkt.type = 0;
            pkt.t1 = t1;
            esp_now_send(peerAddress, (uint8_t*)&pkt, sizeof(pkt));
            Serial.println("[PING] Envoyé");
            lastPing = millis();
            waitingPong = true;
        }
        if (waitingPong && pongReceived) {
            // Calcul DS-TWR
            uint64_t T1 = lastPong.t1;
            uint64_t T2 = lastPong.t2;
            uint64_t T3 = lastPong.t3;
            uint64_t T4 = t4;
            uint64_t delay = cycles_diff(T4, T1) - cycles_diff(T3, T2);
            double tof = delay * CYCLES_TO_SEC / 2.0;
            double distance = tof * SPEED_OF_LIGHT;
            Serial.print("[RESULT] T1="); Serial.print(T1);
            Serial.print(" T2="); Serial.print(T2);
            Serial.print(" T3="); Serial.print(T3);
            Serial.print(" T4="); Serial.print(T4);
            Serial.print(" | ToF="); Serial.print(tof * 1e9, 1); Serial.print(" ns");
            Serial.print(" | Distance="); Serial.print(distance, 3); Serial.println(" m");
            pongReceived = false;
            waitingPong = false;
        }
#else
        static bool waitingPing = false;
        if (pingReceived) {
            // À la réception du PING
            uint64_t t2 = get_cycles();
            RangingPacket resp;
            resp.type = 1;
            resp.t1 = lastPing.t1;
            resp.t2 = t2;
            resp.t3 = 0;
            // Préparation de la réponse
            delayMicroseconds(50); // Optionnel : simuler un petit délai de traitement
            resp.t3 = get_cycles();
            esp_now_send(peerAddress, (uint8_t*)&resp, sizeof(resp));
            Serial.println("[PONG] Répondu");
            pingReceived = false;
        }
#endif
    }
}