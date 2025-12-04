#include <esp_now.h>
#include <WiFi.h>

void onReceive(const uint8_t *mac, const uint8_t *data, int len) {
    if (len >= 4 && strncmp((const char*)data, "Ping", 4) == 0) {
        const char pong[] = "Pong";
        esp_now_send(mac, (const uint8_t*)pong, sizeof(pong));
        Serial.println("Ping reçu, Pong envoyé");
    }
}

void setup() {
    Serial.begin(115200);
    WiFi.mode(WIFI_STA);

    if (esp_now_init() != ESP_OK) {
        Serial.println("Erreur init ESP-NOW");
        return;
    }

    esp_now_register_recv_cb(onReceive);
    Serial.println("[BOOT] ESP32 Pong responder prêt");
}

void loop() {
    // Rien à faire ici, tout est dans le callback
}