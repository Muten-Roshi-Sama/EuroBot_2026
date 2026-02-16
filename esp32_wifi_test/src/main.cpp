#include <Arduino.h>
#include <LEDController.h>
#include <WiFiProtocol.h>

// ========== CONFIGURATION ==========
const char* SSID = "felix123";           // À configurer avec votre WiFi SSID
const char* PASSWORD = "ecamwouw";       // À configurer avec votre WiFi Password
const uint16_t TCP_PORT = 5000;              // Port TCP d'écoute
const uint8_t LED_PIN = 16;                  // Pin GPIO pour la LED de test

// ========== INSTANCES GLOBALES ==========
LEDController ledController;
WiFiProtocol wifiProtocol(SSID, PASSWORD, TCP_PORT);

// ========== TIMERS ==========
uint32_t lastStatusTime = 0;
const uint32_t STATUS_INTERVAL = 10000;      // Afficher status tous les 10 secondes

// ========== PROTOTYPES ==========
void blinkError();
void printStatus();
void printHeader();
void scanAndPrintWiFiNetworks();

// Callback WiFi pour traiter les commandes custom
void handle_wifi_commands(const char* cmd, const JsonObject& params, DynamicJsonDocument& response) {
    if (strcmp(cmd, "HELLO") == 0) {
        response["status"] = "ok";
        response["message"] = "WiFi Test Module Ready";
        response["data"]["version"] = "1.0";
    } else {
        response["status"] = "error";
        response["message"] = "Commande inconnue";
    }
}

// ========== SETUP ==========
void setup() {
    // Initialiser Serial pour le debug
    Serial.begin(115200);
    delay(1000);
    
    // Afficher le header
    printHeader();
    
    // ===== ÉTAPE 1: Initialiser LEDController =====
    Serial.println("[Setup] Initialisation LEDController...");
    ledController.init_led(LED_PIN);
    ledController.stop_all_blinks();
    ledController.led_off(LED_PIN);
    Serial.printf("  ✓ LED pin %d\n", LED_PIN);
    
    // ===== ÉTAPE 2: Configurer WiFiProtocol =====
    Serial.println("[Setup] Configuration WiFi...");
    Serial.printf("  SSID: %s\n", SSID);
    Serial.println("  Scan réseau...");
    scanAndPrintWiFiNetworks();
    Serial.println("  Connexion...");
    
    if (!wifiProtocol.setup()) {
        Serial.println("\n  ✗ WiFi: Connexion échouée");
        Serial.printf("     SSID: %s\n", SSID);
        Serial.println("     Redémarrez l'ESP32 ou vérifiez les paramètres WiFi\n");
    }
    
    Serial.println();
    
    // ===== ÉTAPE 3: Lier les modules =====
    Serial.println("[Setup] Liaison modules...");
    wifiProtocol.set_led_controller(&ledController);
    wifiProtocol.on_command(handle_wifi_commands);
    Serial.println("  ✓ Init complétée\n");
    
    if (wifiProtocol.is_wifi_connected()) {
        String localIP = wifiProtocol.get_local_ip();
        Serial.println("╔════════════════════════════════════════╗");
        Serial.println("║         ✓ PRÊT POUR CONNEXION          ║");
        Serial.println("╠════════════════════════════════════════╣");
        Serial.printf("║ IP:   %s\n", localIP.c_str());
        Serial.printf("║ Port: %d\n", TCP_PORT);
        Serial.println("║ RSSI: Connecté                         ║");
        Serial.println("╚════════════════════════════════════════╝\n");
    }
    
    lastStatusTime = millis();
}

// ========== LOOP ==========
void loop() {
    wifiProtocol.update();
    ledController.update();
    
    if (millis() - lastStatusTime > STATUS_INTERVAL) {
        lastStatusTime = millis();
        printStatus();
    }
    
    delay(10);
}

// ========== FONCTIONS UTILITAIRES ==========

/**
 * Clignoter la LED rapidement en cas d'erreur WiFi
 */
void blinkError() {
    ledController.led_on(LED_PIN);
    delay(150);
    ledController.led_off(LED_PIN);
    delay(150);
}

/**
 * Afficher les statistiques du système
 */
void printStatus() {
    Serial.println("\n┌──────────── STATUS ────────────┐");
    
    if (wifiProtocol.is_wifi_connected()) {
        Serial.printf("│ WiFi........: ✓ Connecté\n");
        Serial.printf("│ IP...........: %s\n", wifiProtocol.get_local_ip().c_str());
        Serial.printf("│ RSSI.........: %d dBm\n", wifiProtocol.get_rssi());
    } else {
        Serial.printf("│ WiFi........: ✗ Déconnecté\n");
    }
    
    uint32_t uptimeSec = millis() / 1000;
    uint32_t hours = uptimeSec / 3600;
    uint32_t minutes = (uptimeSec % 3600) / 60;
    Serial.printf("│ Uptime.......: %02u:%02u\n", hours, minutes);
    
    Serial.println("└────────────────────────────────┘\n");
}

/**
 * Afficher le header au démarrage
 */
void printHeader() {
    Serial.println("\n╔════════════════════════════════════════╗");
    Serial.println("║  EuroBot 2026 - WiFi Test Module      ║");
    Serial.println("╚════════════════════════════════════════╝\n");
}

void scanAndPrintWiFiNetworks() {
    int numNetworks = WiFi.scanNetworks();
    
    if (numNetworks > 0) {
        Serial.printf("  📡 Réseaux détectés: %d\n\n", numNetworks);
        for (int i = 0; i < numNetworks && i < 10; i++) {
            String ssid = WiFi.SSID(i);
            int rssi = WiFi.RSSI(i);
            Serial.printf("     %d. %s (%d dBm)\n", i+1, ssid.c_str(), rssi);
            
            if (ssid == SSID) {
                Serial.println("        ⭐ C'est le nôtre!\n");
            }
        }
        Serial.println();
    } else {
        Serial.println("  ⚠️  Aucun réseau détecté!\n");
    }
}

