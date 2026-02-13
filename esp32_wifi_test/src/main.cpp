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

// ========== SETUP ==========
void setup() {
    // Initialiser Serial pour le debug
    Serial.begin(115200);
    delay(1000);
    
    // Afficher le header
    printHeader();
    
    // ===== ÉTAPE 1: Initialiser LEDController =====
    Serial.println("[Setup] ÉTAPE 1: Initialiser LEDController");
    ledController.init_led(LED_PIN);
    Serial.printf("  ✓ LED initialisée sur pin %d\n\n", LED_PIN);
    
    // ===== ÉTAPE 2: Configurer WiFiProtocol =====
    Serial.println("[Setup] ÉTAPE 2: Configurer WiFiProtocol");
    Serial.printf("  SSID: %s\n", SSID);
    Serial.printf("  Password: ••••••••••\n");
    Serial.printf("  TCP Port: %d\n", TCP_PORT);
    
    if (!wifiProtocol.setup()) {
        Serial.println("\n  ✗ ERREUR: Impossible de se connecter au WiFi!");
        Serial.println("\n  ⚠️  Vérifiez:");
        Serial.println("     - Le SSID dans le code est correct");
        Serial.println("     - Le password dans le code est correct");
        Serial.println("     - L'ESP32 a accès au WiFi");
        Serial.println("\n  Modification du SSID/Password:");
        Serial.println("     1. Éditer ce fichier (src/main.cpp)");
        Serial.println("     2. Changer SSID et PASSWORD");
        Serial.println("     3. Recompiler et uploader");
        
        // Loop infini d'erreur
        while (1) {
            blinkError();
            delay(1000);
        }
    }
    
    Serial.println();
    
    // ===== ÉTAPE 3: Lier les modules =====
    Serial.println("[Setup] ÉTAPE 3: Lier les modules");
    wifiProtocol.set_led_controller(&ledController);
    Serial.println("  ✓ LEDController associé à WiFiProtocol\n");
    
    // ===== SUMMARY =====
    Serial.println("╔══════════════════════════════════════════════════════╗");
    Serial.println("║             ✓ SETUP TERMINÉ AVEC SUCCÈS              ║");
    Serial.println("╚══════════════════════════════════════════════════════╝");
    Serial.printf("\n  IP locale: %s\n", wifiProtocol.get_local_ip().c_str());
    Serial.printf("  WiFi RSSI: %d dBm\n", wifiProtocol.get_rssi());
    Serial.printf("  Port TCP: %d\n\n", TCP_PORT);
    
    Serial.println("╔══════════════════════════════════════════════════════╗");
    Serial.println("║         EN ATTENTE DE CONNEXIONS TCP...              ║");
    Serial.println("╚══════════════════════════════════════════════════════╝\n");
    
    Serial.println("Accueil clients sur:");
    Serial.printf("  → tcp://%s:%d\n\n", wifiProtocol.get_local_ip().c_str(), TCP_PORT);
    
    lastStatusTime = millis();
}

// ========== LOOP ==========
void loop() {
    // Mettre à jour le protocole WiFi
    // - Accepte les connexions TCP
    // - Traite les messages JSON
    // - Exécute les commandes (LED_ON, LED_OFF, LED_BLINK, GET_STATUS, RESET)
    wifiProtocol.update();
    
    // Mettre à jour les LEDs
    // - Gère les clignotements non-bloquants
    // - À appeler régulièrement dans la loop
    ledController.update();
    
    // Afficher un status périodique
    if (millis() - lastStatusTime > STATUS_INTERVAL) {
        lastStatusTime = millis();
        printStatus();
    }
    
    // Petit délai pour éviter de saturer le CPU
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
    Serial.println("\n┌─────────── STATUS SYSTEM ──────────────────┐");
    
    // WiFi Status
    if (wifiProtocol.is_wifi_connected()) {
        Serial.printf("│ WiFi........: ✓ Connecté\n");
        Serial.printf("│ IP...........: %s\n", wifiProtocol.get_local_ip().c_str());
        Serial.printf("│ RSSI.........: %d dBm\n", wifiProtocol.get_rssi());
    } else {
        Serial.printf("│ WiFi........: ✗ Déconnecté\n");
    }
    
    // Memory
    uint32_t freeHeap = ESP.getFreeHeap();
    uint32_t totalHeap = ESP.getHeapSize();
    uint8_t heapPercent = (freeHeap * 100) / totalHeap;
    Serial.printf("│ Heap.........: %u / %u bytes (%d%%)\n", freeHeap, totalHeap, heapPercent);
    
    // Uptime
    uint32_t uptimeSec = millis() / 1000;
    uint32_t hours = uptimeSec / 3600;
    uint32_t minutes = (uptimeSec % 3600) / 60;
    uint32_t seconds = uptimeSec % 60;
    Serial.printf("│ Uptime.......: %02u:%02u:%02u\n", hours, minutes, seconds);
    
    // LED Status
    bool ledState = ledController.is_on(LED_PIN);
    Serial.printf("│ LED (pin %d).: %s\n", LED_PIN, ledState ? "ON ✓" : "OFF");
    
    Serial.println("└────────────────────────────────────────────┘\n");
}

/**
 * Afficher le header au démarrage
 */
void printHeader() {
    Serial.println("\n\n");
    Serial.println("╔══════════════════════════════════════════════════════╗");
    Serial.println("║                                                      ║");
    Serial.println("║      EuroBot 2026 - WiFi Control Test Module         ║");
    Serial.println("║                                                      ║");
    Serial.println("╚══════════════════════════════════════════════════════╝");
    Serial.printf("\nCompilation: %s %s\n", __DATE__, __TIME__);
    Serial.printf("Sketch size: %.1f KB / %u KB\n", 
                 ESP.getSketchSize() / 1024.0, 
                 (ESP.getFlashChipSize() - ESP.getSketchSize()) / 1024);
    Serial.printf("Free heap: %u bytes\n\n", ESP.getFreeHeap());
}

/*
 * ========== USAGE / UTILISATION ==========
 * 
 * Pour compiler et uploader ce code:
 *   $ cd esp32_wifi_test
 *   $ pio run -t upload
 *   $ pio device monitor
 * 
 * Pour tester depuis Python:
 *   $ cd ../protocole_wifi/python
 *   $ python app_gui.py
 *   
 * Entrer:
 *   IP: <IP affichée en Serial> (ex: 192.168.1.100)
 *   Port: 5000
 *   Cliquer "Connecter"
 * 
 * ========== COMMANDES DISPONIBLES ==========
 * 
 * LED_ON (params: pin)
 *   {"cmd": "LED_ON", "params": {"pin": 16}}
 * 
 * LED_OFF (params: pin)
 *   {"cmd": "LED_OFF", "params": {"pin": 16}}
 * 
 * LED_BLINK (params: pin, interval, cycles)
 *   {"cmd": "LED_BLINK", "params": {"pin": 16, "interval": 500, "cycles": 5}}
 * 
 * GET_STATUS
 *   {"cmd": "GET_STATUS", "params": {}}
 * 
 * RESET
 *   {"cmd": "RESET", "params": {}}
 */
