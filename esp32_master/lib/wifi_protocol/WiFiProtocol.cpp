/*
 * WiFiProtocol - Implémentation
 */

#include "WiFiProtocol.h"

WiFiProtocol::WiFiProtocol(const char* ssid, const char* password, uint16_t port)
    : ssid(ssid), password(password), port(port), server(nullptr), 
      led_controller(nullptr), command_callback(nullptr), boot_time_ms(0) {
}

boolean WiFiProtocol::setup() {
    Serial.println("\n=== WiFiProtocol Setup ===");
    
    // Connexion WiFi
    Serial.printf("[WiFi] Connexion à %s...\n", ssid);
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
        delay(500);
        Serial.print(".");
        attempts++;
    }
    Serial.println();
    
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("[WiFi] ✗ Connexion échouée!");
        return false;
    }
    
    Serial.println("[WiFi] ✓ Connecté!");
    Serial.printf("[WiFi] IP: %s\n", WiFi.localIP().toString().c_str());
    Serial.printf("[WiFi] RSSI: %d dBm\n", WiFi.RSSI());
    
    // Démarrer serveur TCP
    server = new WiFiServer(port);
    server->begin();
    
    Serial.printf("[Server] Écoute sur port %d\n", port);
    Serial.println("=== WiFiProtocol Ready ===\n");
    
    boot_time_ms = millis();
    return true;
}

void WiFiProtocol::update() {
    // Accepter nouvelles connexions
    if (server->hasClient()) {
        if (!client || !client.connected()) {
            client = server->available();
            Serial.printf("[Client] Connecté: %s:%d\n", 
                         client.remoteIP().toString().c_str(), 
                         client.remotePort());
        }
    }
    
    // Traiter messages du client
    if (client && client.connected()) {
        while (client.available()) {
            char c = client.read();
            buffer += c;
            
            // Message complet (terminé par \n)
            if (c == '\n') {
                String message = buffer;
                buffer = "";
                
                // Traiter
                process_command(message);
            }
        }
    } else if (client && !client.connected()) {
        client.stop();
        Serial.println("[Client] Déconnecté");
    }
}

void WiFiProtocol::process_command(const String& json_str) {
    Serial.printf("[Command] Reçu: %s", json_str.c_str());
    
    DynamicJsonDocument doc(256);
    DeserializationError error = deserializeJson(doc, json_str);
    
    if (error) {
        Serial.printf("[Command] ✗ JSON Error: %s\n", error.c_str());
        return;
    }
    
    const char* cmd = doc["cmd"];
    JsonObject params = doc["params"].as<JsonObject>();
    
    Serial.printf("[Command] Commande: %s\n", cmd ? cmd : "N/A");
    
    DynamicJsonDocument response(512);
    
    // Commands built-in
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
    else if (command_callback) {
        command_callback(cmd, params, response);
    }
    else {
        response["status"] = "error";
        response["message"] = "Commande inconnue";
    }
    
    // Envoyer réponse
    String response_str;
    serializeJson(response, response_str);
    
    if (client && client.connected()) {
        client.println(response_str);
        Serial.printf("[Response] Envoyé: %s\n", response_str.c_str());
    }
}

void WiFiProtocol::broadcast(const char* message) {
    if (client && client.connected()) {
        client.println(message);
    }
}

boolean WiFiProtocol::is_wifi_connected() {
    return WiFi.status() == WL_CONNECTED;
}

String WiFiProtocol::get_local_ip() {
    return WiFi.localIP().toString();
}

int32_t WiFiProtocol::get_rssi() {
    return WiFi.RSSI();
}

void WiFiProtocol::on_command(OnCommandCallback callback) {
    command_callback = callback;
}

// === HANDLERS ===

void WiFiProtocol::handle_led_on(const JsonObject& params, DynamicJsonDocument& response) {
    uint8_t pin = params["pin"] | 16;
    
    if (led_controller) {
        led_controller->led_on(pin);
    }
    
    response["status"] = "ok";
    response["message"] = "LED allumée";
    response["data"]["pin"] = pin;
    response["data"]["led_state"] = true;
    
    Serial.printf("[LED] ON (pin %d)\n", pin);
}

void WiFiProtocol::handle_led_off(const JsonObject& params, DynamicJsonDocument& response) {
    uint8_t pin = params["pin"] | 16;
    
    if (led_controller) {
        led_controller->led_off(pin);
    }
    
    response["status"] = "ok";
    response["message"] = "LED éteinte";
    response["data"]["pin"] = pin;
    response["data"]["led_state"] = false;
    
    Serial.printf("[LED] OFF (pin %d)\n", pin);
}

void WiFiProtocol::handle_led_blink(const JsonObject& params, DynamicJsonDocument& response) {
    uint8_t pin = params["pin"] | 16;
    uint32_t interval = params["interval"] | 500;
    uint32_t cycles = params["cycles"] | 5;
    
    if (led_controller) {
        led_controller->blink(pin, interval, cycles);
    }
    
    response["status"] = "ok";
    response["message"] = "Clignotement démarré";
    response["data"]["pin"] = pin;
    response["data"]["interval"] = interval;
    response["data"]["cycles"] = cycles;
    
    Serial.printf("[LED] BLINK (pin %d, %dms x %d cycles)\n", pin, interval, cycles);
}

void WiFiProtocol::handle_get_status(DynamicJsonDocument& response) {
    uint32_t uptime = (millis() - boot_time_ms) / 1000;
    int32_t rssi = get_rssi();
    uint32_t free_heap = ESP.getFreeHeap();
    uint32_t chip_id = (uint32_t)ESP.getEfuseMac();
    
    response["status"] = "ok";
    response["message"] = "Status ESP32";
    response["data"]["uptime"] = uptime;
    response["data"]["uptime_str"] = String(uptime) + "s";
    response["data"]["rssi"] = rssi;
    response["data"]["free_heap"] = free_heap;
    response["data"]["chip_id"] = String(chip_id, HEX);
    response["data"]["ip"] = get_local_ip();
    
    Serial.printf("[Status] Uptime: %us, RSSI: %d, Heap: %u\n", uptime, rssi, free_heap);
}

void WiFiProtocol::handle_reset(DynamicJsonDocument& response) {
    response["status"] = "ok";
    response["message"] = "Redémarrage en cours...";
    
    // Envoyer la réponse avant de redémarrer
    String response_str;
    serializeJson(response, response_str);
    
    if (client && client.connected()) {
        client.println(response_str);
        client.stop();
    }
    
    Serial.println("[Reset] Redémarrage ESP32...\n");
    delay(500);
    
    ESP.restart();
}

void WiFiProtocol::send_response(const char* status, const char* message, JsonObject* data) {
    DynamicJsonDocument doc(512);
    doc["status"] = status;
    doc["message"] = message;
    
    if (data) {
        doc["data"] = *data;
    } else {
        doc["data"] = JsonObject();
    }
    
    String response_str;
    serializeJson(doc, response_str);
    
    if (client && client.connected()) {
        client.println(response_str);
    }
}
