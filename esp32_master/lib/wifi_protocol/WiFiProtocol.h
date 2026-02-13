/*
 * WiFiProtocol - Serveur TCP, parsing JSON, gestion des commandes
 */

#ifndef WIFI_PROTOCOL_H
#define WIFI_PROTOCOL_H

#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include "LEDController.h"

// Callback pour event externe (Serial streaming, etc)
typedef void (*OnCommandCallback)(const char* cmd, const JsonObject& params, DynamicJsonDocument& response);

class WiFiProtocol {
public:
    WiFiProtocol(const char* ssid, const char* password, uint16_t port = 5000);
    
    // Initialiser WiFi et serveur TCP
    boolean setup();
    
    // Mettre à jour (accepter connexions, traiter messages)
    // À appeler dans loop()
    void update();
    
    // Définir le callback pour traiter les commandes custom
    void on_command(OnCommandCallback callback);
    
    // Diffuser un message vers tous les clients
    void broadcast(const char* message);
    
    // Vérifier si on est connecté au WiFi
    boolean is_wifi_connected();
    
    // Obtenir l'IP locale
    String get_local_ip();
    
    // Obtenir le RSSI (force du signal WiFi)
    int32_t get_rssi();
    
    // Définir reference au LEDController
    void set_led_controller(LEDController* led_ctrl) { led_controller = led_ctrl; }
    
private:
    const char* ssid;
    const char* password;
    uint16_t port;
    
    WiFiServer* server;
    WiFiClient client;
    String buffer;
    
    LEDController* led_controller;
    OnCommandCallback command_callback;
    
    uint32_t boot_time_ms;
    
    // Traiter une commande JSON
    void process_command(const String& json_str);
    
    // Construire une réponse JSON
    void send_response(const char* status, const char* message, JsonObject* data);
    
    // Handlers pour les commandes built-in
    void handle_led_on(const JsonObject& params, DynamicJsonDocument& response);
    void handle_led_off(const JsonObject& params, DynamicJsonDocument& response);
    void handle_led_blink(const JsonObject& params, DynamicJsonDocument& response);
    void handle_get_status(DynamicJsonDocument& response);
    void handle_reset(DynamicJsonDocument& response);
};

#endif // WIFI_PROTOCOL_H
