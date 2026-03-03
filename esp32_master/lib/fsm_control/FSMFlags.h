#pragma once

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

/**
 * FSMFlags - Commandes BLE pour controle à distance de la FSM
 * 
 * Contient 4 flags volatiles avec accès thread-safe via mutex:
 * - flag_ble_connected: BLE client connecté (bloque IDLE tant que false)
 * - flag_ready: Signal de préparation/démarrage
 * - flag_trigger_launch: Déclenchement du lancement (flanc descendant OU commande distante)
 * - flag_emergency_stop: Arrêt d'urgence (bouton physique OU commande distante)
 * 
 * Design: Simple, non-invasif, thread-safe
 * Les flags s'ajoutent aux sources existantes (IOExpander, boutons) dans la FSM
 */

class FSMFlags {
public:
    FSMFlags();
    
    // === SETTERS (pour BluetoothProtocol callbacks) ===
    void set_ble_connected(bool value);
    void set_ready(bool value);
    void set_trigger_launch(bool value);
    void set_emergency_stop(bool value);
    
    // === GETTERS (pour FSM logic) ===
    bool get_ble_connected() const;
    bool get_ready() const;
    bool get_trigger_launch() const;
    bool get_emergency_stop() const;
    
    // === RESET (pour nettoyer après consommation du flag) ===
    void clear_trigger_launch();
    void clear_emergency_stop();
    
private:
    volatile bool flag_ble_connected;
    volatile bool flag_ready;
    volatile bool flag_trigger_launch;
    volatile bool flag_emergency_stop;
    
    mutable SemaphoreHandle_t flags_mutex;
};
