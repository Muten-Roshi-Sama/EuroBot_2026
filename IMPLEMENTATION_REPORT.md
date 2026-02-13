# EuroBot 2026 - Implémentation des Fonctionnalités WiFi

## 📋 État d'avancement

### ✅ Complété - Python GUI

**Fichiers créés:** `protocole_wifi/python/`

- **protocol.py** ✓
  - Protocole JSON standardisé
  - Commandes: LED_ON, LED_OFF, LED_BLINK, GET_STATUS, RESET
  - Parsing JSON robuste avec gestion d'erreurs

- **wifi_client.py** ✓
  - Client TCP orienté connexion
  - Communication non-bloquante avec threading
  - Callbacks: on_message, on_connection, on_error
  - Buffer de réception pour messages multi-lignes

- **app_gui.py** ✓
  - Interface Tkinter professionnelle
  - Connexion/Déconnexion TCP
  - Boutons LED (ON/OFF/BLINK avec paramètres)
  - Console colorisée en temps réel
  - Demandes: Status, Redémarrage
  - Timestamps pour tous les événements

- **esp32_client.py** ✓
  - Serveur TCP simulé pour tester sans ESP32 réel
  - Implémente tous les commandes
  - Simule uptime, RSSI, heap, chip_id
  - Parfait pour déboguer l'interface GUI

### ✅ Complété - Modules ESP32

**Fichiers créés:** `esp32_master/lib/`

- **LEDController** ✓
  - `LEDController.h` + `LEDController.cpp`
  - Méthodes: init_led(), led_on(), led_off(), blink()
  - Clignotement non-bloquant via update() en loop
  - Support multiple LEDs simultanées
  - Gestion complète des cycles et intervalles

- **WiFiProtocol** ✓
  - `WiFiProtocol.h` + `WiFiProtocol.cpp`
  - Serveur TCP sur port (défaut: 5000)
  - Parsing JSON avec ArduinoJson
  - Handlers intégrés pour: LED_ON, LED_OFF, LED_BLINK, GET_STATUS, RESET
  - Callbacks extensibles pour commandes custom
  - Gestion d'erreurs JSON complète
  - Uptime, RSSI, heap memory tracking

## 🚀 Utilisation

### Mode 1: Test avec serveur simulé (Aucun ESP32 nécessaire)

```bash
cd protocole_wifi/python

# Terminal 1: Démarrer serveur simulé
python esp32_client.py

# Terminal 2: Lancer GUI
python app_gui.py
```

Puis:
1. IP: `127.0.0.1` ou `localhost`
2. Port: `5000`
3. Cliquer "Connecter"

### Mode 2: Avec ESP32 réel

1. Configurer `SSID` et `PASSWORD` dans `src/main.cpp` ou `src/main_wifi_demo.cpp`
2. Compiler/Upload avec PlatformIO:
   ```bash
   pio run -t upload
   pio device monitor
   ```
3. Noter l'IP affichée en Serial
4. Lancer Python GUI avec cette IP

## 📝 Architecture

```
┌─────────────────────────────────────────────────────────────┐
│               Python GUI (Tkinter)                          │
│  - Interface utilisateur                                    │
│  - Gestion des boutons et paramètres                        │
└──────────────────┬──────────────────────────────────────────┘
                   │
        ┌──────────▼──────────┐
        │ protocol.py         │
        │ - make_command()    │
        │ - parse_message()   │
        │ Format JSON         │
        └──────────┬──────────┘
                   │
        ┌──────────▼──────────────┐
        │ WiFiClient (TCP)        │
        │ - Threading             │
        │ - Non-bloquant          │
        │ - Callbacks             │
        └──────────┬──────────────┘
                   │ JSON+\n
        ┌──────────▼──────────────────┐
        │ WiFiProtocol (ESP32)        │
        │ - Serveur TCP               │
        │ - Parser JSON               │
        │ - Dispatch commands         │
        └──────────┬──────────────────┘
                   │
        ┌──────────▼──────────────┐
        │ LEDController           │
        │ - led_on/off            │
        │ - blink() non-bloquant  │
        │ - update() en loop      │
        └─────────────────────────┘
```

## 🔧 Configuration

### ESP32 WiFi

**Fichier:** `src/main.cpp` ou `src/main_wifi_demo.cpp`

```cpp
const char* SSID = "EuroBot_2026";      // Modifier
const char* PASSWORD = "robotics2026";   // Modifier
const uint16_t TCP_PORT = 5000;         // Optionnel
```

### Python GUI

**Fichier:** `app_gui.py`

Valeurs par défaut (modifiables dans l'interface):
- IP: `192.168.1.100`
- Port: `5000`

### platformio.ini

Les dépendances sont ajoutées:
```ini
lib_deps =
    ArduinoJson@6.21.2
    WiFi
```

## 📡 Protocole JSON

### Request (PC → ESP32)

```json
{"cmd": "LED_ON", "params": {"pin": 16}}
{"cmd": "LED_OFF", "params": {"pin": 16}}
{"cmd": "LED_BLINK", "params": {"pin": 16, "interval": 500, "cycles": 5}}
{"cmd": "GET_STATUS", "params": {}}
{"cmd": "RESET", "params": {}}
```

### Response (ESP32 → PC)

```json
{
  "status": "ok",
  "message": "LED allumée",
  "data": {
    "pin": 16,
    "led_state": true
  }
}
```

Status: `"ok"` ou `"error"`

## 🧪 Test Quick Start

### Minimal (Test immediately)

```bash
# Terminal 1
cd protocole_wifi/python
python esp32_client.py

# Terminal 2
cd protocole_wifi/python
python app_gui.py
```

### Avec IP réelle ESP32

1. Obtenir IP de l'ESP32 (Serial Monitor)
2. Remplacer `192.168.1.100` dans app_gui.py par l'IP réelle
3. Relancer app_gui.py

## 📚 Fichiers Clés

```
protocole_wifi/
├── python/
│   ├── protocol.py ................. Protocole JSON
│   ├── wifi_client.py .............. Client TCP
│   ├── app_gui.py .................. GUI Tkinter
│   ├── esp32_client.py ............. Serveur simulé
│   └── README.md ................... Docs Python
│
esp32_master/
├── lib/
│   ├── led_controller/
│   │   ├── LEDController.h
│   │   └── LEDController.cpp
│   └── wifi_protocol/
│       ├── WiFiProtocol.h
│       └── WiFiProtocol.cpp
│
├── src/
│   ├── main.cpp .................... FSM existante
│   └── main_wifi_demo.cpp .......... Demo WiFi
│
└── platformio.ini .................. Config
```

## 🎯 Fonctionnalités Implémentées

- [x] Connexion/Déconnexion TCP
- [x] Bouton Allumer LED
- [x] Bouton Éteindre LED
- [x] Bouton Clignoter LED (avec intervalle ms + cycles)
- [x] Demander status (uptime, RSSI, heap, chip_id)
- [x] Bouton Redémarrer ESP32
- [x] Console logs colorisée en temps réel
- [x] Protocole JSON standardisé
- [x] Parser JSON robuste
- [x] Serveur TCP ESP32
- [x] Clignotement non-bloquant
- [x] Timestamps dans console
- [x] Serveur simulé pour test
- [x] Gestion d'erreurs complet

## ⚠️ Notes

1. **ArduinoJson version**: Version 6.x utilisée (plus légère que 7.x)
2. **Threading**: Python thread pour réception, non-bloquant
3. **SSID/Password**: À configurer dans le code ESP32
4. **Blink non-bloquant**: N'utilise pas `delay()`, utilise `millis()`
5. **Buffer réception**: Gère les messages partiels (multilingne)

## 🔍 Débogage

### GUI affiche "Impossible de se connecter"

1. Vérifier serveur simulé `python esp32_client.py` tourne
2. Vérifier IP/Port corrects (127.0.0.1:5000 pour test local)
3. Console affiche le message d'erreur exact

### ESP32 perd connexion WiFi

1. Vérifier SSID/Password dans src/main.cpp
2. Serial Monitor montrera "WiFi ✓ Connecté!"
3. Ajouter WiFi.reconnect() si besoin (optionnel)

### LED ne clignote pas

1. Vérifier pin 16 disponible sur votre ESP32
2. Appeler `led_controller.update()` dans loop()
3. Console GUI doit afficher "← Reçu: [ok]"
