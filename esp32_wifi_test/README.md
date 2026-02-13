# ESP32 WiFi Test - Dossier Externe

Dossier dédié aux tests WiFi sans affecter le code principal.

## Structure

```
esp32_wifi_test/
├── platformio.ini ........ Configuration PlatformIO
└── src/
    └── main.cpp ......... Main simple WiFi/LED test
```

## Caractéristiques

- ✓ Main simple et clean
- ✓ Appelle les libs externalisées: `../esp32_master/lib/LEDController` et `../esp32_master/lib/WiFiProtocol`
- ✓ N'impacte pas le main.cpp principal (FSM)
- ✓ Configuration WiFi facile à modifier
- ✓ Debug complet en Serial
- ✓ Test des commandes LED_ON, LED_OFF, LED_BLINK, GET_STATUS, RESET

## Usage

### Compilation et Upload

```bash
cd esp32_wifi_test
pio run -e esp32dev -t upload
pio device monitor
```

### Configuration WiFi

Modifier dans `src/main.cpp`:

```cpp
const char* SSID = "VotreSSD";        // ← Changer
const char* PASSWORD = "YourPassword"; // ← Changer
```

### Test depuis Python

```bash
cd ../protocole_wifi/python

# Terminal 1: Serveur simulé (optionnel pour test local)
python esp32_client.py

# Terminal 2: GUI
python app_gui.py
# IP: <IP affichée en Serial> (ex: 192.168.1.100)
# Port: 5000
# Connecter
```

## Lib Externe

Les libs sont appelées depuis `../esp32_master/lib/`:

- `LEDController` - Contrôle LED non-bloquant
- `WiFiProtocol` - Serveur TCP + parsing JSON

Via la directive PlatformIO:
```ini
lib_extra_dirs = 
    ../esp32_master/lib
```

## Serial Output Example

```
╔══════════════════════════════════════════════════════╗
║      EuroBot 2026 - WiFi Control Test Module         ║
╚══════════════════════════════════════════════════════╝

Compilation: Feb 13 2026 10:30:45
Sketch size: 45.2 KB / 1024 KB
Free heap: 156000 bytes

[Setup] ÉTAPE 1: Initialiser LEDController
  ✓ LED initialisée sur pin 16

[Setup] ÉTAPE 2: Configurer WiFiProtocol
  SSID: EuroBot_2026
  Password: ••••••••••
  TCP Port: 5000

[WiFi] Connexion à EuroBot_2026...
[WiFi] ✓ Connecté!
[WiFi] IP: 192.168.1.100
[WiFi] RSSI: -45 dBm
[Server] Écoute sur port 5000

[Setup] ÉTAPE 3: Lier les modules
  ✓ LEDController associé à WiFiProtocol

╔══════════════════════════════════════════════════════╗
║             ✓ SETUP TERMINÉ AVEC SUCCÈS              ║
╚══════════════════════════════════════════════════════╝

  IP locale: 192.168.1.100
  WiFi RSSI: -45 dBm
  Port TCP: 5000

╔══════════════════════════════════════════════════════╗
║         EN ATTENTE DE CONNEXIONS TCP...              ║
╚══════════════════════════════════════════════════════╝

Accueil clients sur:
  → tcp://192.168.1.100:5000
```

## Dépannage

### Erreur: "Impossible de se connecter au WiFi"

1. Vérifier SSID/Password dans `src/main.cpp`
2. Vérifier que l'ESP32 est connecté au WiFi
3. Vérifier la portée du WiFi

### Serial affiche rien

1. Vérifier la vitesse: 115200 baud
2. Vérifier le port USB (COM3, COM4, etc.)
3. Les drivers CH340/CP2102 sont installés

### LED ne répond pas

1. Vérifier le pin GPIO (16 par défaut)
2. Vérifier que `ledController.update()` est appelé dans `loop()`
3. Vérifier la commande JSON envoyée

## Notes

- Ne modifiez PAS les libs dans `esp32_master/lib/`
- Pour les modifications des libs, éditez les fichiers là-bas
- Ce dossier est un espace de test isolé
