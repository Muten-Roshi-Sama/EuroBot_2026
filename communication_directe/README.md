# Communication Directe ESP32-S ↔ PC via WiFi

Communication WiFi entre un ESP32-S et un PC avec protocole JSON pour contrôler une LED.

## Fonctionnalités

- Serveur WiFi TCP sur l'ESP32-S
- Communication via messages JSON
- Commande pour allumer/éteindre une LED
- Réception de confirmation "led allumée" en retour
- Client Python pour envoyer les commandes depuis le PC

## Configuration

### 1. ESP32-S (`esp32_wifi_server.cpp`)

**Modifiez les constantes WiFi dans le code** (lignes 40-41) :
```cpp
static constexpr const char *WIFI_SSID = "VOTRE_SSID";
static constexpr const char *WIFI_PASS = "VOTRE_MOT_DE_PASSE";
```

**Ajustez le pin de la LED si nécessaire** (ligne 48) :
```cpp
static constexpr gpio_num_t LED_PIN = GPIO_NUM_2;  // GPIO 2 pour ESP32-S2/S3
```

### 2. Client Python (`pc_client.py`)

**Modifiez l'IP de l'ESP32-S :**
```python
ESP32_IP = "192.168.1.100"  # Remplacez par l'IP affichée dans le moniteur série
```

## Installation et Upload

### Prérequis

- PlatformIO installé OU Extension ESP-IDF pour VS Code
- cJSON (inclus dans ESP-IDF)

### Upload sur ESP32-S

1. **Avec PlatformIO (recommandé) :**
   ```bash
   # Pour ESP32-S2
   pio run -e esp32s2_wifi -t upload
   
   # Pour ESP32-S3
   pio run -e esp32s3_wifi -t upload
   ```

2. **Ou avec ESP-IDF directement :**
   ```bash
   cd communication_directe
   idf.py set-target esp32s2  # ou esp32s3
   idf.py build
   idf.py flash monitor
   ```

3. **Ouvrez le moniteur série pour voir l'IP :**
   ```bash
   pio device monitor
   ```
   Notez l'adresse IP affichée (ex: `IP address: 192.168.1.100`)

### Configuration ESP-IDF dans VS Code (optionnel)

1. **Lancez le scan ESP-IDF dans VS Code :**
   - Ouvrez la palette de commandes (Ctrl+Shift+P)
   - Tapez "ESP-IDF: Configure ESP-IDF extension"
   - Suivez les instructions pour configurer l'extension
   - Lancez "ESP-IDF: Select port to use"
   - Lancez "ESP-IDF: Set Espressif device target" et sélectionnez votre ESP32-S2 ou ESP32-S3

## Utilisation

### Sur l'ESP32-S

1. Uploadez le code `esp32_wifi_server.ino`
2. Ouvrez le moniteur série (115200 baud)
3. Attendez la connexion WiFi et notez l'adresse IP affichée

### Sur le PC

1. **Installez les dépendances Python (si nécessaire) :**
   ```bash
   pip install pyserial  # Optionnel, pour le moniteur série
   ```

2. **Modifiez l'IP dans `pc_client.py`** avec l'IP affichée par l'ESP32

3. **Lancez le client :**
   ```bash
   python communication_directe/pc_client.py
   ```

4. **Résultat attendu :**
   - Connexion à l'ESP32
   - Envoi de la commande `{"command": "led_on"}`
   - Réception de la réponse `{"status": "success", "message": "led allumée", "led_state": true}`
   - La LED s'allume sur l'ESP32

## Format des Messages JSON

### Commandes (PC → ESP32)

**Allumer la LED :**
```json
{"command": "led_on"}
```

**Éteindre la LED :**
```json
{"command": "led_off"}
```

**Demander l'état de la LED :**
```json
{"command": "led_status"}
```

### Réponses (ESP32 → PC)

**Succès :**
```json
{
  "status": "success",
  "message": "led allumée",
  "led_state": true
}
```

**Erreur :**
```json
{
  "status": "error",
  "message": "Description de l'erreur"
}
```

## Dépannage

- **Connexion échouée** : Vérifiez que l'ESP32 et le PC sont sur le même réseau WiFi
- **Timeout** : Vérifiez que l'IP dans `pc_client.py` correspond à celle affichée par l'ESP32
- **LED ne s'allume pas** : Vérifiez que le pin LED_PIN correspond à votre carte (GPIO 2 pour la plupart des ESP32-S)
- **Erreur de parsing JSON** : Vérifiez que les messages JSON sont bien formatés (pas d'espaces superflus, guillemets corrects)

## Structure des Fichiers

```
communication_directe/
├── esp32_wifi_server.cpp  # Code ESP-IDF pour ESP32-S
├── pc_client.py           # Client Python pour PC
├── README.md              # Documentation complète
└── GUIDE_ETAPES.md        # Guide pas à pas
```
