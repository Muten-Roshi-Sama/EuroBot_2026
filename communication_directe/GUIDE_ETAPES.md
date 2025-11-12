# Guide d'Étapes - Communication ESP32-S ↔ PC via WiFi

## 📋 Vue d'ensemble

Ce guide vous explique comment configurer et utiliser la communication WiFi entre votre ESP32-S et votre PC pour contrôler une LED via des messages JSON.

## 🎯 Prérequis

- ESP32-S2 ou ESP32-S3
- Câble USB pour programmer l'ESP32
- PC avec Python 3 installé
- Extension ESP-IDF pour VS Code (recommandé) OU PlatformIO

---

## 📝 ÉTAPE 1 : Configuration du code ESP32

1. **Ouvrez le fichier** `communication_directe/esp32_wifi_server.cpp`

2. **Modifiez les identifiants WiFi** (lignes 40-41) :
   ```cpp
   static constexpr const char *WIFI_SSID = "VOTRE_SSID";
   static constexpr const char *WIFI_PASS = "VOTRE_MOT_DE_PASSE";
   ```

3. **Vérifiez le pin LED** (ligne 48) :
   ```cpp
   static constexpr gpio_num_t LED_PIN = GPIO_NUM_2;  // Ajustez si nécessaire
   ```

---

## 🔧 ÉTAPE 2 : Configuration de l'extension ESP-IDF

1. **Ouvrez VS Code** et la palette de commandes :
   - `Ctrl+Shift+P` (Windows/Linux) ou `Cmd+Shift+P` (Mac)

2. **Configurez ESP-IDF** :
   - Tapez : `ESP-IDF: Configure ESP-IDF extension`
   - Suivez l'assistant de configuration
   - Sélectionnez la version ESP-IDF (recommandé : v5.1 ou v5.2)

3. **Sélectionnez le port série** :
   - Tapez : `ESP-IDF: Select port to use`
   - Choisissez le port COM de votre ESP32

4. **Sélectionnez la cible** :
   - Tapez : `ESP-IDF: Set Espressif device target`
   - Choisissez `esp32s2` ou `esp32s3` selon votre carte

5. **Lancez le scan** :
   - Tapez : `ESP-IDF: SDK Configuration editor`
   - Ou simplement ouvrez le fichier `esp32_wifi_server.cpp` - l'extension devrait détecter automatiquement le projet

---

## 📤 ÉTAPE 3 : Compilation et Upload

### Avec PlatformIO (recommandé)

```bash
# Pour ESP32-S2
pio run -e esp32s2_wifi -t upload

# Pour ESP32-S3
pio run -e esp32s3_wifi -t upload
```

### Avec ESP-IDF directement

```bash
cd communication_directe
idf.py set-target esp32s2  # ou esp32s3
idf.py build
idf.py flash
```

---

## 📡 ÉTAPE 4 : Récupération de l'adresse IP

1. **Ouvrez le moniteur série** :
   ```bash
   pio device monitor
   ```
   Ou dans VS Code : `ESP-IDF: Monitor your device`

2. **Attendez la connexion WiFi** - Vous devriez voir :
   ```
   Wi-Fi connecté! IP: 192.168.1.100
   Serveur TCP en écoute sur le port 3333
   ```

3. **Notez l'adresse IP** affichée (ex: `192.168.1.100`)

---

## 💻 ÉTAPE 5 : Configuration du client Python

1. **Ouvrez le fichier** `communication_directe/pc_client.py`

2. **Modifiez l'adresse IP** (ligne 14) :
   ```python
   ESP32_IP = "192.168.1.100"  # Remplacez par l'IP de votre ESP32
   ```

---

## 🚀 ÉTAPE 6 : Test de la communication

1. **Lancez le client Python** :
   ```bash
   python communication_directe/pc_client.py
   ```

2. **Résultat attendu** :
   ```
   ==================================================
   Client Python - Communication ESP32-S
   ==================================================
   Connexion à l'ESP32 à l'adresse 192.168.1.100:3333...
   
   ✓ Connecté à l'ESP32 à 192.168.1.100:3333
   
   --- Envoi de la commande 'led_on' ---
   → Commande envoyée: {"command": "led_on"}
   ← Réponse reçue: {"status":"success","message":"led allumée","led_state":true}
   
   --- Réponse de l'ESP32 ---
   Status: success
   Message: led allumée
   État LED: Allumée
   
   ✓ Succès! La LED a été allumée et la confirmation a été reçue.
   ```

3. **Vérifiez** que la LED s'est allumée sur votre ESP32 !

---

## 🔍 Dépannage

### ❌ Connexion WiFi échouée
- Vérifiez que le SSID et le mot de passe sont corrects
- Vérifiez que votre réseau WiFi est en 2.4 GHz (ESP32-S ne supporte pas le 5 GHz)
- Vérifiez la distance entre l'ESP32 et le routeur

### ❌ Timeout lors de la connexion depuis le PC
- Vérifiez que l'IP dans `pc_client.py` correspond à celle affichée par l'ESP32
- Vérifiez que le PC et l'ESP32 sont sur le même réseau WiFi
- Vérifiez que le pare-feu Windows n'bloque pas le port 3333

### ❌ LED ne s'allume pas
- Vérifiez que le pin LED_PIN correspond à votre carte
- Pour ESP32-S2 : généralement GPIO 2
- Pour ESP32-S3 : généralement GPIO 2 ou GPIO 48
- Consultez la documentation de votre carte

### ❌ Erreur de compilation ESP-IDF
- Vérifiez que l'extension ESP-IDF est bien configurée
- Relancez le scan ESP-IDF dans VS Code
- Vérifiez que vous avez sélectionné la bonne cible (esp32s2 ou esp32s3)

---

## 📚 Commandes JSON disponibles

### Allumer la LED
```json
{"command": "led_on"}
```

### Éteindre la LED
```json
{"command": "led_off"}
```

### Demander l'état de la LED
```json
{"command": "led_status"}
```

---

## 📁 Structure des fichiers

```
communication_directe/
├── esp32_wifi_server.cpp    # Code ESP-IDF
├── pc_client.py             # Client Python
├── README.md                # Documentation complète
└── GUIDE_ETAPES.md          # Ce fichier
```

---

## ✅ Checklist rapide

- [ ] Identifiants WiFi configurés dans le code ESP32
- [ ] Pin LED vérifié selon votre carte
- [ ] Extension ESP-IDF configurée (si utilisation ESP-IDF)
- [ ] Code compilé et uploadé sur l'ESP32
- [ ] Adresse IP notée depuis le moniteur série
- [ ] IP configurée dans `pc_client.py`
- [ ] Client Python lancé et test réussi

---

**Bon développement ! 🎉**

