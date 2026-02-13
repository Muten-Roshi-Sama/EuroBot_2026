# Protocole WiFi ESP32 - Python

Communication TCP avec protocole JSON entre ESP32 et PC.

## Fichiers

- **protocol.py** - Protocole JSON standardisé (commandes et parsing)
- **wifi_client.py** - Client TCP pour communication avec l'ESP32
- **app_gui.py** - Interface GUI Tkinter pour contrôler l'ESP32
- **esp32_client.py** - Serveur simulé pour tester sans ESP32 réel

## Installation

```bash
# Pas de dépendances externes pour les fichiers Python purs
# Interface GUI utilise le tkinter standard (inclus dans Python)

# Optionnel: créer virtualenv
python -m venv venv
source venv/Scripts/activate  # Windows: venv\Scripts\activate
```

## Utilisation

### 1. Tester avec le serveur simulé

Terminal 1 - Lancer le serveur ESP32 simulé:
```bash
python esp32_client.py
# Écoute sur 0.0.0.0:5000
```

Terminal 2 - Lancer l'interface GUI:
```bash
python app_gui.py
# Entrer IP: 127.0.0.1 ou localhost
# Port: 5000
# Cliquer "Connecter"
```

### 2. Utiliser avec un vrai ESP32

1. L'ESP32 doit avoir le firmware qui:
   - Configure WiFi
   - Démarre serveur TCP sur port 5000
   - Parse les messages JSON
   - Implémente les commandes

2. Lancer l'interface GUI:
```bash
python app_gui.py
```

3. Entrer l'IP de l'ESP32 et le port

## Protocole JSON

### Commandes envoyées (PC → ESP32)

```json
{"cmd": "LED_ON", "params": {"pin": 16}}
{"cmd": "LED_OFF", "params": {"pin": 16}}
{"cmd": "LED_BLINK", "params": {"pin": 16, "interval": 500, "cycles": 5}}
{"cmd": "GET_STATUS", "params": {}}
{"cmd": "RESET", "params": {}}
```

### Réponses (ESP32 → PC)

```json
{
  "status": "ok",
  "message": "Description du résultat",
  "data": {
    "led_state": true,
    "uptime": 3600,
    "rssi": -45,
    "free_heap": 150000
  }
}
```

## Commandes disponibles

| Commande | Params | Description |
|----------|--------|-------------|
| LED_ON | pin | Allumer la LED |
| LED_OFF | pin | Éteindre la LED |
| LED_BLINK | pin, interval, cycles | Clignoter la LED |
| GET_STATUS | - | Demander status (uptime, RSSI, etc) |
| RESET | - | Redémarrer l'ESP32 |

## Console GUI

La console affiche:
- 🔗 Événements de connexion
- → Commandes envoyées
- ← Réponses reçues
- ⚠️ Erreurs

Chaque ligne est colorisée selon le type:
- **Bleu** (info): Événements normaux
- **Vert** (success): Succès des commandes
- **Rouge** (error): Erreurs
- **Orange** (warning): Avertissements

## Architecture

```
┌─────────────────┐
│   GUI (Tkinter) │
└────────┬────────┘
         │ make_*() commands
┌────────▼──────────────┐
│ protocol.py           │
│ - make_led_on/off     │
│ - parse_message()     │
└────────┬──────────────┘
         │ JSON
┌────────▼──────────────────┐
│ WiFiClient (TCP socket)   │
│ - connect/disconnect      │
│ - send/receive            │
│ - threading               │
└────────┬──────────────────┘
         │ TCP
┌────────▼──────────────┐
│  ESP32 (WiFi Server)  │
│  + Servo/LED control  │
└───────────────────────┘
```

## Notes

- Tous les messages JSON doivent se terminer par `\n`
- Le client utilise un thread de réception pour ne pas bloquer l'UI
- Reconnexion automatique possible (à ajouter si needed)
- Timeouts configurables (défaut: 5 secondes)

## Dépannage

Si le GUI affiche "Impossible de se connecter":
1. Vérifier que l'ESP32 est allumé et connecté au WiFi
2. Vérifier l'IP avec `ping 192.168.x.x`
3. Vérifier le port (défaut ESP32: 5000)
4. Regarder les logs de l'ESP32 en Serial Monitor
