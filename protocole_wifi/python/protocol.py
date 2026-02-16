"""
Protocole JSON standardisé pour communication ESP32-PC
Commandes: LED_ON, LED_OFF, LED_BLINK, GET_STATUS, RESET
"""

import json
from typing import Dict, Any


def make_command(command: str, params: Dict[str, Any] = None) -> str:
    """
    Créer une commande JSON formatée
    
    Args:
        command: Nom de la commande (LED_ON, LED_OFF, LED_BLINK, GET_STATUS, RESET)
        params: Dictionnaire des paramètres (optionnel)
    
    Returns:
        String JSON à envoyer
    """
    msg = {
        "cmd": command,
        "params": params or {}
    }
    return json.dumps(msg) + "\n"


def make_led_on(pin: int) -> str:
    """Allumer une LED"""
    return make_command("LED_ON", {"pin": pin})


def make_led_off(pin: int) -> str:
    """Éteindre une LED"""
    return make_command("LED_OFF", {"pin": pin})


def make_led_blink(pin: int, interval_ms: int, cycles: int) -> str:
    """Clignoter une LED"""
    return make_command("LED_BLINK", {
        "pin": pin,
        "interval": interval_ms,
        "cycles": cycles
    })


def make_get_status() -> str:
    """Demander le status (uptime, RSSI, état LED)"""
    return make_command("GET_STATUS")


def make_reset() -> str:
    """Redémarrer l'ESP32"""
    return make_command("RESET")


def make_fsm_set_ready(value: bool) -> str:
    """Définir le flag ready de la FSM"""
    return make_command("FSM_SET_READY", {"value": value})


def make_fsm_trigger_launch(value: bool) -> str:
    """Déclencher le lancement (flag_trigger_launch)"""
    return make_command("FSM_TRIGGER_LAUNCH", {"value": value})


def make_fsm_emergency_stop(value: bool) -> str:
    """Déclencher l'arrêt d'urgence (flag_emergency_stop)"""
    return make_command("FSM_EMERGENCY_STOP", {"value": value})


def parse_message(data: str) -> Dict[str, Any]:
    """
    Parser un message JSON reçu
    
    Args:
        data: String JSON reçu
    
    Returns:
        Dict parsed ou None si erreur
    
    Format attendu:
    {
        "status": "ok" ou "error",
        "message": "Description",
        "data": {...}
    }
    """
    try:
        return json.loads(data.strip())
    except json.JSONDecodeError as e:
        return {
            "status": "error",
            "message": f"JSON Parse Error: {e}"
        }


def is_valid_response(data: Dict) -> bool:
    """Vérifier si une réponse est valide"""
    return isinstance(data, dict) and "status" in data and "message" in data
