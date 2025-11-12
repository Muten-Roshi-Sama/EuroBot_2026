#!/usr/bin/env python3
"""
Client Python pour communiquer avec l'ESP32-S via WiFi
Envoie des commandes JSON pour contrôler la LED et reçoit des réponses JSON
"""

import socket
import json
import sys
import time

# ========== CONFIGURATION ==========
# Modifiez l'IP de l'ESP32-S ici (visible dans le moniteur série)
ESP32_IP = "192.168.1.100"  # Remplacez par l'IP affichée par l'ESP32
TCP_PORT = 3333

# ========== FONCTIONS ==========

def send_command(sock, command):
    """
    Envoie une commande JSON à l'ESP32 et attend la réponse
    
    Args:
        sock: Socket TCP connecté
        command: Dictionnaire Python représentant la commande JSON
        
    Returns:
        Dictionnaire Python avec la réponse JSON, ou None en cas d'erreur
    """
    try:
        # Convertir la commande en JSON
        json_command = json.dumps(command) + "\n"
        
        # Envoyer la commande
        sock.sendall(json_command.encode('utf-8'))
        print(f"→ Commande envoyée: {json_command.strip()}")
        
        # Attendre la réponse (avec timeout)
        sock.settimeout(2.0)
        response_data = sock.recv(1024)
        
        if response_data:
            response_str = response_data.decode('utf-8').strip()
            print(f"← Réponse reçue: {response_str}")
            
            # Parser la réponse JSON
            try:
                response_json = json.loads(response_str)
                return response_json
            except json.JSONDecodeError as e:
                print(f"✗ Erreur de parsing JSON: {e}")
                return None
        else:
            print("✗ Aucune réponse reçue")
            return None
            
    except socket.timeout:
        print("✗ Timeout: Aucune réponse reçue dans les 2 secondes")
        return None
    except Exception as e:
        print(f"✗ Erreur lors de l'envoi/réception: {e}")
        return None

def connect_to_esp32():
    """
    Établit une connexion TCP avec l'ESP32
    
    Returns:
        Socket connecté, ou None en cas d'erreur
    """
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(5.0)
        sock.connect((ESP32_IP, TCP_PORT))
        print(f"✓ Connecté à l'ESP32 à {ESP32_IP}:{TCP_PORT}")
        return sock
    except socket.timeout:
        print(f"✗ Timeout: Impossible de se connecter à {ESP32_IP}:{TCP_PORT}")
        print("Vérifiez que l'ESP32 est allumé et connecté au WiFi")
        return None
    except ConnectionRefusedError:
        print(f"✗ Connexion refusée")
        print(f"Vérifiez que l'ESP32 écoute bien sur le port {TCP_PORT}")
        return None
    except Exception as e:
        print(f"✗ Erreur de connexion: {e}")
        return None

def main():
    """
    Fonction principale: se connecte à l'ESP32 et envoie la commande pour allumer la LED
    """
    print("=" * 50)
    print("Client Python - Communication ESP32-S")
    print("=" * 50)
    print(f"Connexion à l'ESP32 à l'adresse {ESP32_IP}:{TCP_PORT}...\n")
    
    # Connexion à l'ESP32
    sock = connect_to_esp32()
    if not sock:
        return
    
    try:
        # Attendre un peu pour que la connexion soit stable
        time.sleep(0.5)
        
        # Envoyer la commande pour allumer la LED
        print("\n--- Envoi de la commande 'led_on' ---")
        command = {
            "command": "led_on"
        }
        
        response = send_command(sock, command)
        
        if response:
            # Afficher la réponse de manière lisible
            print("\n--- Réponse de l'ESP32 ---")
            print(f"Status: {response.get('status', 'N/A')}")
            print(f"Message: {response.get('message', 'N/A')}")
            if 'led_state' in response:
                print(f"État LED: {'Allumée' if response['led_state'] else 'Éteinte'}")
            
            # Vérifier si la LED a bien été allumée
            if response.get('status') == 'success' and response.get('message') == 'led allumée':
                print("\n✓ Succès! La LED a été allumée et la confirmation a été reçue.")
            else:
                print("\n⚠ Attention: La réponse n'indique pas un succès complet")
        else:
            print("\n✗ Échec: Aucune réponse valide reçue")
        
        # Optionnel: Demander l'état de la LED après 1 seconde
        time.sleep(1)
        print("\n--- Vérification de l'état de la LED ---")
        status_command = {
            "command": "led_status"
        }
        status_response = send_command(sock, status_command)
        
        if status_response:
            print(f"État actuel de la LED: {'Allumée' if status_response.get('led_state') else 'Éteinte'}")
        
    except KeyboardInterrupt:
        print("\n\nInterruption par l'utilisateur")
    except Exception as e:
        print(f"\n✗ Erreur: {e}")
    finally:
        if sock:
            sock.close()
            print("\nConnexion fermée")

if __name__ == "__main__":
    main()
