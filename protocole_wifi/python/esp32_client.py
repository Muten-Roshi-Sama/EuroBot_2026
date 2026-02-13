"""
Client simple pour tester la communication avec l'ESP32
Lance un serveur TCP pour simuler l'ESP32 
"""

import socket
import threading
import json
import time
from datetime import datetime


class FakeESP32Server:
    """Serveur TCP qui simule un ESP32"""
    
    def __init__(self, host="0.0.0.0", port=5000):
        self.host = host
        self.port = port
        self.server_socket = None
        self.running = False
        self.led_state = False
        self.led_pin = 16
        self.boot_time = time.time()
    
    def start(self):
        """Démarrer le serveur"""
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind((self.host, self.port))
        self.server_socket.listen(5)
        self.running = True
        
        print(f"🌐 Serveur ESP32 simulé en écoute sur {self.host}:{self.port}")
        
        server_thread = threading.Thread(target=self._accept_connections, daemon=True)
        server_thread.start()
    
    def stop(self):
        """Arrêter le serveur"""
        self.running = False
        if self.server_socket:
            self.server_socket.close()
        print("Serveur arrêté")
    
    def _accept_connections(self):
        """Accepter les connexions TCP"""
        while self.running:
            try:
                client_socket, client_addr = self.server_socket.accept()
                print(f"✅ Client connecté: {client_addr}")
                
                client_thread = threading.Thread(
                    target=self._handle_client,
                    args=(client_socket, client_addr),
                    daemon=True
                )
                client_thread.start()
            except:
                break
    
    def _handle_client(self, client_socket, client_addr):
        """Gérer la communication avec un client"""
        buffer = ""
        
        try:
            while self.running:
                data = client_socket.recv(1024)
                
                if not data:
                    print(f"🔌 Client {client_addr} déconnecté")
                    break
                
                buffer += data.decode('utf-8', errors='ignore')
                
                # Parser les messages JSON
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    line = line.strip()
                    
                    if line:
                        self._process_command(line, client_socket)
        
        except Exception as e:
            print(f"❌ Erreur client {client_addr}: {e}")
        finally:
            client_socket.close()
    
    def _process_command(self, json_str, client_socket):
        """Traiter une commande JSON"""
        try:
            cmd_data = json.loads(json_str)
            command = cmd_data.get("cmd")
            params = cmd_data.get("params", {})
            
            print(f"📨 Reçu: {command} | {params}")
            
            response = None
            
            if command == "LED_ON":
                pin = params.get("pin", 16)
                self.led_state = True
                response = {
                    "status": "ok",
                    "message": f"LED (pin {pin}) allumée",
                    "data": {"led_state": True}
                }
                print(f"🟢 LED PIN {pin} ALLUMÉE")
            
            elif command == "LED_OFF":
                pin = params.get("pin", 16)
                self.led_state = False
                response = {
                    "status": "ok",
                    "message": f"LED (pin {pin}) éteinte",
                    "data": {"led_state": False}
                }
                print(f"🔴 LED PIN {pin} ÉTEINTE")
            
            elif command == "LED_BLINK":
                pin = params.get("pin", 16)
                interval = params.get("interval", 500)
                cycles = params.get("cycles", 5)
                
                response = {
                    "status": "ok",
                    "message": f"Clignotement démarré",
                    "data": {
                        "pin": pin,
                        "interval": interval,
                        "cycles": cycles
                    }
                }
                print(f"💫 CLIGNOTEMENT pin {pin}: {interval}ms x {cycles} cycles")
                
                # Simuler le clignotement
                def blink_thread():
                    for i in range(cycles):
                        self.led_state = not self.led_state
                        time.sleep(interval / 1000.0)
                
                blink = threading.Thread(target=blink_thread, daemon=True)
                blink.start()
            
            elif command == "GET_STATUS":
                uptime = int(time.time() - self.boot_time)
                rssi = -45  # WiFi signal (simulé)
                
                response = {
                    "status": "ok",
                    "message": "Status ESP32",
                    "data": {
                        "uptime": uptime,
                        "uptime_str": f"{uptime}s",
                        "rssi": rssi,
                        "led_state": self.led_state,
                        "free_heap": 150000,
                        "chip_id": "0x12345678"
                    }
                }
                print(f"📊 Status demandé - Uptime: {uptime}s, RSSI: {rssi}dBm, LED: {self.led_state}")
            
            elif command == "RESET":
                response = {
                    "status": "ok",
                    "message": "Redémarrage en cours...",
                    "data": {}
                }
                print("🔄 RESET demandé - Destruction de la connexion...")
                
                # Envoyer et fermer
                client_socket.send((json.dumps(response) + "\n").encode())
                time.sleep(0.5)
                client_socket.close()
                return
            
            else:
                response = {
                    "status": "error",
                    "message": f"Commande inconnue: {command}",
                    "data": {}
                }
            
            # Envoyer la réponse
            if response:
                response_json = json.dumps(response) + "\n"
                client_socket.send(response_json.encode())
        
        except json.JSONDecodeError as e:
            error_response = {
                "status": "error",
                "message": f"Erreur JSON: {str(e)}",
                "data": {}
            }
            client_socket.send((json.dumps(error_response) + "\n").encode())
        except Exception as e:
            print(f"❌ Erreur traitement: {e}")


def main():
    """Démarrer le serveur simulé"""
    import argparse
    
    parser = argparse.ArgumentParser(description="Serveur ESP32 simulé")
    parser.add_argument("--host", default="0.0.0.0", help="Adresse d'écoute")
    parser.add_argument("--port", type=int, default=5000, help="Port TCP")
    
    args = parser.parse_args()
    
    server = FakeESP32Server(host=args.host, port=args.port)
    server.start()
    
    try:
        print("Appuyez sur Ctrl+C pour arrêter...")
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n⏹️  Arrêt du serveur...")
        server.stop()


if __name__ == "__main__":
    main()
