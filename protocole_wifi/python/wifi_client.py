"""
Client TCP pour communication avec ESP32
Gère connexion, envoi/réception messages, reconnexion automatique
"""

import socket
import threading
import time
from typing import Callable, Optional
from protocol import parse_message


class WiFiClient:
    def __init__(
        self,
        host: str,
        port: int,
        on_message: Optional[Callable] = None,
        on_connection: Optional[Callable] = None,
        on_error: Optional[Callable] = None,
        timeout: float = 5.0
    ):
        """
        Initialiser le client WiFi
        
        Args:
            host: Adresse IP de l'ESP32
            port: Port TCP
            on_message: Callback reçu message - signature: fn(dict_response)
            on_connection: Callback changement connexion - signature: fn(bool_connected)
            on_error: Callback erreur - signature: fn(str_error)
            timeout: Timeout socket en secondes
        """
        self.host = host
        self.port = port
        self.timeout = timeout
        
        self.on_message = on_message
        self.on_connection = on_connection
        self.on_error = on_error
        
        self.socket = None
        self.connected = False
        self.receive_thread = None
        self.stop_event = threading.Event()
    
    def connect(self) -> bool:
        """
        Établir la connexion TCP
        
        Returns:
            True si succès, False sinon
        """
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(self.timeout)
            self.socket.connect((self.host, self.port))
            
            self.connected = True
            self.stop_event.clear()
            
            # Démarrer thread de réception
            self.receive_thread = threading.Thread(target=self._receive_loop, daemon=True)
            self.receive_thread.start()
            
            if self.on_connection:
                self.on_connection(True)
            
            return True
            
        except (socket.timeout, socket.error, OSError) as e:
            self.connected = False
            if self.on_error:
                self.on_error(str(e))
            return False
    
    def disconnect(self):
        """Fermer la connexion"""
        self.stop_event.set()
        self.connected = False
        
        if self.socket:
            try:
                self.socket.close()
            except:
                pass
        
        if self.on_connection:
            self.on_connection(False)
    
    def send(self, message: str) -> bool:
        """
        Envoyer un message
        
        Args:
            message: String (déjà formaté en JSON)
        
        Returns:
            True si succès, False sinon
        """
        if not self.connected or not self.socket:
            if self.on_error:
                self.on_error("Non connecté")
            return False
        
        try:
            # Assurer qu'on envoie un byte string
            if isinstance(message, str):
                message = message.encode('utf-8')
            
            # Ajouter newline si absent
            if not message.endswith(b'\n'):
                message += b'\n'
            
            self.socket.sendall(message)
            return True
            
        except socket.error as e:
            self.connected = False
            if self.on_error:
                self.on_error(f"Erreur envoi: {e}")
            return False
    
    def is_connected(self) -> bool:
        """Vérifier l'état de la connexion"""
        return self.connected
    
    def _receive_loop(self):
        """Boucle de réception (tourne en thread)"""
        buffer = ""
        
        while not self.stop_event.is_set() and self.connected:
            try:
                data = self.socket.recv(4096)
                
                if not data:
                    # Connexion fermée par serveur
                    self.connected = False
                    if self.on_connection:
                        self.on_connection(False)
                    break
                
                # Décoder et ajouter au buffer
                buffer += data.decode('utf-8', errors='ignore')
                
                # Parser les messages (séparés par \n)
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    line = line.strip()
                    
                    if line:
                        # Parser et appeler callback
                        parsed = parse_message(line)
                        if self.on_message:
                            self.on_message(parsed)
                
            except socket.timeout:
                # Timeout normal, continuer
                continue
            
            except socket.error as e:
                self.connected = False
                if self.on_error:
                    self.on_error(f"Erreur réception: {e}")
                if self.on_connection:
                    self.on_connection(False)
                break
    
    def __del__(self):
        """Nettoyage"""
        self.disconnect()
