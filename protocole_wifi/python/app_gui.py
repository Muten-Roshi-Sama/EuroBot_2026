"""
Application GUI pour contrôler l'ESP32 à distance
Interface simple pour allumer/éteindre/clignoter LED et afficher les logs
Connexion TCP, console colorisée en temps réel
"""

import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext
import threading
import time
from datetime import datetime

from wifi_client import WiFiClient
from protocol import (
    make_led_on, make_led_off, make_led_blink, 
    make_get_status, make_reset
)


class RobotControlApp:
    def __init__(self, root):
        self.root = root
        self.root.title("🤖 EuroBot 2026 - Contrôle WiFi ESP32")
        self.root.geometry("1000x750")
        self.root.resizable(True, True)
        
        self.client = None
        self.connected = False
        
        self._create_ui()
        self._load_config()
    
    def _create_ui(self):
        """Créer l'interface utilisateur"""
        
        # ===== FRAME CONNEXION =====
        frame_connection = ttk.LabelFrame(self.root, text="🌐 Connexion WiFi TCP", padding=10)
        frame_connection.pack(fill="x", padx=10, pady=10)
        
        # Ligne 1: IP et Port
        ttk.Label(frame_connection, text="IP ESP32:").grid(row=0, column=0, sticky="w")
        self.entry_ip = ttk.Entry(frame_connection, width=20)
        self.entry_ip.insert(0, "192.168.1.100")
        self.entry_ip.grid(row=0, column=1, padx=5)
        
        ttk.Label(frame_connection, text="Port:").grid(row=0, column=2, sticky="w")
        self.entry_port = ttk.Entry(frame_connection, width=10)
        self.entry_port.insert(0, "5000")
        self.entry_port.grid(row=0, column=3, padx=5)
        
        self.btn_connect = ttk.Button(frame_connection, text="🔗 Connecter", 
                                     command=self._on_connect)
        self.btn_connect.grid(row=0, column=4, padx=5)
        
        self.btn_disconnect = ttk.Button(frame_connection, text="❌ Déconnecter", 
                                        command=self._on_disconnect, state="disabled")
        self.btn_disconnect.grid(row=0, column=5, padx=5)
        
        # Status
        self.label_status = ttk.Label(frame_connection, text="❌ Déconnecté", 
                                     foreground="red", font=("Arial", 10, "bold"))
        self.label_status.grid(row=1, column=0, columnspan=6, sticky="w", pady=5)
        
        # ===== FRAME COMMANDES LED =====
        frame_commands = ttk.LabelFrame(self.root, text="💡 Contrôle LED (Pin 16)", padding=10)
        frame_commands.pack(fill="x", padx=10, pady=10)
        
        # Boutons allumer/éteindre
        self.btn_led_on = ttk.Button(frame_commands, text="🟢 Allumer", 
                                    command=self._on_led_on, state="disabled")
        self.btn_led_on.grid(row=0, column=0, padx=5, pady=5, sticky="ew")
        
        self.btn_led_off = ttk.Button(frame_commands, text="🔴 Éteindre", 
                                     command=self._on_led_off, state="disabled")
        self.btn_led_off.grid(row=0, column=1, padx=5, pady=5, sticky="ew")
        
        # Clignoter avec paramètres
        ttk.Label(frame_commands, text="Clignoter:").grid(row=1, column=0, sticky="w", pady=5)
        
        ttk.Label(frame_commands, text="Intervalle (ms):").grid(row=1, column=1, sticky="e")
        self.entry_interval = ttk.Entry(frame_commands, width=10)
        self.entry_interval.insert(0, "500")
        self.entry_interval.grid(row=1, column=2, padx=5)
        
        ttk.Label(frame_commands, text="Cycles:").grid(row=1, column=3, sticky="e")
        self.entry_cycles = ttk.Entry(frame_commands, width=10)
        self.entry_cycles.insert(0, "5")
        self.entry_cycles.grid(row=1, column=4, padx=5)
        
        self.btn_led_blink = ttk.Button(frame_commands, text="💫 Clignoter", 
                                       command=self._on_led_blink, state="disabled")
        self.btn_led_blink.grid(row=1, column=5, padx=5)
        
        # Configure weights pour expander
        for i in range(6):
            frame_commands.columnconfigure(i, weight=1)
        
        # ===== FRAME ACTIONS SYSTÈME =====
        frame_system = ttk.LabelFrame(self.root, text="⚙️ Système", padding=10)
        frame_system.pack(fill="x", padx=10, pady=10)
        
        self.btn_status = ttk.Button(frame_system, text="📊 Demander Status", 
                                    command=self._on_get_status, state="disabled")
        self.btn_status.grid(row=0, column=0, padx=5, sticky="ew")
        
        self.btn_reset = ttk.Button(frame_system, text="🔄 Redémarrer ESP32", 
                                   command=self._on_reset, state="disabled")
        self.btn_reset.grid(row=0, column=1, padx=5, sticky="ew")
        
        self.btn_clear_logs = ttk.Button(frame_system, text="🗑️ Effacer les logs", 
                                        command=self._on_clear_logs)
        self.btn_clear_logs.grid(row=0, column=2, padx=5, sticky="ew")
        
        for i in range(3):
            frame_system.columnconfigure(i, weight=1)
        
        # ===== FRAME LOGS/CONSOLE =====
        frame_logs = ttk.LabelFrame(self.root, text="📋 Console / Logs en temps réel", padding=10)
        frame_logs.pack(fill="both", expand=True, padx=10, pady=10)
        
        # Text area avec scrollbar
        self.text_logs = scrolledtext.ScrolledText(frame_logs, height=15, width=80, 
                                                  state="disabled", wrap=tk.WORD,
                                                  font=("Courier", 9))
        self.text_logs.pack(fill="both", expand=True)
        
        # Tags pour coloration
        self.text_logs.tag_config("error", foreground="red", font=("Courier", 9, "bold"))
        self.text_logs.tag_config("success", foreground="green", font=("Courier", 9, "bold"))
        self.text_logs.tag_config("info", foreground="blue", font=("Courier", 9))
        self.text_logs.tag_config("serial", foreground="black", font=("Courier", 9))
        self.text_logs.tag_config("warning", foreground="orange", font=("Courier", 9, "bold"))
        self.text_logs.tag_config("status", foreground="purple", font=("Courier", 9))
        
        # Log initial
        self._log("=== EuroBot 2026 Contrôle WiFi ===", "info")
        self._log("Entrez l'IP de l'ESP32 et cliquez sur 'Connecter'", "info")
        self._log("Protocole: JSON TCP", "info")
    
    def _on_connect(self):
        """Établir la connexion TCP"""
        try:
            ip = self.entry_ip.get().strip()
            port = int(self.entry_port.get().strip())
            
            if not ip:
                messagebox.showerror("Erreur", "Entrez une adresse IP")
                return
            
            self._log(f"📡 Connexion à {ip}:{port}...", "info")
            self.root.update()
            
            self.client = WiFiClient(
                host=ip,
                port=port,
                on_message=self._on_message,
                on_connection=self._on_connection_changed,
                on_error=self._on_wifi_error,
                timeout=5.0
            )
            
            if self.client.connect():
                self._update_connection_state(True)
                self._log(f"✓ Connecté à {ip}:{port}", "success")
            else:
                self._log(f"✗ Impossible de se connecter à {ip}:{port}", "error")
                
        except ValueError:
            messagebox.showerror("Erreur", "Port invalide")
    
    def _on_disconnect(self):
        """Fermer la connexion"""
        if self.client:
            self.client.disconnect()
            self._update_connection_state(False)
            self._log("🔌 Déconnecté", "info")
    
    def _on_led_on(self):
        """Allumer la LED"""
        if self.client and self.client.is_connected():
            cmd = make_led_on(16)
            self._log(f"→ Envoi: LED_ON (pin 16)", "info")
            self.client.send(cmd)
    
    def _on_led_off(self):
        """Éteindre la LED"""
        if self.client and self.client.is_connected():
            cmd = make_led_off(16)
            self._log(f"→ Envoi: LED_OFF (pin 16)", "info")
            self.client.send(cmd)
    
    def _on_led_blink(self):
        """Clignoter la LED"""
        try:
            interval = int(self.entry_interval.get())
            cycles = int(self.entry_cycles.get())
            
            if interval <= 0 or cycles <= 0:
                messagebox.showerror("Erreur", "Intervalle et cycles doivent être positifs")
                return
            
            if self.client and self.client.is_connected():
                cmd = make_led_blink(16, interval, cycles)
                self._log(f"→ Envoi: LED_BLINK (pin 16, interval={interval}ms, cycles={cycles})", "info")
                self.client.send(cmd)
        except ValueError:
            messagebox.showerror("Erreur", "Paramètres invalides (nombres entiers)")
    
    def _on_get_status(self):
        """Demander le status"""
        if self.client and self.client.is_connected():
            cmd = make_get_status()
            self._log(f"→ Envoi: GET_STATUS", "info")
            self.client.send(cmd)
    
    def _on_reset(self):
        """Redémarrer l'ESP32"""
        if messagebox.askyesno("Confirmation", "Êtes-vous sûr de vouloir redémarrer l'ESP32?"):
            if self.client and self.client.is_connected():
                cmd = make_reset()
                self._log(f"→ Envoi: RESET (redémarrage en cours...)", "warning")
                self.client.send(cmd)
                # La connexion sera fermée après le redémarrage
    
    def _on_clear_logs(self):
        """Effacer les logs"""
        self.text_logs.config(state="normal")
        self.text_logs.delete(1.0, tk.END)
        self.text_logs.config(state="disabled")
    
    def _on_message(self, data):
        """Callback quand un message est reçu"""
        if not data:
            return
        
        # Afficher le message reçu
        if isinstance(data, dict):
            status = data.get("status", "unknown")
            message = data.get("message", "")
            msg_data = data.get("data", {})
            
            # Détermine la couleur selon le status
            tag = "success" if status == "ok" else "error" if status == "error" else "info"
            
            # Format: ← Reçu: [status] message {data}
            log_msg = f"← Reçu: [{status}] {message}"
            if msg_data:
                log_msg += f" | {msg_data}"
            
            self._log(log_msg, tag)
        else:
            self._log(f"← Reçu (raw): {data}", "serial")
    
    def _on_connection_changed(self, connected):
        """Callback changement connexion"""
        self._update_connection_state(connected)
    
    def _on_wifi_error(self, error):
        """Callback erreur WiFi"""
        self._log(f"⚠️ Erreur WiFi: {error}", "error")
        self._update_connection_state(False)
    
    def _update_connection_state(self, connected):
        """Mettre à jour l'état de l'UI selon la connexion"""
        self.connected = connected
        
        if connected:
            self.label_status.config(text="✅ Connecté", foreground="green")
            self.btn_connect.config(state="disabled")
            self.btn_disconnect.config(state="normal")
            self.btn_led_on.config(state="normal")
            self.btn_led_off.config(state="normal")
            self.btn_led_blink.config(state="normal")
            self.btn_status.config(state="normal")
            self.btn_reset.config(state="normal")
            self.entry_ip.config(state="disabled")
            self.entry_port.config(state="disabled")
        else:
            self.label_status.config(text="❌ Déconnecté", foreground="red")
            self.btn_connect.config(state="normal")
            self.btn_disconnect.config(state="disabled")
            self.btn_led_on.config(state="disabled")
            self.btn_led_off.config(state="disabled")
            self.btn_led_blink.config(state="disabled")
            self.btn_status.config(state="disabled")
            self.btn_reset.config(state="disabled")
            self.entry_ip.config(state="normal")
            self.entry_port.config(state="normal")
    
    def _log(self, message: str, tag: str = "serial"):
        """Ajouter un message au log console"""
        self.text_logs.config(state="normal")
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.text_logs.insert(tk.END, f"[{timestamp}] {message}\n", tag)
        self.text_logs.see(tk.END)  # Auto-scroll vers le bas
        self.text_logs.config(state="disabled")
    
    def _load_config(self):
        """Charger configuration (à implémenter si besoin)"""
        # Vous pouvez ajouter ici le chargement d'une config JSON
        pass
    
    def on_closing(self):
        """Événement fermeture fenêtre"""
        if self.connected and self.client:
            self.client.disconnect()
        self.root.destroy()


def main():
    root = tk.Tk()
    app = RobotControlApp(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()


if __name__ == "__main__":
    main()
