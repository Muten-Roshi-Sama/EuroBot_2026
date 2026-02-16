"""
Application GUI pour contrôler l'ESP32 à distance via WiFi TCP
"""

import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext
from datetime import datetime

from wifi_client import WiFiClient
from protocol import (
    make_led_on, make_led_off, make_led_blink, 
    make_get_status, make_reset,
    make_fsm_set_ready, make_fsm_trigger_launch, make_fsm_emergency_stop
)


class RobotControlApp:
    def __init__(self, root):
        self.root = root
        self.root.title("🤖 EuroBot 2026 - Contrôle WiFi ESP32")
        self.root.geometry("1100x800")
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
        ttk.Label(frame_connection, text="IP ESP32:").grid(row=0, column=0, sticky="w", padx=5)
        self.entry_ip = ttk.Entry(frame_connection, width=20)
        self.entry_ip.insert(0, "192.168.4.1")  # IP par défaut du WiFi
        self.entry_ip.grid(row=0, column=1, padx=5)
        
        ttk.Label(frame_connection, text="Port:").grid(row=0, column=2, sticky="w", padx=5)
        self.entry_port = ttk.Entry(frame_connection, width=10)
        self.entry_port.insert(0, "5000")
        self.entry_port.grid(row=0, column=3, padx=5)
        
        self.btn_connect = ttk.Button(frame_connection, text="🔗 Connecter", 
                                     command=self._on_connect)
        self.btn_connect.grid(row=0, column=4, padx=5)
        
        self.btn_disconnect = ttk.Button(frame_connection, text="❌ Déconnecter", 
                                        command=self._on_disconnect, state="disabled")
        self.btn_disconnect.grid(row=0, column=5, padx=5)
        
        # Module type
        self.label_module = ttk.Label(frame_connection, text="Module: Non détecté", 
                                     foreground="gray", font=("Arial", 9))
        self.label_module.grid(row=1, column=0, columnspan=2, sticky="w", pady=5)
        
        # Status
        self.label_status = ttk.Label(frame_connection, text="❌ Déconnecté", 
                                     foreground="red", font=("Arial", 10, "bold"))
        self.label_status.grid(row=1, column=2, columnspan=4, sticky="w", pady=5)
        
        # Quick presets
        ttk.Label(frame_connection, text="Quick:", font=("Arial", 9, "bold")).grid(row=2, column=0, sticky="w")
        
        preset_frame = ttk.Frame(frame_connection)
        preset_frame.grid(row=2, column=1, columnspan=5, sticky="w")
        
        ttk.Button(preset_frame, text="ESP32 (Default)", width=18,
                  command=lambda: self._set_connection("192.168.4.1", "5001")).pack(side="left", padx=2)
        ttk.Button(preset_frame, text="Custom IP...", width=18,
                  command=self._custom_ip).pack(side="left", padx=2)
        
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
        
        # ===== FRAME FSM CONTROLS =====
        frame_fsm = ttk.LabelFrame(self.root, text="🕹️ Contrôle FSM", padding=10)
        frame_fsm.pack(fill="x", padx=10, pady=10)
        self.frame_fsm = frame_fsm  # Store reference for enable/disable
        
        # FSM Status Display
        self.label_fsm_status = ttk.Label(frame_fsm, text="État: 🟢 Prêt", 
                                         foreground="green", font=("Arial", 9))
        self.label_fsm_status.pack(anchor="w", pady=5)
        
        # Buttons frame
        btn_frame = ttk.Frame(frame_fsm)
        btn_frame.pack(fill="x", expand=True, pady=5)
        
        self.btn_fsm_ready = ttk.Button(btn_frame, text="✓ Ready", 
                                       command=self._on_fsm_ready, state="disabled", width=15)
        self.btn_fsm_ready.pack(side="left", padx=5)
        
        self.btn_fsm_launch = ttk.Button(btn_frame, text="🚀 Launch!", 
                                        command=self._on_fsm_launch, state="disabled", width=15)
        self.btn_fsm_launch.pack(side="left", padx=5)
        
        self.btn_fsm_emergency = ttk.Button(btn_frame, text="🛑 Emergency STOP", 
                                           command=self._on_fsm_emergency, state="disabled", width=15)
        self.btn_fsm_emergency.pack(side="left", padx=5)
        
        # Status LED indicators
        status_frame = ttk.Frame(frame_fsm)
        status_frame.pack(fill="x", pady=5)
        
        ttk.Label(status_frame, text="Flags: ").pack(side="left")
        
        self.label_ready_indicator = ttk.Label(status_frame, text="🔴 Ready", foreground="red", font=("Arial", 9))
        self.label_ready_indicator.pack(side="left", padx=10)
        
        self.label_launch_indicator = ttk.Label(status_frame, text="🔴 Launch", foreground="red", font=("Arial", 9))
        self.label_launch_indicator.pack(side="left", padx=10)
        
        self.label_emergency_indicator = ttk.Label(status_frame, text="🔴 Emergency", foreground="red", font=("Arial", 9))
        self.label_emergency_indicator.pack(side="left", padx=10)
        
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
    
    def _set_connection(self, ip, port):
        """Set IP and Port then connect"""
        self.entry_ip.delete(0, tk.END)
        self.entry_ip.insert(0, ip)
        self.entry_port.delete(0, tk.END)
        self.entry_port.insert(0, port)
        self._on_connect()
    
    def _custom_ip(self):
        """Ask for custom IP"""
        top = tk.Toplevel(self.root)
        top.title("IP Personnalisée")
        top.geometry("300x150")
        
        ttk.Label(top, text="Adresse IP:").pack(pady=5)
        entry_ip = ttk.Entry(top, width=20)
        entry_ip.pack(pady=5)
        entry_ip.insert(0, self.entry_ip.get())
        
        ttk.Label(top, text="Port:").pack(pady=5)
        entry_port = ttk.Entry(top, width=20)
        entry_port.pack(pady=5)
        entry_port.insert(0, self.entry_port.get())
        
        def set_custom():
            self._set_connection(entry_ip.get(), entry_port.get())
            top.destroy()
        
        ttk.Button(top, text="Connecter", command=set_custom).pack(pady=10)
    
    def _on_connect(self):
        """Établir la connexion TCP"""
        try:
            ip = self.entry_ip.get().strip()
            port = int(self.entry_port.get().strip())
            
            if not ip:
                messagebox.showerror("Erreur", "Entrez une adresse IP")
                return
            
            self._log(f"📡 Tentative de connexion à {ip}:{port}...", "info")
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
                self._log(f"✓ Connecté à {ip}:{port}", "success")
                self._update_connection_state(True)
            else:
                self._log(f"✗ Connexion échouée à {ip}:{port}", "error")
                
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
            self.client.send(cmd)
    
    def _on_led_off(self):
        """Éteindre la LED"""
        if self.client and self.client.is_connected():
            cmd = make_led_off(16)
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
                self.client.send(cmd)
        except ValueError:
            messagebox.showerror("Erreur", "Paramètres invalides (nombres entiers)")
    
    def _on_get_status(self):
        """Demander le status"""
        if self.client and self.client.is_connected():
            cmd = make_get_status()
            self.client.send(cmd)
    
    def _on_reset(self):
        """Redémarrer l'ESP32"""
        if messagebox.askyesno("Confirmation", "Êtes-vous sûr de vouloir redémarrer l'ESP32?"):
            if self.client and self.client.is_connected():
                cmd = make_reset()
                self._log(f"Redémarrage ESP32...", "warning")
                self.client.send(cmd)
    
    def _on_clear_logs(self):
        """Effacer les logs"""
        self.text_logs.config(state="normal")
        self.text_logs.delete(1.0, tk.END)
        self.text_logs.config(state="disabled")
    
    def _on_message(self, data):
        """Callback quand un message est reçu"""
        if not data:
            return
        
        if isinstance(data, dict):
            status = data.get("status", "unknown")
            message = data.get("message", "")
            tag = "success" if status == "ok" else "error" if status == "error" else "info"
            self._log(f"[{status}] {message}", tag)
    
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
            self.label_module.config(text="Module: 🤖 EuroBot Control", foreground="green")
            
            self.btn_connect.config(state="disabled")
            self.btn_disconnect.config(state="normal")
            self.btn_led_on.config(state="normal")
            self.btn_led_off.config(state="normal")
            self.btn_led_blink.config(state="normal")
            self.btn_status.config(state="normal")
            self.btn_reset.config(state="normal")
            self.btn_fsm_ready.config(state="normal")
            self.btn_fsm_launch.config(state="normal")
            self.btn_fsm_emergency.config(state="normal")
            self.entry_ip.config(state="disabled")
            self.entry_port.config(state="disabled")
        else:
            self.label_status.config(text="❌ Déconnecté", foreground="red")
            self.label_module.config(text="Module: Non détecté", foreground="gray")
            
            self.btn_connect.config(state="normal")
            self.btn_disconnect.config(state="disabled")
            self.btn_led_on.config(state="disabled")
            self.btn_led_off.config(state="disabled")
            self.btn_led_blink.config(state="disabled")
            self.btn_status.config(state="disabled")
            self.btn_reset.config(state="disabled")
            self.btn_fsm_ready.config(state="disabled")
            self.btn_fsm_launch.config(state="disabled")
            self.btn_fsm_emergency.config(state="disabled")
            self.entry_ip.config(state="normal")
            self.entry_port.config(state="normal")
    

    def _on_fsm_ready(self):
        """Envoyer FSM_SET_READY command"""
        if self.client and self.client.is_connected():
            cmd = make_fsm_set_ready(True)
            self.client.send(cmd)
            self._log("📤 [FSM] SET_READY = True", "info")
            self.label_ready_indicator.config(text="🟢 Ready", foreground="green")
    
    def _on_fsm_launch(self):
        """Envoyer FSM_TRIGGER_LAUNCH command (demander confirmation)"""
        if messagebox.askyesno("Confirmation", "Êtes-vous sûr de vouloir déclencher le lancement?"):
            if self.client and self.client.is_connected():
                cmd = make_fsm_trigger_launch(True)
                self.client.send(cmd)
                self._log("🚀 [FSM] TRIGGER_LAUNCH = True", "warning")
                self.label_launch_indicator.config(text="🟡 Launch", foreground="orange")
    
    def _on_fsm_emergency(self):
        """Envoyer FSM_EMERGENCY_STOP command (confirmation urgente)"""
        if messagebox.showwarning("⚠️ ARRÊT D'URGENCE", 
                                  "ÊTES-VOUS ABSOLUMENT SÛR?\n\nCeci arrêtera immédiatement le robot!",
                                  icon="warning"):
            if self.client and self.client.is_connected():
                cmd = make_fsm_emergency_stop(True)
                self.client.send(cmd)
                self._log("🛑 [FSM] EMERGENCY_STOP = True - ROBOT ARRÊTÉ!", "error")
                self.label_emergency_indicator.config(text="🟠 Emergency", foreground="red")
    
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
