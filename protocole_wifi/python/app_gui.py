"""
Application GUI pour contrôler l'ESP32 à distance via Bluetooth BLE
"""
# [WiFi] Application GUI pour contrôler l'ESP32 à distance via WiFi TCP

import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext
from datetime import datetime
import threading

# [WiFi] from wifi_client import WiFiClient
from ble_client import BLEClient
from protocol import (
    make_led_on, make_led_off, make_led_blink, 
    make_get_status, make_reset,
    make_fsm_set_ready, make_fsm_trigger_launch, make_fsm_emergency_stop
)


class RobotControlApp:
    def __init__(self, root):
        self.root = root
        self.root.title("🤖 EuroBot 2026 - Contrôle BLE ESP32")
        # [WiFi] self.root.title("🤖 EuroBot 2026 - Contrôle WiFi ESP32")
        self.root.geometry("1100x800")
        self.root.resizable(True, True)
        
        self.client = None
        self.connected = False
        
        self._create_ui()
        self._load_config()
    
    def _create_ui(self):
        """Créer l'interface utilisateur"""
        
        # ===== FRAME CONNEXION =====
        # [WiFi] frame_connection = ttk.LabelFrame(self.root, text="🌐 Connexion WiFi TCP", ...)
        frame_connection = ttk.LabelFrame(self.root, text="📶 Connexion Bluetooth BLE", padding=10)
        frame_connection.pack(fill="x", padx=10, pady=10)

        # [WiFi] Ligne 1: IP et Port
        # [WiFi] ttk.Label(frame_connection, text="IP ESP32:")
        # [WiFi] self.entry_ip = ttk.Entry(frame_connection, width=20); insert(0, "192.168.4.1")
        # [WiFi] ttk.Label(frame_connection, text="Port:")
        # [WiFi] self.entry_port = ttk.Entry(frame_connection, width=10); insert(0, "5000")

        # Ligne 1: Nom BLE + adresse MAC
        ttk.Label(frame_connection, text="Nom BLE:").grid(row=0, column=0, sticky="w", padx=5)
        self.entry_device_name = ttk.Entry(frame_connection, width=20)
        self.entry_device_name.insert(0, "EuroBot")
        self.entry_device_name.grid(row=0, column=1, padx=5)

        ttk.Label(frame_connection, text="Adresse MAC (opt.):").grid(row=0, column=2, sticky="w", padx=5)
        self.entry_mac = ttk.Entry(frame_connection, width=20)
        self.entry_mac.grid(row=0, column=3, padx=5)

        self.btn_scan = ttk.Button(frame_connection, text="🔍 Scanner",
                                   command=self._on_scan)
        self.btn_scan.grid(row=0, column=4, padx=5)

        self.btn_connect = ttk.Button(frame_connection, text="🔗 Connecter",
                                      command=self._on_connect)
        self.btn_connect.grid(row=0, column=5, padx=5)

        self.btn_disconnect = ttk.Button(frame_connection, text="❌ Déconnecter",
                                         command=self._on_disconnect, state="disabled")
        self.btn_disconnect.grid(row=0, column=6, padx=5)

        # Module type
        self.label_module = ttk.Label(frame_connection, text="Module: Non détecté",
                                      foreground="gray", font=("Arial", 9))
        self.label_module.grid(row=1, column=0, columnspan=3, sticky="w", pady=5)

        # Status
        self.label_status = ttk.Label(frame_connection, text="❌ Déconnecté",
                                      foreground="red", font=("Arial", 10, "bold"))
        self.label_status.grid(row=1, column=3, columnspan=4, sticky="w", pady=5)

        # Quick presets
        ttk.Label(frame_connection, text="Quick:", font=("Arial", 9, "bold"))\
            .grid(row=2, column=0, sticky="w")
        preset_frame = ttk.Frame(frame_connection)
        preset_frame.grid(row=2, column=1, columnspan=6, sticky="w")
        # [WiFi] ttk.Button(preset_frame, text="ESP32 (Default)", command=lambda: self._set_connection(...))
        # [WiFi] ttk.Button(preset_frame, text="Custom IP...", command=self._custom_ip)
        ttk.Button(preset_frame, text="EuroBot (défaut)", width=18,
                   command=lambda: self._set_ble_name("EuroBot")).pack(side="left", padx=2)
        ttk.Button(preset_frame, text="Adresse manuelle...", width=18,
                   command=self._custom_address).pack(side="left", padx=2)
        
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
    
    # [WiFi] def _set_connection(self, ip, port): ...
    def _set_ble_name(self, name: str):
        """Pré-remplir le nom BLE"""
        self.entry_device_name.delete(0, tk.END)
        self.entry_device_name.insert(0, name)

    # [WiFi] def _custom_ip(self): ...
    def _custom_address(self):
        """Saisir manuellement l'adresse MAC"""
        top = tk.Toplevel(self.root)
        top.title("Adresse MAC manuelle")
        top.geometry("320x140")
        top.resizable(False, False)

        ttk.Label(top, text="Adresse MAC (ex: AA:BB:CC:DD:EE:FF):").pack(pady=8)
        entry_mac = ttk.Entry(top, width=26)
        entry_mac.pack(pady=5)
        entry_mac.insert(0, self.entry_mac.get())

        def set_mac():
            self.entry_mac.delete(0, tk.END)
            self.entry_mac.insert(0, entry_mac.get().strip())
            top.destroy()

        ttk.Button(top, text="Confirmer", command=set_mac).pack(pady=10)

    def _on_scan(self):
        """Scanner les appareils BLE et afficher une fenêtre de sélection"""
        top = tk.Toplevel(self.root)
        top.title("🔍 Scan BLE")
        top.geometry("420x300")
        top.resizable(False, False)

        ttk.Label(top, text="Appareils BLE détectés :", font=("Arial", 10, "bold")).pack(pady=8)
        frame_list = ttk.Frame(top)
        frame_list.pack(fill="both", expand=True, padx=10)

        scrollbar = ttk.Scrollbar(frame_list)
        scrollbar.pack(side="right", fill="y")
        listbox = tk.Listbox(frame_list, yscrollcommand=scrollbar.set, width=50, height=8)
        listbox.pack(side="left", fill="both", expand=True)
        scrollbar.config(command=listbox.yview)

        lbl_scan = ttk.Label(top, text="Scan en cours...", foreground="blue")
        lbl_scan.pack(pady=4)
        ttk.Button(top, text="✅ Sélectionner",
                   command=lambda: _select()).pack(pady=4)

        devices_found = []

        def do_scan():
            import asyncio
            from bleak import BleakScanner
            loop = asyncio.new_event_loop()
            devs = loop.run_until_complete(BleakScanner.discover(timeout=6.0))
            loop.close()
            self.root.after(0, lambda: _populate(devs))

        def _populate(devs):
            lbl_scan.config(text=f"{len(devs)} appareil(s) trouvé(s)")
            listbox.delete(0, tk.END)
            devices_found.clear()
            for d in devs:
                listbox.insert(tk.END, f"{d.name or '(sans nom)'}  —  {d.address}")
                devices_found.append(d)

        def _select():
            sel = listbox.curselection()
            if not sel:
                return
            dev = devices_found[sel[0]]
            self.entry_device_name.delete(0, tk.END)
            self.entry_device_name.insert(0, dev.name or "EuroBot")
            self.entry_mac.delete(0, tk.END)
            self.entry_mac.insert(0, dev.address)
            self._log(f"📶 Appareil sélectionné : {dev.name} [{dev.address}]", "info")
            top.destroy()

        listbox.bind("<Double-Button-1>", lambda e: _select())
        threading.Thread(target=do_scan, daemon=True).start()
    
    def _on_connect(self):
        """Établir la connexion BLE"""
        # [WiFi] ip = self.entry_ip.get(); port = int(self.entry_port.get())
        # [WiFi] self.client = WiFiClient(host=ip, port=port, ...)
        device_name = self.entry_device_name.get().strip() or "EuroBot"
        mac_address = self.entry_mac.get().strip() or None

        if mac_address:
            self._log(f"📶 Connexion directe à [{mac_address}]...", "info")
        else:
            self._log(f"📶 Scan + connexion au device '{device_name}'...", "info")

        self.btn_connect.config(state="disabled")
        self.btn_scan.config(state="disabled")
        self.label_status.config(text="⏳ Connexion...", foreground="orange")
        self.root.update()

        self.client = BLEClient(
            device_name=device_name,
            device_address=mac_address,
            on_message=self._on_message,
            on_connection=self._on_connection_changed,
            on_error=self._on_ble_error,
            scan_timeout=8.0,
        )

        def _do_connect():
            success = self.client.connect()
            self.root.after(0, lambda: self._post_connect(success, device_name, mac_address))

        threading.Thread(target=_do_connect, daemon=True).start()

    def _post_connect(self, success: bool, device_name: str, mac_address):
        """Mise à jour UI après tentative de connexion BLE"""
        self.btn_scan.config(state="normal")
        if success:
            addr_str = mac_address or "(via scan)"
            self._log(f"✓ Connecté BLE à '{device_name}' {addr_str}", "success")
            self._update_connection_state(True)
        else:
            self._log(f"✗ Connexion BLE échouée pour '{device_name}'", "error")
            self.btn_connect.config(state="normal")
    
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
    
    # [WiFi] def _on_wifi_error(self, error): ...
    def _on_ble_error(self, error):
        """Callback erreur BLE"""
        self._log(f"⚠️ Erreur BLE: {error}", "error")
        self._update_connection_state(False)
    
    def _update_connection_state(self, connected):
        """Mettre à jour l'état de l'UI selon la connexion"""
        self.connected = connected
        
        # [WiFi] self.entry_ip / self.entry_port  →  self.entry_device_name / self.entry_mac
        if connected:
            self.label_status.config(text="✅ Connecté BLE", foreground="green")
            self.label_module.config(text="Module: 🤖 EuroBot BLE", foreground="green")

            self.btn_connect.config(state="disabled")
            self.btn_scan.config(state="disabled")
            self.btn_disconnect.config(state="normal")
            self.btn_led_on.config(state="normal")
            self.btn_led_off.config(state="normal")
            self.btn_led_blink.config(state="normal")
            self.btn_status.config(state="normal")
            self.btn_reset.config(state="normal")
            self.btn_fsm_ready.config(state="normal")
            self.btn_fsm_launch.config(state="normal")
            self.btn_fsm_emergency.config(state="normal")
            self.entry_device_name.config(state="disabled")
            self.entry_mac.config(state="disabled")
        else:
            self.label_status.config(text="❌ Déconnecté", foreground="red")
            self.label_module.config(text="Module: Non détecté", foreground="gray")

            self.btn_connect.config(state="normal")
            self.btn_scan.config(state="normal")
            self.btn_disconnect.config(state="disabled")
            self.btn_led_on.config(state="disabled")
            self.btn_led_off.config(state="disabled")
            self.btn_led_blink.config(state="disabled")
            self.btn_status.config(state="disabled")
            self.btn_reset.config(state="disabled")
            self.btn_fsm_ready.config(state="disabled")
            self.btn_fsm_launch.config(state="disabled")
            self.btn_fsm_emergency.config(state="disabled")
            self.entry_device_name.config(state="normal")
            self.entry_mac.config(state="normal")
    

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
