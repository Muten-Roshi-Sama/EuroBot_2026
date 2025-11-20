import tkinter as tk
from tkinter import simpledialog, scrolledtext
import socket
import threading
import json

HOST = ''
PORT = 8080

clients = {}  # addr -> conn
clients_lock = threading.Lock()

class ServerGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("ESP32 Multi-Client Server")
        self.text = scrolledtext.ScrolledText(root, width=80, height=20, state='disabled')
        self.text.pack(padx=10, pady=10)
        self.client_var = tk.StringVar()
        self.client_menu = tk.OptionMenu(root, self.client_var, "")
        self.client_menu.pack()
        self.cmd_entry = tk.Entry(root, width=60)
        self.cmd_entry.pack(side=tk.LEFT, padx=5)
        self.send_btn = tk.Button(root, text="Envoyer", command=self.send_command)
        self.send_btn.pack(side=tk.LEFT, padx=5)
        self.update_clients_menu()
        self.root.after(1000, self.update_clients_menu)

    def log(self, msg):
        self.text.config(state='normal')
        self.text.insert(tk.END, msg + '\n')
        self.text.see(tk.END)
        self.text.config(state='disabled')

    def update_clients_menu(self):
        with clients_lock:
            menu = self.client_menu['menu']
            menu.delete(0, 'end')
            for addr in clients.keys():
                menu.add_command(label=f"{addr[0]}:{addr[1]}", command=lambda a=addr: self.client_var.set(f"{a[0]}:{a[1]}"))
            if clients:
                if not self.client_var.get() or self.client_var.get() not in [f"{a[0]}:{a[1]}" for a in clients.keys()]:
                    first = next(iter(clients.keys()))
                    self.client_var.set(f"{first[0]}:{first[1]}")
            else:
                self.client_var.set("")
        self.root.after(1000, self.update_clients_menu)

    def send_command(self):
        ip_port = self.client_var.get()
        cmd = self.cmd_entry.get()
        if not ip_port or not cmd:
            self.log("[!] Sélectionnez un client et entrez une commande JSON.")
            return
        with clients_lock:
            for addr, conn in clients.items():
                if f"{addr[0]}:{addr[1]}" == ip_port:
                    try:
                        json.loads(cmd)
                        conn.sendall((cmd + '\n').encode())
                        self.log(f"[-> {ip_port}] {cmd}")
                        return
                    except Exception as e:
                        self.log(f"[!] Erreur d'envoi à {ip_port}: {e}")
                        return
        self.log(f"[!] Aucun client connecté à l'adresse {ip_port}")

def handle_client(conn, addr, gui):
    gui.log(f"[+] Connecté par {addr}")
    with clients_lock:
        clients[addr] = conn
    try:
        while True:
            data = conn.recv(1024)
            if not data:
                gui.log(f'[-] Déconnecté de {addr}.')
                break
            try:
                response = json.loads(data.decode().strip())
                gui.log(f'[Réponse {addr}] {json.dumps(response, ensure_ascii=False)}')
            except Exception as e:
                gui.log(f'[Réponse {addr}] (non-JSON): {data.decode().strip()}')
    except Exception as e:
        gui.log(f"[!] Erreur avec {addr}: {e}")
    finally:
        with clients_lock:
            if addr in clients:
                del clients[addr]
        conn.close()

def server_thread(gui):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((HOST, PORT))
        s.listen()
        gui.log(f"Serveur multi-clients en attente sur le port {PORT}...")
        while True:
            conn, addr = s.accept()
            t = threading.Thread(target=handle_client, args=(conn, addr, gui), daemon=True)
            t.start()

def main():
    root = tk.Tk()
    gui = ServerGUI(root)
    threading.Thread(target=server_thread, args=(gui,), daemon=True).start()
    root.mainloop()

if __name__ == "__main__":
    main()
