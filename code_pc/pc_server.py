import socket
import threading
import json

HOST = ''  
PORT = 8080

def handle_client(conn, addr):
    print(f"[+] Connecté par {addr}")
    import socket
    import threading
    import json

    HOST = ''  # écoute sur toutes les interfaces
    PORT = 8080

    clients = {}  # addr -> conn
    clients_lock = threading.Lock()

    def handle_client(conn, addr):
        print(f"[+] Connecté par {addr}")
        with clients_lock:
            clients[addr] = conn
        try:
            while True:
                data = conn.recv(1024)
                if not data:
                    print(f'[-] Déconnecté de {addr}.')
                    break
                try:
                    response = json.loads(data.decode().strip())
                    print(f'[Réponse {addr}]', json.dumps(response, indent=2, ensure_ascii=False))
                except Exception as e:
                    print(f'[Réponse {addr}] (non-JSON):', data.decode().strip())
        except Exception as e:
            print(f"[!] Erreur avec {addr}: {e}")
        finally:
            with clients_lock:
                if addr in clients:
                    del clients[addr]
            conn.close()

    def send_command_to(ip_port, cmd):
        with clients_lock:
            # ip_port doit être sous forme "IP:PORT"
            for addr, conn in clients.items():
                if f"{addr[0]}:{addr[1]}" == ip_port:
                    try:
                        json.loads(cmd)  # Vérifie que c'est du JSON
                        conn.sendall((cmd + '\n').encode())
                        return
                    except Exception as e:
                        print(f"[!] Erreur d'envoi à {ip_port}: {e}")
                        return
            print(f"[!] Aucun client connecté à l'adresse {ip_port}")

    def input_thread():
        import time
        while True:
            with clients_lock:
                if not clients:
                    print("Aucun client connecté. Attente de connexion...")
                    time.sleep(2)
                    continue
                print("\nClients connectés :")
                for addr in clients.keys():
                    print(f"  {addr[0]}:{addr[1]}")
            ip_port = input("\nIP:PORT du client à viser : ").strip()
            if not ip_port:
                continue
            cmd = input(f"Commande JSON à envoyer à {ip_port} : ").strip()
            if not cmd:
                continue
            send_command_to(ip_port, cmd)

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((HOST, PORT))
        s.listen()
        print(f"Serveur multi-clients en attente sur le port {PORT}...")
        # Lancer la saisie utilisateur au premier plan (pas en thread)
        threading.Thread(target=lambda: [handle_client(*s.accept()) for _ in iter(int, 1)], daemon=True).start()
        input_thread()
