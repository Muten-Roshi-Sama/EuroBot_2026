#!/usr/bin/env python3
"""
ESP32-S LED WiFi Control Client
================================
Controls an LED on ESP32-S over WiFi using TCP socket.

Usage:
    python esp32_led_client.py <ESP32_IP> <COMMAND>

Commands:
    LED_ON       - Turn LED on
    LED_OFF      - Turn LED off
    LED_TOGGLE   - Toggle LED state
    LED_STATUS   - Check LED status

Example:
    python esp32_led_client.py 192.168.1.100 LED_ON
"""

import socket
import sys
import time

# Configuration
ESP32_PORT = 8080
TIMEOUT = 5  # seconds

def send_command(esp32_ip, command):
    """Send command to ESP32 and receive response."""
    try:
        # Créer socket TCP
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(TIMEOUT)
        
        print(f"[*] Connecting to {esp32_ip}:{ESP32_PORT}...")
        sock.connect((esp32_ip, ESP32_PORT))
        print("[+] Connected!")
        
        # Envoyer commande
        print(f"[*] Sending command: {command}")
        sock.sendall(f"{command}\n".encode())
        
        # Recevoir réponse
        response = sock.recv(1024).decode().strip()
        print(f"[+] Response: {response}")
        
        sock.close()
        return True
        
    except socket.timeout:
        print("[-] Connection timeout!")
        return False
    except ConnectionRefusedError:
        print("[-] Connection refused! Make sure ESP32 is running and IP/port are correct.")
        return False
    except Exception as e:
        print(f"[-] Error: {e}")
        return False

def interactive_mode(esp32_ip):
    """Interactive command mode."""
    print(f"[*] ESP32-S LED Controller - Interactive Mode")
    print(f"[*] Connected to: {esp32_ip}:{ESP32_PORT}")
    print("[*] Commands: LED_ON, LED_OFF, LED_TOGGLE, LED_STATUS, quit")
    print()
    
    while True:
        try:
            cmd = input("Enter command > ").strip().upper()
            
            if cmd == "QUIT":
                print("[*] Exiting...")
                break
            elif cmd in ["LED_ON", "LED_OFF", "LED_TOGGLE", "LED_STATUS"]:
                send_command(esp32_ip, cmd)
            else:
                print("[-] Invalid command!")
            print()
        except KeyboardInterrupt:
            print("\n[*] Interrupted by user")
            break

def main():
    if len(sys.argv) < 2:
        print("Usage:")
        print("  python esp32_led_client.py <ESP32_IP>           # Interactive mode")
        print("  python esp32_led_client.py <ESP32_IP> <COMMAND> # Send single command")
        print()
        print("Commands: LED_ON, LED_OFF, LED_TOGGLE, LED_STATUS")
        print()
        print("Examples:")
        print("  python esp32_led_client.py 192.168.1.100")
        print("  python esp32_led_client.py 192.168.1.100 LED_ON")
        sys.exit(1)
    
    esp32_ip = sys.argv[1]
    
    if len(sys.argv) == 2:
        # Interactive mode
        interactive_mode(esp32_ip)
    else:
        # Single command mode
        command = sys.argv[2]
        send_command(esp32_ip, command)

if __name__ == "__main__":
    main()
