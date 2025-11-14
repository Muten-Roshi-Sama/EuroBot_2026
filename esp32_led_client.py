#!/usr/bin/env python3
"""
Simple ESP32 LED TCP client.

Usage:
  python esp32_led_client.py <ESP32_IP>            # interactive mode
  python esp32_led_client.py <ESP32_IP> <COMMAND>  # single command

Commands:
  LED_ON            Turn LED on
  LED_OFF           Turn LED off
  LED_BLINK [ms]    Start non-blocking blink; optional interval in ms (>=50)

Examples:
  python esp32_led_client.py 192.168.4.1
  python esp32_led_client.py 192.168.4.1 "LED_BLINK 200"
"""

import socket
import sys

# Configuration
ESP32_PORT = 8080
TIMEOUT = 5


def send_command(esp32_ip: str, command: str) -> bool:
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(TIMEOUT)
        print(f"[*] Connecting to {esp32_ip}:{ESP32_PORT}...")
        sock.connect((esp32_ip, ESP32_PORT))
        print("[+] Connected")

        print(f"[*] Sending: {command}")
        sock.sendall((command + "\n").encode())

        try:
            resp = sock.recv(1024).decode().strip()
            if resp:
                print(f"[+] Response: {resp}")
        except socket.timeout:
            print("[!] No response received (timeout)")

        sock.close()
        return True
    except socket.timeout:
        print("[-] Connection timeout")
        return False
    except ConnectionRefusedError:
        print("[-] Connection refused - check IP/port and that ESP server is running")
        return False
    except Exception as e:
        print(f"[-] Error: {e}")
        return False


def normalize_command(raw: str) -> str:
    parts = raw.strip().split()
    if not parts:
        return ""
    name = parts[0].upper()
    if name == "LED_BLINK":
        if len(parts) > 1:
            try:
                ms = int(parts[1])
                if ms < 50:
                    ms = 50
                return f"LED_BLINK {ms}"
            except ValueError:
                return "LED_BLINK"
        return "LED_BLINK"
    if name in ("LED_ON", "LED_OFF"):
        return name
    return ""


def interactive_mode(esp32_ip: str):
    print(f"Interactive mode — target {esp32_ip}:{ESP32_PORT}")
    print("Commands: LED_ON, LED_OFF, LED_BLINK [ms], quit")
    while True:
        try:
            raw = input("> ").strip()
        except (EOFError, KeyboardInterrupt):
            print("\nExiting")
            break
        if not raw:
            continue
        if raw.lower() in ("quit", "exit"):
            break
        cmd = normalize_command(raw)
        if not cmd:
            print("Invalid command")
            continue
        send_command(esp32_ip, cmd)


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    esp32_ip = sys.argv[1]
    if len(sys.argv) == 2:
        interactive_mode(esp32_ip)
    else:
        raw = " ".join(sys.argv[2:])
        cmd = normalize_command(raw)
        if not cmd:
            print("Invalid command")
            sys.exit(2)
        send_command(esp32_ip, cmd)


if __name__ == "__main__":
    main()
