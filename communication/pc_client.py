"""
README (PC client for ESP32-C3 NDJSON TCP bridge)
-------------------------------------------------
Purpose: Connect to ESP32-C3 TCP server (port 3333), read NDJSON lines,
print structured messages, and periodically send command JSON.

Run:
  python3 pc_client.py --host esp32c3.local --port 3333 --key ""

Notes:
- Uses only Python standard library: socket, threading, json, time.
- Automatically reconnects with exponential backoff upon connection loss.

Example outgoing command:
  {"type":"command","command":"set_speed","value":100,"seq":1}
"""

import argparse
import json
import socket
import threading
import time
from typing import Optional


DEFAULT_HOST = "esp32c3.local"  # or static IP
DEFAULT_PORT = 3333
RECONNECT_BASE_SEC = 0.5
RECONNECT_MAX_SEC = 8.0
SEND_PERIOD_SEC = 2.0
SOCKET_TIMEOUT_SEC = 0.5


class LineReader:
    def __init__(self, sock: socket.socket):
        self.sock = sock
        self.buf = bytearray()

    def read_line(self, timeout: float) -> Optional[bytes]:
        self.sock.settimeout(timeout)
        try:
            chunk = self.sock.recv(4096)
            if not chunk:
                return None
            self.buf.extend(chunk)
            nl = self.buf.find(b"\n")
            if nl != -1:
                line = self.buf[:nl]
                del self.buf[: nl + 1]
                return bytes(line)
        except socket.timeout:
            return b""  # indicate timeout (no full line yet)
        return b""


def connect_with_retries(host: str, port: int) -> socket.socket:
    delay = RECONNECT_BASE_SEC
    while True:
        try:
            s = socket.create_connection((host, port), timeout=5.0)
            s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            print(f"# Connected to {host}:{port}")
            return s
        except OSError as e:
            print(f"# Connect failed: {e}; retrying in {delay:.1f}s")
            time.sleep(delay)
            delay = min(delay * 2.0, RECONNECT_MAX_SEC)


def reader_thread(sock: socket.socket, stop_event: threading.Event):
    lr = LineReader(sock)
    while not stop_event.is_set():
        line = lr.read_line(SOCKET_TIMEOUT_SEC)
        if line is None:
            # connection closed
            print("# Connection closed by remote")
            break
        if line == b"":
            continue  # timeout slice, keep looping
        try:
            obj = json.loads(line.decode("utf-8", errors="replace"))
            print(f"RX: {json.dumps(obj, ensure_ascii=False)}")
        except json.JSONDecodeError:
            print(f"# Invalid JSON line: {line!r}")


def writer_thread(sock: socket.socket, stop_event: threading.Event, key: str):
    seq = 1
    while not stop_event.is_set():
        cmd = {
            "type": "command",
            "command": "set_speed",
            "value": 100 + (seq % 50),
            "seq": seq,
        }
        if key:
            cmd["key"] = key
        data = (json.dumps(cmd, ensure_ascii=False) + "\n").encode("utf-8")
        try:
            sock.sendall(data)
            print(f"TX: {json.dumps(cmd, ensure_ascii=False)}")
        except OSError as e:
            print(f"# Send failed: {e}")
            break
        seq += 1
        for _ in range(int(SEND_PERIOD_SEC / 0.1)):
            if stop_event.is_set():
                break
            time.sleep(0.1)


def run_client(host: str, port: int, key: str):
    while True:
        sock = connect_with_retries(host, port)
        stop_event = threading.Event()
        t_r = threading.Thread(target=reader_thread, args=(sock, stop_event), daemon=True)
        t_w = threading.Thread(target=writer_thread, args=(sock, stop_event, key), daemon=True)
        t_r.start()
        t_w.start()
        try:
            while t_r.is_alive() and t_w.is_alive():
                time.sleep(0.2)
        except KeyboardInterrupt:
            print("# KeyboardInterrupt; shutting down...")
            stop_event.set()
        finally:
            stop_event.set()
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            sock.close()
        # reconnect after a short delay
        time.sleep(0.5)


def main():
    parser = argparse.ArgumentParser(description="PC client for ESP32-C3 NDJSON TCP bridge")
    parser.add_argument("--host", default=DEFAULT_HOST, help="ESP32-C3 hostname or IP")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT, help="TCP port (default 3333)")
    parser.add_argument("--key", default="", help="Pre-shared key (if required by bridge)")
    args = parser.parse_args()
    run_client(args.host, args.port, args.key)


if __name__ == "__main__":
    main()


