"""
Client BLE pour communication avec ESP32 (Nordic UART Service)
Remplace WiFiClient pour la communication Bluetooth Low Energy.

UUIDs (mêmes que dans bluetooth.cpp):
  SERVICE_UUID : 6E400001-B5A3-F393-E0A9-E50E24DCCA9E
  RX_CHAR_UUID : 6E400002-B5A3-F393-E0A9-E50E24DCCA9E  (PC → ESP32, write)
  TX_CHAR_UUID : 6E400003-B5A3-F393-E0A9-E50E24DCCA9E  (ESP32 → PC, notify)
"""

import asyncio
import threading
from typing import Callable, Optional, List

from bleak import BleakClient, BleakScanner
from bleak.backends.device import BLEDevice
from protocol import parse_message

# Nordic UART Service UUIDs (identiques à bluetooth.cpp)
SERVICE_UUID  = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
RX_CHAR_UUID  = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"  # write  (PC → ESP32)
TX_CHAR_UUID  = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"  # notify (ESP32 → PC)


class BLEClient:
    """
    Client BLE synchrone (API calquée sur WiFiClient).
    Lance un event-loop asyncio dans un thread dédié.
    """

    def __init__(
        self,
        device_name: str = "EuroBot",
        device_address: Optional[str] = None,
        on_message: Optional[Callable] = None,
        on_connection: Optional[Callable] = None,
        on_error: Optional[Callable] = None,
        scan_timeout: float = 8.0,
    ):
        """
        Args:
            device_name    : Nom BLE à rechercher lors du scan (si device_address absent)
            device_address : Adresse MAC directe (bypass le scan si fournie)
            on_message     : Callback réception -> fn(dict_response)
            on_connection  : Callback changement connexion -> fn(bool_connected)
            on_error       : Callback erreur    -> fn(str_error)
            scan_timeout   : Durée max du scan en secondes
        """
        self.device_name    = device_name
        self.device_address = device_address
        self.scan_timeout   = scan_timeout

        self.on_message    = on_message
        self.on_connection = on_connection
        self.on_error      = on_error

        self._client: Optional[BleakClient] = None
        self._connected = False
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._loop_thread: Optional[threading.Thread] = None
        self._buffer = ""

        # Démarrer le loop asyncio dans un thread daemon
        self._start_loop()

    # ------------------------------------------------------------------
    # Gestion du loop asyncio
    # ------------------------------------------------------------------

    def _start_loop(self):
        """Lance l'event-loop asyncio dans un thread dédié."""
        self._loop = asyncio.new_event_loop()
        self._loop_thread = threading.Thread(
            target=self._loop.run_forever, daemon=True, name="BLE-AsyncLoop"
        )
        self._loop_thread.start()

    def _run(self, coro):
        """Soumet une coroutine au loop et attend le résultat (bloquant)."""
        future = asyncio.run_coroutine_threadsafe(coro, self._loop)
        return future.result(timeout=self.scan_timeout + 5)

    # ------------------------------------------------------------------
    # API publique (synchrone, même interface que WiFiClient)
    # ------------------------------------------------------------------

    def scan(self) -> List[BLEDevice]:
        """
        Scanner les appareils BLE disponibles.
        Returns:
            Liste de BLEDevice trouvés
        """
        return self._run(self._async_scan())

    def connect(self) -> bool:
        """
        Se connecter à l'ESP32.
        Si device_address est défini, connexion directe; sinon scan par nom.
        Returns:
            True si succès
        """
        return self._run(self._async_connect())

    def disconnect(self):
        """Fermer la connexion BLE."""
        if self._connected and self._client:
            future = asyncio.run_coroutine_threadsafe(
                self._async_disconnect(), self._loop
            )
            try:
                future.result(timeout=5)
            except Exception:
                pass
        self._connected = False
        if self.on_connection:
            self.on_connection(False)

    def send(self, message: str) -> bool:
        """
        Envoyer une commande JSON à l'ESP32 (écriture sur RX_CHAR).
        Args:
            message: String JSON (avec ou sans '\\n' final)
        Returns:
            True si succès
        """
        if not self._connected or not self._client:
            if self.on_error:
                self.on_error("Non connecté")
            return False
        try:
            future = asyncio.run_coroutine_threadsafe(
                self._async_send(message), self._loop
            )
            return future.result(timeout=5)
        except Exception as e:
            if self.on_error:
                self.on_error(f"Erreur envoi: {e}")
            return False

    def is_connected(self) -> bool:
        return self._connected

    # ------------------------------------------------------------------
    # Implémentation async interne
    # ------------------------------------------------------------------

    async def _async_scan(self) -> List[BLEDevice]:
        """Effectue un scan BLE et retourne la liste des appareils."""
        devices = await BleakScanner.discover(timeout=self.scan_timeout)
        return devices

    async def _async_connect(self) -> bool:
        try:
            # Résoudre l'adresse si non fournie
            address = self.device_address
            if not address:
                address = await self._find_device_address()
                if not address:
                    if self.on_error:
                        self.on_error(
                            f"Appareil '{self.device_name}' introuvable (scan {self.scan_timeout}s)"
                        )
                    return False

            self._client = BleakClient(
                address,
                disconnected_callback=self._on_disconnect_callback,
            )
            await self._client.connect()
            self._connected = True

            # S'abonner aux notifications TX (ESP32 → PC)
            await self._client.start_notify(TX_CHAR_UUID, self._on_notify)

            if self.on_connection:
                self.on_connection(True)
            return True

        except Exception as e:
            self._connected = False
            if self.on_error:
                self.on_error(f"Connexion BLE échouée: {e}")
            return False

    async def _async_disconnect(self):
        if self._client and self._client.is_connected:
            try:
                await self._client.stop_notify(TX_CHAR_UUID)
            except Exception:
                pass
            await self._client.disconnect()
        self._connected = False

    async def _async_send(self, message: str) -> bool:
        if not message.endswith("\n"):
            message += "\n"
        data = message.encode("utf-8")
        await self._client.write_gatt_char(RX_CHAR_UUID, data, response=False)
        return True

    async def _find_device_address(self) -> Optional[str]:
        """Scan et retourne l'adresse du premier appareil ayant le bon nom."""
        devices = await BleakScanner.discover(timeout=self.scan_timeout)
        for d in devices:
            if d.name and self.device_name.lower() in d.name.lower():
                return d.address
        return None

    # ------------------------------------------------------------------
    # Callbacks BLE internes
    # ------------------------------------------------------------------

    def _on_notify(self, sender, data: bytearray):
        """Appelé à chaque notification reçue de l'ESP32."""
        self._buffer += data.decode("utf-8", errors="ignore")
        # Traiter les messages ligne par ligne
        while "\n" in self._buffer:
            line, self._buffer = self._buffer.split("\n", 1)
            line = line.strip()
            if line:
                parsed = parse_message(line)
                if self.on_message:
                    self.on_message(parsed)

    def _on_disconnect_callback(self, client: BleakClient):
        """Appelé par bleak quand la connexion est perdue de façon inattendue."""
        self._connected = False
        if self.on_error:
            self.on_error("Connexion BLE perdue")
        if self.on_connection:
            self.on_connection(False)
