from __future__ import annotations

import logging
import threading
import time
from typing import Dict, Optional

from pythonosc.udp_client import SimpleUDPClient

from ..osc_transport import SharedOscServer

_log = logging.getLogger(__name__)

GLOVE_OSC_PORT = 4040
GLOVE_CLIENT_PORT = 4042
GLOVE_TARGET_IP = "127.0.0.1"
HEARTBEAT_INTERVAL_SEC = 1.0


class GloveReceiverBase:

    def __init__(
        self,
        glove_port: int = GLOVE_OSC_PORT,
        listen_ip: str = "0.0.0.0",
        target_ip: str = GLOVE_TARGET_IP,
        client_port: int = GLOVE_CLIENT_PORT,
    ):
        self._srv = SharedOscServer.get(glove_port, listen_ip)
        self._lock = threading.Lock()
        self._state: Dict[str, dict] = {"left": {}, "right": {}}
        self._connected: Dict[str, bool] = {"left": False, "right": False}

        self._target_ip = target_ip
        self._client_port = client_port
        self._udp_client: Optional[SimpleUDPClient] = None

        self._running = False
        self._heartbeat_thread: Optional[threading.Thread] = None

        self._srv.add_handler("/device/status/get", self._on_device_status)

    def start(self) -> None:
        if self._udp_client is None:
            self._udp_client = SimpleUDPClient(self._target_ip, self._client_port)
        self._srv.start()
        if not self._running:
            self._running = True
            self._heartbeat_thread = threading.Thread(
                target=self._heartbeat_loop, daemon=True, name="GloveHeartbeat"
            )
            self._heartbeat_thread.start()

    def stop(self) -> None:
        self._running = False
        self._srv.stop()

    def connected(self, side: str) -> bool:
        with self._lock:
            return self._connected[side]

    def _on_device_status(self, address, *args):
        if len(args) < 3:
            return
        with self._lock:
            self._connected["left"] = bool(args[1])
            self._connected["right"] = bool(args[2])

    def _heartbeat_loop(self):
        while self._running:
            try:
                if self._udp_client is not None:
                    self._udp_client.send_message("/device/status/get", "4")
            except Exception as e:
                _log.warning("[Glove] heartbeat 실패: %s", e)
            time.sleep(HEARTBEAT_INTERVAL_SEC)

    @staticmethod
    def _split(args):
        side = args[0]
        if isinstance(side, (list, tuple)):
            side = side[0]
        return side, args[1:]
