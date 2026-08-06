from __future__ import annotations

import logging
import threading
from typing import Any, Callable, Dict

from pythonosc import dispatcher as osc_dispatcher
from pythonosc import osc_server

_log = logging.getLogger(__name__)

_registry: Dict[int, "SharedOscServer"] = {}
_registry_lock = threading.Lock()


class SharedOscServer:

    def __init__(self, listen_ip: str, port: int):
        self._listen_ip = listen_ip
        self._port = port
        self.dispatcher = osc_dispatcher.Dispatcher()

        self._lock = threading.Lock()
        self._refcount = 0
        self._server: osc_server.ThreadingOSCUDPServer | None = None
        self._thread: threading.Thread | None = None

    # --------------------------------------------------------------- registry
    @classmethod
    def get(cls, port: int, listen_ip: str = "0.0.0.0") -> "SharedOscServer":
        with _registry_lock:
            srv = _registry.get(port)
            if srv is None:
                srv = cls(listen_ip, port)
                _registry[port] = srv
            return srv

    # ---------------------------------------------------------------- handler
    def add_handler(self, address: str, fn: Callable[..., Any], *args: Any) -> None:
        self.dispatcher.map(address, fn, *args)

    # ------------------------------------------------------------- lifecycle
    @property
    def is_running(self) -> bool:
        with self._lock:
            return self._server is not None

    def start(self) -> None:
        with self._lock:
            self._refcount += 1
            if self._server is not None:
                return
            # SO_REUSEADDR — 직전 서버 소켓 TIME_WAIT 중에도 같은 포트 재바인드 허용
            # (빠른 stop→start 재시작/테스트에서 "Address already in use" 방지).
            osc_server.ThreadingOSCUDPServer.allow_reuse_address = True
            self._server = osc_server.ThreadingOSCUDPServer(
                (self._listen_ip, self._port), self.dispatcher
            )
            self._thread = threading.Thread(
                target=self._server.serve_forever, daemon=True,
                name=f"SharedOscServer:{self._port}",
            )
            self._thread.start()
            _log.info("[SharedOscServer] %s:%d 대기 시작", self._listen_ip, self._port)

    def stop(self) -> None:
        with self._lock:
            if self._refcount <= 0:
                return
            self._refcount -= 1
            if self._refcount > 0:
                return
            if self._server is not None:
                try:
                    self._server.shutdown()
                    self._server.server_close()
                except Exception:
                    pass
                _log.info("[SharedOscServer] %s:%d 종료", self._listen_ip, self._port)
            self._server = None
            self._thread = None
