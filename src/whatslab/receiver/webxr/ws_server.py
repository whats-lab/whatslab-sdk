from __future__ import annotations

import json
import logging
import os
import socket
import ssl
import threading
from http import HTTPStatus
from typing import Callable, Optional

from websockets.datastructures import Headers
from websockets.http11 import Request, Response
from websockets.sync.server import Server, ServerConnection, serve

from .tls import ensure_cert, preferred_ip

_log = logging.getLogger(__name__)

_STATIC_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "static")
_PAGE_PATHS = ("/", "/index.html")


def _serve_page(connection: ServerConnection, request: Request) -> Optional[Response]:
    if request.headers.get("Upgrade", "").lower() == "websocket":
        return None
    if request.path.split("?", 1)[0] not in _PAGE_PATHS:
        return connection.respond(HTTPStatus.NOT_FOUND, "")
    with open(os.path.join(_STATIC_DIR, "index.html"), "rb") as fp:
        body = fp.read()
    headers = Headers({"Content-Type": "text/html; charset=utf-8",
                       "Content-Length": str(len(body)), "Cache-Control": "no-store"})
    return Response(HTTPStatus.OK, "OK", headers, body)


class WebXRServer:

    def __init__(self, handler: Callable[[dict], None], port: int, listen_ip: str = "0.0.0.0",
                 tls: bool = True, certfile: str = None, keyfile: str = None):
        self._handler = handler
        self._port = port
        self._listen_ip = listen_ip
        self._tls = tls or bool(certfile)
        self._certfile = certfile
        self._keyfile = keyfile
        self._lock = threading.Lock()
        self._server: Optional[Server] = None
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()

    @property
    def scheme(self) -> str:
        return "https" if self._tls else "http"

    @property
    def port(self) -> int:
        return self._port

    @property
    def is_running(self) -> bool:
        with self._lock:
            return self._server is not None

    def start(self) -> None:
        with self._lock:
            if self._server is not None:
                return
            ctx = None
            if self._tls:
                cert, key = ((self._certfile, self._keyfile) if self._certfile
                             else ensure_cert())
                ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
                ctx.load_cert_chain(cert, key)
            sock = socket.create_server((self._listen_ip, self._port))
            self._port = sock.getsockname()[1]
            stop = threading.Event()
            self._server = serve(lambda ws: self._serve(ws, stop), sock=sock, ssl=ctx,
                                 process_request=_serve_page)
            self._stop = stop
            self._thread = threading.Thread(target=self._server.serve_forever, daemon=True,
                                            name=f"webxr-{self._port}")
            self._thread.start()
            _log.info("WebXR 서버 %s://%s:%d", self.scheme,
                      preferred_ip() if self._tls else "localhost", self._port)

    def stop(self) -> None:
        with self._lock:
            server, thread = self._server, self._thread
            self._server = self._thread = None
            if server is None:
                return
            self._stop.set()
        server.shutdown()
        thread.join(timeout=2.0)

    def _serve(self, ws: ServerConnection, stop: threading.Event) -> None:
        for raw in ws:
            if stop.is_set():
                return
            self._handler(json.loads(raw))
