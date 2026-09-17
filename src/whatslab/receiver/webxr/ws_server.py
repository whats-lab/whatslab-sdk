from __future__ import annotations

import base64
import errno
import hashlib
import json
import logging
import os
import socket
import ssl
import struct
import threading
import time
from typing import Any, Callable, Dict, Optional, Tuple

from .tls import ensure_cert, preferred_ip

_log = logging.getLogger(__name__)

_GUID = b"258EAFA5-E914-47DA-95CA-C5AB0DC85B11"
_STATIC_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "static")

IDLE_PING_S = 5.0
IDLE_DROP_S = 20.0

OP_CONT = 0x0
OP_TEXT = 0x1
OP_BINARY = 0x2
OP_CLOSE = 0x8
OP_PING = 0x9
OP_PONG = 0xA

_registry: Dict[int, "WebXRServer"] = {}
_registry_lock = threading.Lock()


class ProtocolError(RuntimeError):
    pass


def accept_key(client_key: str) -> str:
    digest = hashlib.sha1(client_key.strip().encode("ascii") + _GUID).digest()
    return base64.b64encode(digest).decode("ascii")


def encode_frame(payload: bytes, opcode: int = OP_TEXT) -> bytes:
    n = len(payload)
    head = bytes([0x80 | opcode])
    if n < 126:
        head += bytes([n])
    elif n < 65536:
        head += bytes([126]) + struct.pack(">H", n)
    else:
        head += bytes([127]) + struct.pack(">Q", n)
    return head + payload


class _FrameReader:

    def __init__(self, sock: socket.socket, max_payload: int = 1 << 20):
        self._sock = sock
        self._buf = bytearray()
        self._max = max_payload

    def _need(self, n: int) -> bytes:
        while len(self._buf) < n:
            chunk = self._sock.recv(4096)
            if not chunk:
                raise ConnectionError("peer closed")
            self._buf.extend(chunk)
        out = bytes(self._buf[:n])
        del self._buf[:n]
        return out

    def read_frame(self) -> Tuple[int, bytes]:
        b0, b1 = self._need(2)
        fin = b0 & 0x80
        opcode = b0 & 0x0F
        masked = b1 & 0x80
        length = b1 & 0x7F
        if length == 126:
            length = struct.unpack(">H", self._need(2))[0]
        elif length == 127:
            length = struct.unpack(">Q", self._need(8))[0]
        if length > self._max:
            raise ProtocolError(f"payload {length} > {self._max}")
        if not masked:
            raise ProtocolError("client frame must be masked")
        mask = self._need(4)
        data = bytearray(self._need(length))
        for i in range(length):
            data[i] ^= mask[i & 3]
        if not fin:
            raise ProtocolError("fragmented frames are not supported")
        return opcode, bytes(data)


class WebXRServer:

    def __init__(self, port: int, listen_ip: str = "0.0.0.0",
                 tls: bool = True, certfile: str = None, keyfile: str = None):
        self._port = port
        self._listen_ip = listen_ip
        self._tls = tls or bool(certfile)
        self._certfile = certfile
        self._keyfile = keyfile
        self._ssl_ctx: Optional[ssl.SSLContext] = None
        self._on_message: Optional[Callable[[dict], None]] = None
        self._lock = threading.Lock()
        self._refcount = 0
        self._sock: Optional[socket.socket] = None
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._clients = 0
        self._frames = 0
        self._bad = 0

    @classmethod
    def get(cls, port: int, listen_ip: str = "0.0.0.0", tls: bool = True,
            certfile: str = None, keyfile: str = None) -> "WebXRServer":
        with _registry_lock:
            srv = _registry.get(port)
            if srv is None:
                srv = cls(port, listen_ip, tls, certfile, keyfile)
                _registry[port] = srv
            return srv

    @property
    def scheme(self) -> str:
        return "https" if self._tls else "http"

    def set_handler(self, fn: Callable[[dict], None]) -> None:
        self._on_message = fn

    @property
    def port(self) -> int:
        return self._port

    @property
    def is_running(self) -> bool:
        with self._lock:
            return self._sock is not None

    @property
    def stats(self) -> Tuple[int, int, int]:
        return self._clients, self._frames, self._bad

    def start(self) -> None:
        with self._lock:
            self._refcount += 1
            if self._sock is not None:
                return
            self._stop.clear()
            if self._tls and self._ssl_ctx is None:
                cert, key = ((self._certfile, self._keyfile) if self._certfile
                             else ensure_cert())
                ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
                ctx.load_cert_chain(cert, key)
                self._ssl_ctx = ctx
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            try:
                sock.bind((self._listen_ip, self._port))
            except OSError as exc:
                sock.close()
                if exc.errno == errno.EADDRINUSE:
                    raise OSError(
                        f"포트 {self._port} 를 이미 다른 프로세스가 쓰고 있다 — "
                        f"`ss -ltnp | grep {self._port}` 로 확인하고 끄거나, "
                        f"다른 포트를 지정해라") from exc
                raise
            sock.listen(4)
            sock.settimeout(0.5)
            self._sock = sock
            if self._port == 0:
                self._port = sock.getsockname()[1]
            self._thread = threading.Thread(target=self._accept_loop, daemon=True,
                                            name=f"webxr-{self._port}")
            self._thread.start()
            _log.info("WebXR 서버 %s://%s:%d", self.scheme,
                      preferred_ip() if self._tls else "localhost", self._port)

    def stop(self) -> None:
        with self._lock:
            if self._refcount > 0:
                self._refcount -= 1
            if self._refcount > 0 or self._sock is None:
                return
            self._stop.set()
            sock, thread = self._sock, self._thread
            self._sock = self._thread = None
        try:
            sock.close()
        except OSError:
            pass
        if thread is not None:
            thread.join(timeout=2.0)

    def _accept_loop(self) -> None:
        while not self._stop.is_set():
            sock = self._sock
            if sock is None:
                break
            try:
                conn, _ = sock.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            if self._ssl_ctx is not None:
                try:
                    conn.settimeout(10.0)
                    conn = self._ssl_ctx.wrap_socket(conn, server_side=True)
                except (ssl.SSLError, OSError) as exc:
                    self._bad += 1
                    _log.debug("TLS 핸드셰이크 실패: %r", exc)
                    try:
                        conn.close()
                    except OSError:
                        pass
                    continue
            threading.Thread(target=self._serve, args=(conn,), daemon=True,
                             name="webxr-conn").start()

    def _serve(self, conn: socket.socket) -> None:
        try:
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            conn.settimeout(10.0)
            head = self._read_head(conn)
            if head is None:
                return
            request, headers = head
            if headers.get("upgrade", "").lower() == "websocket":
                self._handshake(conn, headers)
                self._pump(conn)
            else:
                self._serve_static(conn, request)
        except (ConnectionError, OSError, ProtocolError) as exc:
            _log.debug("webxr 연결 종료: %r", exc)
        finally:
            try:
                conn.close()
            except OSError:
                pass

    @staticmethod
    def _read_head(conn: socket.socket):
        buf = bytearray()
        while b"\r\n\r\n" not in buf:
            chunk = conn.recv(4096)
            if not chunk:
                return None
            buf.extend(chunk)
            if len(buf) > 65536:
                raise ProtocolError("request head too large")
        text = bytes(buf).split(b"\r\n\r\n", 1)[0].decode("latin-1")
        lines = text.split("\r\n")
        headers = {}
        for line in lines[1:]:
            if ":" in line:
                k, v = line.split(":", 1)
                headers[k.strip().lower()] = v.strip()
        return lines[0], headers

    @staticmethod
    def _handshake(conn: socket.socket, headers: Dict[str, str]) -> None:
        key = headers.get("sec-websocket-key")
        if not key:
            raise ProtocolError("Sec-WebSocket-Key 없음")
        resp = ("HTTP/1.1 101 Switching Protocols\r\n"
                "Upgrade: websocket\r\n"
                "Connection: Upgrade\r\n"
                f"Sec-WebSocket-Accept: {accept_key(key)}\r\n\r\n")
        conn.sendall(resp.encode("ascii"))

    def _pump(self, conn: socket.socket) -> None:
        conn.settimeout(IDLE_PING_S)
        reader = _FrameReader(conn)
        self._clients += 1
        last = time.monotonic()
        try:
            while not self._stop.is_set():
                try:
                    opcode, data = reader.read_frame()
                except socket.timeout:
                    if time.monotonic() - last > IDLE_DROP_S:
                        _log.info("무응답 %.0fs — 연결을 끊는다", IDLE_DROP_S)
                        return
                    conn.sendall(encode_frame(b"", OP_PING))
                    continue
                last = time.monotonic()
                if opcode == OP_CLOSE:
                    conn.sendall(encode_frame(b"", OP_CLOSE))
                    return
                if opcode == OP_PING:
                    conn.sendall(encode_frame(data, OP_PONG))
                    continue
                if opcode == OP_PONG:
                    continue
                if opcode not in (OP_TEXT, OP_BINARY):
                    continue
                self._dispatch(data)
        finally:
            self._clients -= 1

    def _dispatch(self, data: bytes) -> None:
        try:
            msg = json.loads(data.decode("utf-8"))
        except (UnicodeDecodeError, ValueError):
            self._bad += 1
            return
        if not isinstance(msg, dict):
            self._bad += 1
            return
        self._frames += 1
        fn = self._on_message
        if fn is not None:
            fn(msg)

    def _serve_static(self, conn: socket.socket, request: str) -> None:
        parts = request.split()
        path = parts[1] if len(parts) > 1 else "/"
        path = path.split("?", 1)[0]
        name = "index.html" if path in ("/", "") else os.path.basename(path)
        full = os.path.join(_STATIC_DIR, name)
        if not os.path.isfile(full):
            conn.sendall(b"HTTP/1.1 404 Not Found\r\nContent-Length: 0\r\n\r\n")
            return
        with open(full, "rb") as fp:
            body = fp.read()
        ctype = "text/html; charset=utf-8" if name.endswith(".html") else "text/plain"
        conn.sendall((f"HTTP/1.1 200 OK\r\nContent-Type: {ctype}\r\n"
                      f"Content-Length: {len(body)}\r\n"
                      f"Cache-Control: no-store\r\n\r\n").encode("ascii") + body)


def clear_registry() -> None:
    with _registry_lock:
        servers = list(_registry.values())
        _registry.clear()
    for srv in servers:
        srv._refcount = 1
        srv.stop()
