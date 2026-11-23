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
from typing import Callable, Dict, List, Optional, Tuple

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
        self._consumed = bytearray()
        self._max = max_payload

    def _need(self, n: int, track: bool = True) -> bytes:
        while len(self._buf) < n:
            chunk = self._sock.recv(4096)
            if not chunk:
                raise ConnectionError("peer closed")
            self._buf.extend(chunk)
        out = bytes(self._buf[:n])
        del self._buf[:n]
        if track:
            self._consumed += out
        return out

    def read_frame(self) -> Tuple[int, bytes]:
        """프레임 하나. 중간에 recv 가 timeout 나면 읽은 바이트를 버퍼에 되돌린다.

        호출자는 timeout 을 유휴로 보고 ping 뒤 다시 부르므로, 반쯤 읽은 헤더를 버리면 다음
        호출이 payload 를 헤더로 읽어 연결이 어긋난다.
        """
        self._consumed = bytearray()
        try:
            return self._read_frame()
        except socket.timeout:
            self._buf[:0] = self._consumed
            raise

    def _read_frame(self) -> Tuple[int, bytes]:
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
        # payload 는 마지막 읽기라 timeout 때 되돌릴 것이 없다.
        data = unmask(self._need(length, track=False), mask)
        if not fin:
            raise ProtocolError("fragmented frames are not supported")
        return opcode, data


def unmask(payload: bytes, mask: bytes) -> bytes:
    """클라이언트 프레임 마스크(4바이트 반복 XOR)를 푼다.

    손 메시지는 수 KB 라 바이트별 파이썬 루프 대신 큰 정수 XOR 한 번으로 한다.
    """
    n = len(payload)
    key = (mask * (n // 4 + 1))[:n]
    return (int.from_bytes(payload, "big") ^ int.from_bytes(key, "big")).to_bytes(n, "big")


class WebXRServer:

    def __init__(self, port: int, listen_ip: str = "0.0.0.0",
                 tls: bool = True, certfile: str = None, keyfile: str = None):
        self._port = port
        self._listen_ip = listen_ip
        self._tls = tls or bool(certfile)
        self._certfile = certfile
        self._keyfile = keyfile
        self._ssl_ctx: Optional[ssl.SSLContext] = None
        self._handlers: List[Callable[[dict], None]] = []
        self._lock = threading.Lock()
        self._refcount = 0
        self._sock: Optional[socket.socket] = None
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()  # 실행마다 새로 — 옛 연결이 새 실행에 섞이지 않게
        self._clients = 0
        self._frames = 0
        self._bad = 0
        self._handler_errors = 0  # 파싱은 됐지만 수신기가 처리에 실패한 횟수(_bad 와 따로)
        self._failed_handlers: List[Callable[[dict], None]] = []

    @classmethod
    def get(cls, port: int, listen_ip: str = "0.0.0.0", tls: bool = True,
            certfile: str = None, keyfile: str = None) -> "WebXRServer":
        """포트의 공유 서버. port=0 은 공유하지 않는 새 서버다(서버를 함께 쓰려면 고정 포트).

        한 포트의 서버는 프로세스에서 하나이고 설정도 하나다 — 같은 포트를 다른 설정으로 달라고
        하면 ValueError.
        """
        srv = cls(port, listen_ip, tls, certfile, keyfile)
        if port == 0:
            return srv
        with _registry_lock:
            have = _registry.setdefault(port, srv)
        if have._settings != srv._settings:
            raise ValueError(
                f"포트 {port} 의 WebXR 서버가 이미 다른 설정 {have._settings} 으로 있다 — "
                f"요청 {srv._settings}. 같은 포트의 수신기는 listen_ip·tls·인증서가 같아야 한다")
        return have

    @property
    def _settings(self) -> Tuple[str, bool, Optional[str], Optional[str]]:
        return self._listen_ip, self._tls, self._certfile, self._keyfile

    @property
    def scheme(self) -> str:
        return "https" if self._tls else "http"

    def add_handler(self, fn: Callable[[dict], None]) -> None:
        """메시지 하나를 모든 핸들러에 넘긴다 — 같은 포트의 수신기들이 함께 받는다."""
        with self._lock:
            if fn not in self._handlers:
                self._handlers.append(fn)

    def remove_handler(self, fn: Callable[[dict], None]) -> None:
        """핸들러를 뺀다. 없으면 무시한다."""
        with self._lock:
            if fn in self._handlers:
                self._handlers.remove(fn)
            if fn in self._failed_handlers:  # 버린 수신기를 붙잡지 않고, 다시 붙으면 다시 경고한다
                self._failed_handlers.remove(fn)

    @property
    def port(self) -> int:
        return self._port

    @property
    def is_running(self) -> bool:
        with self._lock:
            return self._sock is not None

    @property
    def stats(self) -> Tuple[int, int, int]:
        """(접속 수, 받은 JSON 메시지 수, 버린 프레임 수 — 파싱 실패·형식 오류)."""
        return self._clients, self._frames, self._bad

    @property
    def handler_errors(self) -> int:
        """파싱된 메시지를 수신기 핸들러가 처리하다 예외를 낸 횟수."""
        return self._handler_errors

    def start(self) -> None:
        with self._lock:
            if self._sock is not None:
                self._refcount += 1
                return
            self._stop = threading.Event()
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
            self._refcount += 1  # bind 가 성공한 뒤에만 센다 — 실패한 start 가 참조를 남기지 않게
            # port=0 은 처음 bind 한 포트를 계속 쓴다 — 다시 시작해도 페이지가 붙은 포트 그대로.
            self._port = sock.getsockname()[1]
            self._thread = threading.Thread(
                target=self._accept_loop, args=(sock, self._stop), daemon=True,
                name=f"webxr-{self._port}",
            )
            self._thread.start()
            _log.info(
                "WebXR 서버 %s://%s:%d",
                self.scheme,
                preferred_ip() if self._tls else "localhost",
                self._port,
            )

    def stop(self) -> None:
        with self._lock:
            if self._refcount > 0:
                self._refcount -= 1
            if self._refcount > 0 or self._sock is None:
                return
            self._stop.set()
            thread = self._thread
            self._sock.close()  # 잠금 안에서 닫는다 — 곧바로 다시 start() 해도 포트가 비어 있게
            self._sock = self._thread = None
        if thread is not None:
            thread.join(timeout=2.0)

    def _accept_loop(self, sock: socket.socket, stop: threading.Event) -> None:
        # 자기 실행의 소켓·이벤트만 본다 — stop() 의 join 전에 다시 start() 해도 섞이지 않게.
        while not stop.is_set():
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
            threading.Thread(
                target=self._serve, args=(conn, stop), daemon=True, name="webxr-conn"
            ).start()

    def _serve(self, conn: socket.socket, stop: threading.Event) -> None:
        try:
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            conn.settimeout(10.0)
            head = self._read_head(conn)
            if head is None:
                return
            request, headers = head
            if headers.get("upgrade", "").lower() == "websocket":
                self._handshake(conn, headers)
                self._pump(conn, stop)
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
        resp = (
            "HTTP/1.1 101 Switching Protocols\r\n"
            "Upgrade: websocket\r\n"
            "Connection: Upgrade\r\n"
            f"Sec-WebSocket-Accept: {accept_key(key)}\r\n\r\n"
        )
        conn.sendall(resp.encode("ascii"))

    def _pump(self, conn: socket.socket, stop: threading.Event) -> None:
        conn.settimeout(IDLE_PING_S)
        reader = _FrameReader(conn)
        self._clients += 1
        last = time.monotonic()
        try:
            while not stop.is_set():
                try:
                    opcode, data = reader.read_frame()
                except socket.timeout:
                    if time.monotonic() - last > IDLE_DROP_S:
                        _log.info("무응답 %.0fs — 연결을 끊는다", IDLE_DROP_S)
                        return
                    conn.sendall(encode_frame(b"", OP_PING))
                    continue
                if stop.is_set():  # 읽는 동안 서버가 멈췄다 — 이 프레임은 넘기지 않는다
                    return
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
        with self._lock:
            handlers = list(self._handlers)
        for fn in handlers:  # 한 수신기의 예외가 다른 수신기·연결을 끊지 않게 한다
            try:
                fn(msg)
            except Exception:
                # 72–90 Hz 라 트레이스백은 핸들러마다 한 번만(연결 스레드끼리 잠금 아래에서 센다).
                with self._lock:
                    self._handler_errors += 1
                    first = fn in self._handlers and fn not in self._failed_handlers
                    if first:
                        self._failed_handlers.append(fn)
                if first:
                    _log.warning(
                        "WebXR 메시지 처리 실패 (%s) — 이후 같은 실패는 handler_errors 로만 센다",
                        getattr(fn, "__qualname__", fn),
                        exc_info=True,
                    )

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
        conn.sendall(
            (
                f"HTTP/1.1 200 OK\r\nContent-Type: {ctype}\r\n"
                f"Content-Length: {len(body)}\r\n"
                f"Cache-Control: no-store\r\n\r\n"
            ).encode("ascii")
            + body
        )

