"""WebXR 서버(ws_server)·TLS·정적 페이지 — 수신기 종류와 무관한 전송 계층."""
import base64
import json
import os
import pathlib
import socket
import ssl
import struct
import time

import numpy as np
import pytest

from whatslab.receiver.webxr import WebXRHandReceiver
from whatslab.receiver.webxr.ws_server import (
    OP_CLOSE,
    OP_TEXT,
    WebXRServer,
    accept_key,
    encode_frame,
)


def _mask(payload, opcode=OP_TEXT):
    n = len(payload)
    head = bytes([0x80 | opcode])
    if n < 126:
        head += bytes([0x80 | n])
    elif n < 65536:
        head += bytes([0x80 | 126]) + struct.pack(">H", n)
    else:
        head += bytes([0x80 | 127]) + struct.pack(">Q", n)
    key = b"\x01\x02\x03\x04"
    masked = bytes(b ^ key[i & 3] for i, b in enumerate(payload))
    return head + key + masked


def _upgrade(sock):
    key = base64.b64encode(b"0123456789abcdef").decode()
    sock.sendall((f"GET /ws HTTP/1.1\r\nHost: localhost\r\nUpgrade: websocket\r\n"
                  f"Connection: Upgrade\r\nSec-WebSocket-Key: {key}\r\n"
                  f"Sec-WebSocket-Version: 13\r\n\r\n").encode())
    resp = b""
    while b"\r\n\r\n" not in resp:
        resp += sock.recv(4096)
    assert b"101" in resp.split(b"\r\n")[0]
    assert accept_key(key).encode() in resp
    return sock


def _handshake(port):
    return _upgrade(socket.create_connection(("127.0.0.1", port), timeout=5.0))


def _wait(fn, timeout=3.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if fn():
            return True
        time.sleep(0.01)
    return False


def _cert(tmp_path):
    from whatslab.receiver.webxr.tls import ensure_cert
    return ensure_cert(cache_dir=tmp_path)


@pytest.fixture
def server():
    got = []
    srv = WebXRServer(got.append, 0, tls=False)
    srv.start()
    try:
        yield srv, got
    finally:
        srv.stop()


def test_accept_key_rfc6455_vector():
    assert accept_key("dGhlIHNhbXBsZSBub25jZQ==") == "s3pPLMBiTxaQ9kYGzzhZRbK+xOo="


def test_encode_frame_lengths():
    assert encode_frame(b"hi")[:2] == bytes([0x81, 2])
    assert encode_frame(b"x" * 200)[:4] == bytes([0x81, 126]) + struct.pack(">H", 200)
    assert encode_frame(b"x" * 70000)[:10] == bytes([0x81, 127]) + struct.pack(">Q", 70000)


def test_unmask_matches_bytewise_xor():
    from whatslab.receiver.webxr.ws_server import unmask

    rng = np.random.default_rng(0)
    for n in (0, 1, 3, 4, 5, 125, 4096 + 3):
        payload = rng.integers(0, 256, n, dtype=np.uint8).tobytes()
        mask = rng.integers(0, 256, 4, dtype=np.uint8).tobytes()
        want = bytes(b ^ mask[i & 3] for i, b in enumerate(payload))
        assert unmask(payload, mask) == want


def test_frame_reader_keeps_partial_frame_on_timeout():
    """헤더만 오고 payload 가 늦으면(timeout) 다음 호출이 프레임을 처음부터 다시 읽는다."""
    from whatslab.receiver.webxr.ws_server import _FrameReader

    frame = _mask(b'{"t": 1}')
    chunks = [frame[:3], socket.timeout(), frame[3:]]

    class _Sock:
        def recv(self, n):
            item = chunks.pop(0)
            if isinstance(item, Exception):
                raise item
            return item

    reader = _FrameReader(_Sock())
    with pytest.raises(socket.timeout):
        reader.read_frame()
    assert reader.read_frame() == (OP_TEXT, b'{"t": 1}')


def test_message_roundtrip_and_close(server):
    srv, got = server
    sock = _handshake(srv.port)
    sock.sendall(_mask(json.dumps({"t": 1.0}).encode()))
    assert _wait(lambda: got == [{"t": 1.0}])
    sock.sendall(_mask(b"", OP_CLOSE))
    sock.close()


def test_malformed_payload_counted_not_raised(server):
    srv, got = server
    sock = _handshake(srv.port)
    sock.sendall(_mask(b"not json"))
    sock.sendall(_mask(json.dumps([1, 2, 3]).encode()))
    assert _wait(lambda: srv.stats[2] >= 2)
    sock.sendall(_mask(json.dumps({"ok": 1}).encode()))
    assert _wait(lambda: {"ok": 1} in got)
    sock.close()


def test_handler_errors_counted_apart_from_bad_frames():
    """handler 예외는 handler_errors 로 센다(버린 프레임 아님)."""
    def bad(msg):
        raise KeyError("x")

    srv = WebXRServer(bad, 0, tls=False)
    for _ in range(3):
        srv._dispatch(b'{"t": 1}')
    assert srv.stats[1:] == (3, 0) and srv.handler_errors == 3


def test_reconnect_after_abrupt_client_drop(server):
    srv, got = server
    first = _handshake(srv.port)
    first.sendall(_mask(b'{"n": 1}'))
    assert _wait(lambda: {"n": 1} in got)
    first.setsockopt(socket.SOL_SOCKET, socket.SO_LINGER, struct.pack("ii", 1, 0))
    first.close()
    assert _wait(lambda: srv.stats[0] == 0, timeout=5.0)
    second = _handshake(srv.port)
    second.sendall(_mask(b'{"n": 2}'))
    assert _wait(lambda: {"n": 2} in got)
    second.close()


def test_server_pings_idle_client():
    from whatslab.receiver.webxr import ws_server
    old = ws_server.IDLE_PING_S
    ws_server.IDLE_PING_S = 0.2
    try:
        srv = WebXRServer(lambda msg: None, 0, tls=False)
        srv.start()
        try:
            sock = _handshake(srv.port)
            sock.settimeout(5.0)
            assert sock.recv(2)[0] & 0x0F == ws_server.OP_PING
        finally:
            srv.stop()
    finally:
        ws_server.IDLE_PING_S = old


def test_restart_keeps_ephemeral_port():
    """port=0 은 처음 bind 한 포트를 다시 시작해도 쓴다(페이지가 그 포트에 붙어 있다)."""
    srv = WebXRServer(lambda msg: None, 0, tls=False)
    srv.start()
    port = srv.port
    srv.stop()
    srv.start()
    try:
        assert srv.port == port and srv.is_running
    finally:
        srv.stop()


def test_start_and_stop_are_idempotent():
    srv = WebXRServer(lambda msg: None, 0, tls=False)
    srv.stop()
    srv.start()
    srv.start()
    assert srv.is_running
    srv.stop()
    srv.stop()
    assert not srv.is_running


def test_connection_from_before_restart_is_not_dispatched():
    """멈췄다 곧바로 다시 시작해도 포트가 비고, 멈추기 전 연결의 프레임은 새 실행에 안 간다."""
    got = []
    srv = WebXRServer(got.append, 0, tls=False)
    srv.start()
    old = _handshake(srv.port)
    srv.stop()
    srv.start()
    try:
        old.sendall(_mask(b'{"stale": 1}'))
        new = _handshake(srv.port)
        new.sendall(_mask(b'{"fresh": 1}'))
        assert _wait(lambda: {"fresh": 1} in got)
        time.sleep(0.1)
        assert {"stale": 1} not in got
        old.close()
        new.close()
    finally:
        srv.stop()


def test_static_page_is_served(server):
    srv, _ = server
    sock = socket.create_connection(("127.0.0.1", srv.port), timeout=5.0)
    sock.sendall(b"GET / HTTP/1.1\r\nHost: localhost\r\n\r\n")
    resp = b""
    while b"</script>" not in resp:
        chunk = sock.recv(4096)
        if not chunk:
            break
        resp += chunk
    assert b"200 OK" in resp
    for needle in (b"immersive-ar", b"immersive-vr", b"getJointPose", b"XRWebGLLayer"):
        assert needle in resp
    sock.close()


def test_static_dir_ships_index():
    from whatslab.receiver.webxr import ws_server
    assert os.path.isfile(os.path.join(ws_server._STATIC_DIR, "index.html"))


def _page() -> str:
    from whatslab.receiver.webxr import ws_server
    return pathlib.Path(ws_server._STATIC_DIR, "index.html").read_text()


def test_page_prefers_passthrough_and_sends_only_hands():
    html = _page()
    assert html.index("immersive-ar") < html.index('"immersive-vr"')
    assert "gl.clearColor(0, 0, 0, passthrough ? 0 : 1)" in html
    assert "JSON.stringify({ t: t, hands: hands })" in html
    assert "gripSpace" not in html and "msg.hmd" not in html


def test_page_recovers_from_stalled_socket():
    html = _page()
    for needle in ("stalledSince", "dropSocket()", "socketReady(t)", "reconnects",
                   "bufferedAmount > BUFFER_MAX"):
        assert needle in html


def test_tls_cert_covers_local_ips(tmp_path):
    import subprocess

    from whatslab.receiver.webxr.tls import local_ips
    cert, key = _cert(tmp_path)
    assert os.path.isfile(cert) and os.path.isfile(key)
    assert oct(os.stat(key).st_mode & 0o777) == "0o600"
    txt = subprocess.run(["openssl", "x509", "-in", cert, "-noout", "-text"],
                         capture_output=True, text=True).stdout
    assert "DNS:localhost" in txt
    for ip in local_ips():
        assert f"IP Address:{ip}" in txt


def test_tls_cert_is_cached(tmp_path):
    cert, _ = _cert(tmp_path)
    first = pathlib.Path(cert).read_bytes()
    cert2, _ = _cert(tmp_path)
    assert pathlib.Path(cert2).read_bytes() == first


def test_wss_roundtrip(tmp_path):
    cert, key = _cert(tmp_path)
    got = []
    srv = WebXRServer(got.append, 0, tls=True, certfile=cert, keyfile=key)
    srv.start()
    try:
        assert srv.scheme == "https"
        ctx = ssl.create_default_context()
        ctx.check_hostname = False
        ctx.verify_mode = ssl.CERT_NONE
        raw = socket.create_connection(("127.0.0.1", srv.port), timeout=5.0)
        sock = _upgrade(ctx.wrap_socket(raw, server_hostname="localhost"))
        sock.sendall(_mask(b'{"secure": 1}'))
        assert _wait(lambda: {"secure": 1} in got)
        sock.close()
    finally:
        srv.stop()


def test_plain_http_client_is_rejected_by_tls_server(tmp_path):
    cert, key = _cert(tmp_path)
    srv = WebXRServer(lambda msg: None, 0, tls=True, certfile=cert, keyfile=key)
    srv.start()
    try:
        sock = socket.create_connection(("127.0.0.1", srv.port), timeout=5.0)
        sock.sendall(b"GET / HTTP/1.1\r\nHost: localhost\r\n\r\n")
        sock.settimeout(3.0)
        try:
            data = sock.recv(4096)
        except (socket.timeout, ConnectionResetError, OSError):
            data = b""
        assert b"200 OK" not in data
        sock.close()
    finally:
        srv.stop()


def test_preferred_ip_is_not_a_docker_bridge():
    from whatslab.receiver.webxr.tls import local_ips, preferred_ip, route_ip
    ip = preferred_ip()
    assert ip in local_ips() or ip == "127.0.0.1"
    routed = route_ip()
    if routed and not routed.startswith("127."):
        assert ip == routed


def test_tls_url_uses_lan_ip_not_localhost(tmp_path):
    from whatslab.receiver.webxr.tls import preferred_ip
    cert, key = _cert(tmp_path)
    rx = WebXRHandReceiver(port=0, tls=True, certfile=cert, keyfile=key)
    rx.start()
    try:
        assert "localhost" not in rx.url
        assert rx.url.startswith(f"https://{preferred_ip()}:")
    finally:
        rx.stop()


def test_plain_url_uses_localhost_for_adb_reverse():
    rx = WebXRHandReceiver(port=0, tls=False)
    rx.start()
    try:
        assert rx.url.startswith("http://localhost:")
    finally:
        rx.stop()


def test_tls_is_on_by_default():
    rx = WebXRHandReceiver(port=0)
    rx.start()
    try:
        assert rx.url.startswith("https://")
    finally:
        rx.stop()
