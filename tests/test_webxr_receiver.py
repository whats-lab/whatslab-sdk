import base64
import json
import os
import socket
import struct
import time

import numpy as np
import pathlib
import pytest

from whatslab.receiver.webxr import WebXRControllerReceiver
from whatslab.receiver.webxr.base import _CANONICAL_M
from whatslab.receiver.webxr.ws_server import (OP_CLOSE, OP_TEXT, WebXRServer,
                                               accept_key, encode_frame)


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


def _handshake(port):
    sock = socket.create_connection(("127.0.0.1", port), timeout=5.0)
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


def ensure_cert_for(tmp_path):
    from whatslab.receiver.webxr.tls import ensure_cert
    return ensure_cert(cache_dir=tmp_path)


def _wait(fn, timeout=3.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if fn():
            return True
        time.sleep(0.01)
    return False


def test_accept_key_rfc6455_vector():
    assert accept_key("dGhlIHNhbXBsZSBub25jZQ==") == "s3pPLMBiTxaQ9kYGzzhZRbK+xOo="


def test_encode_frame_lengths():
    assert encode_frame(b"hi")[:2] == bytes([0x81, 2])
    assert encode_frame(b"x" * 200)[:4] == bytes([0x81, 126]) + struct.pack(">H", 200)
    assert encode_frame(b"x" * 70000)[:10] == bytes([0x81, 127]) + struct.pack(">Q", 70000)


def test_canonical_matrix_is_proper_rotation():
    assert np.isclose(np.linalg.det(_CANONICAL_M), 1.0)
    assert np.allclose(_CANONICAL_M @ _CANONICAL_M.T, np.eye(3))


def test_webxr_axes_map_to_canonical():
    forward = _CANONICAL_M @ np.array([0.0, 0.0, -1.0])
    up = _CANONICAL_M @ np.array([0.0, 1.0, 0.0])
    right = _CANONICAL_M @ np.array([1.0, 0.0, 0.0])
    assert np.allclose(forward, [1.0, 0.0, 0.0])
    assert np.allclose(up, [0.0, 0.0, 1.0])
    assert np.allclose(right, [0.0, -1.0, 0.0])


def test_controller_roundtrip_over_socket():
    rx = WebXRControllerReceiver(port=0, tls=False)
    rx.start()
    try:
        port = rx.port
        assert port > 0
        sock = _handshake(port)
        msg = {"t": 1.0,
               "hmd": {"pos": [0.0, 1.6, 0.0], "quat": [0.0, 0.0, 0.0, 1.0]},
               "right": {"pos": [0.1, 1.2, -0.5], "quat": [0.0, 0.0, 0.0, 1.0]}}
        sock.sendall(_mask(json.dumps(msg).encode()))
        assert _wait(lambda: rx.get("right").tracked)

        s = rx.get("right")
        assert np.allclose(s.controller.pos, [0.5, -0.1, -0.4])
        assert s.hmd is not None
        assert np.allclose(s.hmd.pos, [0.0, 0.0, 1.6])
        assert not rx.get("left").tracked

        sock.sendall(_mask(b"", OP_CLOSE))
        sock.close()
    finally:
        rx.stop()


def test_stale_timeout_marks_untracked():
    rx = WebXRControllerReceiver(port=0, stale_timeout=0.05, tls=False)
    rx.start()
    try:
        sock = _handshake(rx.port)
        sock.sendall(_mask(json.dumps(
            {"left": {"pos": [0, 0, 0], "quat": [0, 0, 0, 1]}}).encode()))
        assert _wait(lambda: rx.get("left").tracked)
        assert _wait(lambda: not rx.get("left").tracked)
        sock.close()
    finally:
        rx.stop()


def test_malformed_payload_counted_not_raised():
    rx = WebXRControllerReceiver(port=0, tls=False)
    rx.start()
    try:
        sock = _handshake(rx.port)
        sock.sendall(_mask(b"not json"))
        sock.sendall(_mask(json.dumps([1, 2, 3]).encode()))
        assert _wait(lambda: rx._srv.stats[2] >= 2)
        sock.sendall(_mask(json.dumps(
            {"right": {"pos": [0, 0, 0], "quat": [0, 0, 0, 1]}}).encode()))
        assert _wait(lambda: rx.get("right").tracked)
        sock.close()
    finally:
        rx.stop()


def test_static_page_is_served():
    srv = WebXRServer.get(0, tls=False)
    srv.start()
    try:
        sock = socket.create_connection(("127.0.0.1", srv.port), timeout=5.0)
        sock.sendall(b"GET / HTTP/1.1\r\nHost: localhost\r\n\r\n")
        resp = b""
        while b"</script>" not in resp:
            chunk = sock.recv(4096)
            if not chunk:
                break
            resp += chunk
        assert b"200 OK" in resp
        assert b"immersive-ar" in resp
        assert b"immersive-vr" in resp
        assert b"gripSpace" in resp
        assert b"XRWebGLLayer" in resp
        sock.close()
    finally:
        srv.stop()


def test_static_dir_ships_index():
    from whatslab.receiver.webxr import ws_server
    assert os.path.isfile(os.path.join(ws_server._STATIC_DIR, "index.html"))


def test_page_renders_markers_and_passthrough():
    from whatslab.receiver.webxr import ws_server
    html = pathlib.Path(ws_server._STATIC_DIR, "index.html").read_text()
    assert "immersive-ar" in html
    assert html.index("immersive-ar") < html.index('"immersive-vr"')
    assert "gl.clearColor(0, 0, 0, passthrough ? 0 : 1)" in html
    assert "getViewport" in html
    assert "projectionMatrix" in html
    assert "transform.inverse.matrix" in html
    assert "drawArrays" in html


def test_tls_cert_covers_local_ips(tmp_path):
    from whatslab.receiver.webxr.tls import ensure_cert, local_ips
    import subprocess
    cert, key = ensure_cert(cache_dir=tmp_path)
    assert os.path.isfile(cert) and os.path.isfile(key)
    assert oct(os.stat(key).st_mode & 0o777) == "0o600"
    txt = subprocess.run(["openssl", "x509", "-in", cert, "-noout", "-text"],
                         capture_output=True, text=True).stdout
    assert "DNS:localhost" in txt
    for ip in local_ips():
        assert f"IP Address:{ip}" in txt


def test_tls_cert_is_cached(tmp_path):
    from whatslab.receiver.webxr.tls import ensure_cert
    cert, _ = ensure_cert(cache_dir=tmp_path)
    first = pathlib.Path(cert).read_bytes()
    cert2, _ = ensure_cert(cache_dir=tmp_path)
    assert pathlib.Path(cert2).read_bytes() == first


def test_wss_roundtrip(tmp_path):
    import ssl
    cert, key = ensure_cert_for(tmp_path)
    rx = WebXRControllerReceiver(port=0, tls=True, certfile=cert, keyfile=key)
    rx.start()
    try:
        assert rx.url.startswith("https://")
        ctx = ssl.create_default_context()
        ctx.check_hostname = False
        ctx.verify_mode = ssl.CERT_NONE
        raw = socket.create_connection(("127.0.0.1", rx.port), timeout=5.0)
        sock = ctx.wrap_socket(raw, server_hostname="localhost")
        key_b64 = base64.b64encode(b"0123456789abcdef").decode()
        sock.sendall((f"GET /ws HTTP/1.1\r\nHost: localhost\r\nUpgrade: websocket\r\n"
                      f"Connection: Upgrade\r\nSec-WebSocket-Key: {key_b64}\r\n"
                      f"Sec-WebSocket-Version: 13\r\n\r\n").encode())
        resp = b""
        while b"\r\n\r\n" not in resp:
            resp += sock.recv(4096)
        assert b"101" in resp.split(b"\r\n")[0]
        sock.sendall(_mask(json.dumps(
            {"left": {"pos": [0.0, 0.0, -1.0], "quat": [0, 0, 0, 1]}}).encode()))
        assert _wait(lambda: rx.get("left").tracked)
        assert np.allclose(rx.get("left").controller.pos, [1.0, 0.0, 0.0])
        sock.close()
    finally:
        rx.stop()


def test_plain_http_client_is_rejected_by_tls_server(tmp_path):
    cert, key = ensure_cert_for(tmp_path)
    rx = WebXRControllerReceiver(port=0, tls=True, certfile=cert, keyfile=key)
    rx.start()
    try:
        sock = socket.create_connection(("127.0.0.1", rx.port), timeout=5.0)
        sock.sendall(b"GET / HTTP/1.1\r\nHost: localhost\r\n\r\n")
        sock.settimeout(3.0)
        try:
            data = sock.recv(4096)
        except (socket.timeout, ConnectionResetError, OSError):
            data = b""
        assert b"200 OK" not in data
        sock.close()
    finally:
        rx.stop()


def test_preferred_ip_is_not_a_docker_bridge():
    import ipaddress
    from whatslab.receiver.webxr.tls import local_ips, preferred_ip, route_ip
    ip = preferred_ip()
    assert ip in local_ips() or ip == "127.0.0.1"
    routed = route_ip()
    if routed and not routed.startswith("127."):
        assert ip == routed


def test_tls_url_uses_lan_ip_not_localhost(tmp_path):
    from whatslab.receiver.webxr.tls import preferred_ip
    cert, key = ensure_cert_for(tmp_path)
    rx = WebXRControllerReceiver(port=0, tls=True, certfile=cert, keyfile=key)
    rx.start()
    try:
        assert "localhost" not in rx.url
        assert rx.url.startswith(f"https://{preferred_ip()}:")
    finally:
        rx.stop()


def test_plain_url_uses_localhost_for_adb_reverse():
    rx = WebXRControllerReceiver(port=0, tls=False)
    rx.start()
    try:
        assert rx.url.startswith("http://localhost:")
    finally:
        rx.stop()


def test_tls_is_on_by_default():
    rx = WebXRControllerReceiver(port=0)
    rx.start()
    try:
        assert rx.url.startswith("https://")
        assert "localhost" not in rx.url
    finally:
        rx.stop()


def _feed(rx, payload):
    sock = _handshake_for(rx)
    sock.sendall(_mask(json.dumps(payload).encode()))
    return sock


def _handshake_for(rx):
    return _handshake(rx.port)


PAYLOAD = {"hmd": {"pos": [1.0, 1.6, 2.0], "quat": [0.0, 0.0, 0.0, 1.0]},
           "right": {"pos": [1.3, 1.2, 1.5], "quat": [0.0, 0.0, 0.0, 1.0]}}


def test_pos_frame_room_is_absolute():
    rx = WebXRControllerReceiver(port=0, tls=False, pos_frame="room")
    rx.start()
    try:
        sock = _feed(rx, PAYLOAD)
        assert _wait(lambda: rx.get("right").tracked)
        assert np.allclose(rx.get("right").controller.pos, [-1.5, -1.3, 1.2])
        sock.close()
    finally:
        rx.stop()


def test_pos_frame_hmd_subtracts_head_position():
    rx = WebXRControllerReceiver(port=0, tls=False, pos_frame="hmd")
    rx.start()
    try:
        sock = _feed(rx, PAYLOAD)
        assert _wait(lambda: rx.get("right").tracked)
        s = rx.get("right")
        assert np.allclose(s.controller.pos, np.array([-1.5, -1.3, 1.2]) - s.hmd.pos)
        assert np.linalg.norm(s.controller.pos) < 1.0
        sock.close()
    finally:
        rx.stop()


def test_pos_frame_hmd_yaw_is_default_and_derotates():
    rx = WebXRControllerReceiver(port=0, tls=False)
    assert rx.pos_frame == "hmd_yaw"
    rx.start()
    try:
        yaw90 = [0.0, np.sin(np.pi / 4), 0.0, np.cos(np.pi / 4)]
        sock = _feed(rx, {"hmd": {"pos": [0.0, 1.6, 0.0], "quat": yaw90},
                          "right": {"pos": [0.0, 1.6, -0.4], "quat": [0, 0, 0, 1]}})
        assert _wait(lambda: rx.get("right").tracked)
        pos = rx.get("right").controller.pos
        assert np.isclose(np.linalg.norm(pos), 0.4)
        assert np.allclose(pos, [0.0, -0.4, 0.0], atol=1e-6)
        sock.close()
    finally:
        rx.stop()


def test_pos_frame_falls_back_to_raw_without_hmd():
    rx = WebXRControllerReceiver(port=0, tls=False)
    rx.start()
    try:
        sock = _feed(rx, {"right": {"pos": [0.0, 0.0, -1.0], "quat": [0, 0, 0, 1]}})
        assert _wait(lambda: rx.get("right").tracked)
        assert np.allclose(rx.get("right").controller.pos, [1.0, 0.0, 0.0])
        sock.close()
    finally:
        rx.stop()


def test_pos_frame_rejects_unknown():
    with pytest.raises(ValueError):
        WebXRControllerReceiver(port=0, tls=False, pos_frame="nope")


def test_reconnect_after_abrupt_client_drop():
    rx = WebXRControllerReceiver(port=0, tls=False, pos_frame="room")
    rx.start()
    try:
        first = _handshake(rx.port)
        first.sendall(_mask(json.dumps(
            {"right": {"pos": [0.0, 0.0, -1.0], "quat": [0, 0, 0, 1]}}).encode()))
        assert _wait(lambda: rx.get("right").tracked)
        assert np.allclose(rx.get("right").controller.pos, [1.0, 0.0, 0.0])

        first.setsockopt(socket.SOL_SOCKET, socket.SO_LINGER,
                         struct.pack("ii", 1, 0))
        first.close()
        assert _wait(lambda: rx._srv.stats[0] == 0, timeout=5.0)

        second = _handshake(rx.port)
        second.sendall(_mask(json.dumps(
            {"right": {"pos": [0.0, 0.0, -2.0], "quat": [0, 0, 0, 1]}}).encode()))
        assert _wait(lambda: np.allclose(rx.get("right").controller.pos,
                                         [2.0, 0.0, 0.0]))
        second.close()
    finally:
        rx.stop()


def test_server_pings_idle_client():
    from whatslab.receiver.webxr import ws_server
    old = ws_server.IDLE_PING_S
    ws_server.IDLE_PING_S = 0.2
    try:
        rx = WebXRControllerReceiver(port=0, tls=False)
        rx.start()
        try:
            sock = _handshake(rx.port)
            sock.settimeout(5.0)
            b0 = sock.recv(2)
            assert b0[0] & 0x0F == ws_server.OP_PING
        finally:
            rx.stop()
    finally:
        ws_server.IDLE_PING_S = old


def test_page_recovers_from_stalled_socket():
    from whatslab.receiver.webxr import ws_server
    html = pathlib.Path(ws_server._STATIC_DIR, "index.html").read_text()
    assert "stalledSince" in html
    assert "dropSocket()" in html
    assert "socketReady(t)" in html
    assert "reconnects" in html
    assert "bufferedAmount > BUFFER_MAX" in html


def test_glove_model_rejects_unknown_arm_source():
    pytest.importorskip("pinocchio")
    from whatslab.teleop.models.glove import ARM_SOURCES
    assert set(ARM_SOURCES) == {"quest", "webxr"}
