import base64
import json
import os
import pathlib
import socket
import struct
import time

import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from webxr_pose_receiver import PoseReceiver

from whatslab.receiver.base import Y_UP_RH_TO_CANONICAL
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
    assert np.isclose(np.linalg.det(Y_UP_RH_TO_CANONICAL), 1.0)
    assert np.allclose(Y_UP_RH_TO_CANONICAL @ Y_UP_RH_TO_CANONICAL.T, np.eye(3))


def test_webxr_axes_map_to_canonical():
    forward = Y_UP_RH_TO_CANONICAL @ np.array([0.0, 0.0, -1.0])
    up = Y_UP_RH_TO_CANONICAL @ np.array([0.0, 1.0, 0.0])
    right = Y_UP_RH_TO_CANONICAL @ np.array([1.0, 0.0, 0.0])
    assert np.allclose(forward, [1.0, 0.0, 0.0])
    assert np.allclose(up, [0.0, 0.0, 1.0])
    assert np.allclose(right, [0.0, -1.0, 0.0])


def test_controller_roundtrip_over_socket():
    rx = PoseReceiver(port=0, tls=False)
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
    rx = PoseReceiver(port=0, stale_timeout=0.05, tls=False)
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
    rx = PoseReceiver(port=0, tls=False)
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
    import subprocess

    from whatslab.receiver.webxr.tls import ensure_cert, local_ips
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
    rx = PoseReceiver(port=0, tls=True, certfile=cert, keyfile=key)
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
    rx = PoseReceiver(port=0, tls=True, certfile=cert, keyfile=key)
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
    from whatslab.receiver.webxr.tls import local_ips, preferred_ip, route_ip
    ip = preferred_ip()
    assert ip in local_ips() or ip == "127.0.0.1"
    routed = route_ip()
    if routed and not routed.startswith("127."):
        assert ip == routed


def test_tls_url_uses_lan_ip_not_localhost(tmp_path):
    from whatslab.receiver.webxr.tls import preferred_ip
    cert, key = ensure_cert_for(tmp_path)
    rx = PoseReceiver(port=0, tls=True, certfile=cert, keyfile=key)
    rx.start()
    try:
        assert "localhost" not in rx.url
        assert rx.url.startswith(f"https://{preferred_ip()}:")
    finally:
        rx.stop()


def test_plain_url_uses_localhost_for_adb_reverse():
    rx = PoseReceiver(port=0, tls=False)
    rx.start()
    try:
        assert rx.url.startswith("http://localhost:")
    finally:
        rx.stop()


def test_tls_is_on_by_default():
    rx = PoseReceiver(port=0)
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
    rx = PoseReceiver(port=0, tls=False, pos_frame="room")
    rx.start()
    try:
        sock = _feed(rx, PAYLOAD)
        assert _wait(lambda: rx.get("right").tracked)
        assert np.allclose(rx.get("right").controller.pos, [-1.5, -1.3, 1.2])
        sock.close()
    finally:
        rx.stop()


def test_pos_frame_hmd_subtracts_head_position():
    rx = PoseReceiver(port=0, tls=False, pos_frame="hmd")
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
    rx = PoseReceiver(port=0, tls=False)
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
    rx = PoseReceiver(port=0, tls=False)
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
        PoseReceiver(port=0, tls=False, pos_frame="nope")


def test_reconnect_after_abrupt_client_drop():
    rx = PoseReceiver(port=0, tls=False, pos_frame="room")
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
        rx = PoseReceiver(port=0, tls=False)
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


@pytest.mark.parametrize("pos_frame, removes_yaw", [("hmd_yaw", True), ("room", False)])
def test_controller_orientation_follows_pos_frame(pos_frame, removes_yaw):
    """컨트롤러 방위도 위치와 같은 기준 — hmd_yaw 면 머리 yaw 를 뺀다."""
    from scipy.spatial.transform import Rotation

    rx = PoseReceiver(port=0, tls=False, pos_frame=pos_frame)
    head = Rotation.from_euler("y", 0.7).as_quat().tolist()  # WebXR y-up 의 yaw
    rx._on_message({"hmd": {"pos": [0.0, 1.6, 0.0], "quat": head},
                    "right": {"pos": [0.0, 1.2, -0.3], "quat": [0.0, 0.0, 0.0, 1.0]}})
    yaw = Rotation.from_quat(rx.get("right").controller.quat).as_euler("xyz")[2]
    assert yaw == pytest.approx(-0.7 if removes_yaw else 0.0, abs=1e-9)


@pytest.mark.parametrize("item", [
    {"pos": [0.0, 1.6], "quat": [0.0, 0.0, 0.0, 1.0]},
    {"pos": [0.0, 1.6, 0.0], "quat": [0.0, 0.0, 1.0]},
    {"pos": [0.0, float("nan"), 0.0], "quat": [0.0, 0.0, 0.0, 1.0]},
    {"pos": "abc", "quat": [0.0, 0.0, 0.0, 1.0]},
])
def test_malformed_pose_is_ignored(item):
    """잘못된 hmd·컨트롤러 항목은 버린다 — 저장했다가 get() 이 죽지 않게."""
    rx = PoseReceiver(port=0, tls=False)
    rx._on_message({"hmd": item, "right": item})
    s = rx.get("right")
    assert s.controller is None and s.hmd is None


def test_port_zero_servers_are_private():
    """port=0 서버는 공유하지 않는다 — 실제 포트로도 레지스트리에서 찾지 않는다."""
    a = PoseReceiver(port=0, tls=False)
    b = PoseReceiver(port=0, tls=False)
    assert a._srv is not b._srv
    a.start()
    try:
        assert WebXRServer.get(a.port, tls=False) is not a._srv
    finally:
        a.stop()


def test_unmask_matches_bytewise_xor():
    from whatslab.receiver.webxr.ws_server import unmask

    rng = np.random.default_rng(0)
    for n in (0, 1, 3, 4, 5, 125, 4096 + 3):
        payload = rng.integers(0, 256, n, dtype=np.uint8).tobytes()
        mask = rng.integers(0, 256, 4, dtype=np.uint8).tobytes()
        want = bytes(b ^ mask[i & 3] for i, b in enumerate(payload))
        assert unmask(payload, mask) == want


def test_restart_keeps_ephemeral_port():
    """port=0 은 처음 bind 한 포트를 다시 시작해도 쓴다(페이지가 그 포트에 붙어 있다)."""
    rx = PoseReceiver(port=0, tls=False)
    rx.start()
    port = rx.port
    rx.stop()
    rx.start()
    try:
        assert rx.port == port and rx._srv.is_running
    finally:
        rx.stop()


def test_shared_fixed_port_survives_any_restart_order(free_port):
    a = PoseReceiver(port=free_port, tls=False)
    b = PoseReceiver(port=free_port, tls=False)
    assert a._srv is b._srv
    a.start()
    b.start()
    a.stop()
    assert b._srv.is_running
    b.stop()
    assert not b._srv.is_running
    b.start()
    a.start()
    try:
        assert a._srv.is_running
    finally:
        a.stop()
        b.stop()
    assert not a._srv.is_running


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


def test_handler_errors_counted_apart_and_released():
    """핸들러 예외는 handler_errors 로 세고(버린 프레임 아님), 뗀 핸들러는 붙잡지 않는다."""
    srv = WebXRServer(0, tls=False)

    def bad(msg):
        raise KeyError("x")

    srv.add_handler(bad)
    for _ in range(3):
        srv._dispatch(b'{"t": 1}')
    assert srv.stats[1:] == (3, 0) and srv.handler_errors == 3
    srv.remove_handler(bad)
    assert bad not in srv._failed_handlers


def _yaw(head_quat) -> float:
    from whatslab.receiver.webxr.base import yaw_rz

    rz = yaw_rz(head_quat)
    return -float(np.arctan2(rz[1, 0], rz[0, 0]))


@pytest.mark.parametrize("pitch_deg", [0.0, 45.0, -45.0, 89.0, -89.0])
@pytest.mark.parametrize("roll_deg", [0.0, 30.0])
def test_head_yaw_is_forward_heading(pitch_deg, roll_deg):
    """yaw 는 앞 축의 수평 방향 — 피치 ±89° 에서도, 고개를 기울여도 몸 yaw 그대로다."""
    head = Rotation.from_euler("ZYX", [0.5, np.radians(pitch_deg), np.radians(roll_deg)])
    assert _yaw(head.as_quat()) == pytest.approx(0.5, abs=1e-9)


@pytest.mark.parametrize("pitch_deg", [90.0, -90.0])
def test_head_yaw_falls_back_to_up_axis_when_looking_straight_down_or_up(pitch_deg):
    """앞 축이 수직이면 위 축의 수평 방향으로 정한다(아래를 보면 위 축이 몸 앞을 향한다)."""
    head = Rotation.from_euler("ZYX", [0.5, np.radians(pitch_deg), 0.0])
    assert _yaw(head.as_quat()) == pytest.approx(0.5, abs=1e-9)


def test_connection_from_before_restart_is_not_dispatched():
    """멈췄다 곧바로 다시 시작해도 포트가 비고, 멈추기 전 연결의 프레임은 새 실행에 안 간다."""
    srv = WebXRServer.get(0, tls=False)
    got = []
    srv.add_handler(got.append)
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


def test_same_port_with_other_settings_is_rejected(free_port):
    """같은 포트를 다른 tls 로 달라고 하면 앞 서버 설정을 조용히 쓰지 않고 거부한다."""
    PoseReceiver(port=free_port, tls=False)
    with pytest.raises(ValueError):
        PoseReceiver(port=free_port, tls=True)
