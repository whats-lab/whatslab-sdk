import json
import os
import pathlib
import socket
import ssl
import time
import urllib.error
import urllib.request

import pytest
from websockets.sync.client import connect

from whatslab.receiver.webxr import WebXRHandReceiver
from whatslab.receiver.webxr.ws_server import _STATIC_DIR, WebXRServer


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


def _insecure():
    ctx = ssl.create_default_context()
    ctx.check_hostname = False
    ctx.verify_mode = ssl.CERT_NONE
    ctx.maximum_version = ssl.TLSVersion.TLSv1_2
    return ctx


@pytest.fixture
def server():
    got = []
    srv = WebXRServer(got.append, 0, tls=False)
    srv.start()
    try:
        yield srv, got
    finally:
        srv.stop()


def test_message_roundtrip(server):
    srv, got = server
    with connect(f"ws://127.0.0.1:{srv.port}/ws") as ws:
        ws.send(json.dumps({"t": 1.0}))
        assert _wait(lambda: got == [{"t": 1.0}])


def test_bad_message_closes_only_that_connection():
    good = []
    srv = WebXRServer(good.append, 0, tls=False)
    srv.start()
    try:
        with connect(f"ws://127.0.0.1:{srv.port}/ws") as bad:
            bad.send("not json")
            assert _wait(lambda: bad.state.name == "CLOSED")
        with connect(f"ws://127.0.0.1:{srv.port}/ws") as ws:
            ws.send(json.dumps({"ok": 1}))
            assert _wait(lambda: good == [{"ok": 1}])
    finally:
        srv.stop()


def test_page_is_served_and_other_paths_are_404(server):
    srv, _ = server
    body = urllib.request.urlopen(f"http://127.0.0.1:{srv.port}/", timeout=5).read()
    for needle in (b"immersive-ar", b"immersive-vr", b"getJointPose", b"XRWebGLLayer"):
        assert needle in body
    with pytest.raises(urllib.error.HTTPError) as e:
        urllib.request.urlopen(f"http://127.0.0.1:{srv.port}/nope", timeout=5)
    assert e.value.code == 404


def test_restart_keeps_ephemeral_port():
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
    got = []
    srv = WebXRServer(got.append, 0, tls=False)
    srv.start()
    old = connect(f"ws://127.0.0.1:{srv.port}/ws")
    srv.stop()
    srv.start()
    try:
        old.send(json.dumps({"stale": 1}))
        with connect(f"ws://127.0.0.1:{srv.port}/ws") as new:
            new.send(json.dumps({"fresh": 1}))
            assert _wait(lambda: {"fresh": 1} in got)
        time.sleep(0.1)
        assert {"stale": 1} not in got
    finally:
        old.close()
        srv.stop()


def test_busy_port_raises():
    blocker = socket.socket()
    blocker.bind(("0.0.0.0", 0))
    blocker.listen(1)
    try:
        srv = WebXRServer(lambda msg: None, blocker.getsockname()[1], tls=False)
        with pytest.raises(OSError):
            srv.start()
        assert not srv.is_running
    finally:
        blocker.close()


def _page() -> str:
    return pathlib.Path(_STATIC_DIR, "index.html").read_text()


def test_page_prefers_passthrough_and_sends_only_hands():
    html = _page()
    assert html.index("immersive-ar") < html.index('"immersive-vr"')
    assert '"hand-tracking"' in html
    assert "JSON.stringify({ t: t, hands: hands })" in html
    assert "gripSpace" not in html and "hmd" not in html


def test_page_recovers_from_stalled_socket():
    html = _page()
    for needle in ("stalledSince", "dropSocket()", "socketReady(t)", "reconnects",
                   "bufferedAmount > BUFFER_MAX"):
        assert needle in html


def test_tls_cert_covers_preferred_ip(tmp_path):
    import subprocess

    from whatslab.receiver.webxr.tls import preferred_ip
    cert, key = _cert(tmp_path)
    assert os.path.isfile(cert) and os.path.isfile(key)
    assert oct(os.stat(key).st_mode & 0o777) == "0o600"
    txt = subprocess.run(["openssl", "x509", "-in", cert, "-noout", "-text"],
                         capture_output=True, text=True).stdout
    assert "DNS:localhost" in txt
    ip = preferred_ip()
    assert ip.startswith("127.") or f"IP Address:{ip}" in txt


def test_tls_cert_is_cached(tmp_path):
    cert, _ = _cert(tmp_path)
    first = pathlib.Path(cert).read_bytes()
    cert2, _ = _cert(tmp_path)
    assert pathlib.Path(cert2).read_bytes() == first


def test_wss_roundtrip_and_https_page(tmp_path):
    cert, key = _cert(tmp_path)
    got = []
    srv = WebXRServer(got.append, 0, tls=True, certfile=cert, keyfile=key)
    srv.start()
    try:
        assert srv.scheme == "https"
        page = urllib.request.urlopen(f"https://127.0.0.1:{srv.port}/", timeout=5,
                                      context=_insecure()).read()
        assert b"getJointPose" in page
        with connect(f"wss://127.0.0.1:{srv.port}/ws", ssl=_insecure()) as ws:
            ws.send(json.dumps({"secure": 1}))
            assert _wait(lambda: {"secure": 1} in got)
    finally:
        srv.stop()


def test_plain_http_client_is_rejected_by_tls_server(tmp_path):
    cert, key = _cert(tmp_path)
    srv = WebXRServer(lambda msg: None, 0, tls=True, certfile=cert, keyfile=key)
    srv.start()
    try:
        with pytest.raises(OSError):
            urllib.request.urlopen(f"http://127.0.0.1:{srv.port}/", timeout=3).read()
    finally:
        srv.stop()


class _UdpSocket:
    def __init__(self, rc, addr):
        self.rc, self.addr = rc, addr

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        return False

    def connect_ex(self, target):
        return self.rc

    def getsockname(self):
        return (self.addr, 40000)


def test_preferred_ip_is_the_routed_source_address(monkeypatch):
    from whatslab.receiver.webxr import tls
    monkeypatch.setattr(tls.socket, "socket", lambda *a: _UdpSocket(0, "172.20.1.5"))
    assert tls.preferred_ip() == "172.20.1.5"
    monkeypatch.setattr(tls.socket, "socket", lambda *a: _UdpSocket(101, ""))
    assert tls.preferred_ip() == "127.0.0.1"


def test_tls_url_uses_lan_ip_not_localhost(tmp_path):
    from whatslab.receiver.webxr.tls import preferred_ip
    cert, key = _cert(tmp_path)
    rx = WebXRHandReceiver(port=0, tls=True, certfile=cert, keyfile=key)
    rx.start()
    try:
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
