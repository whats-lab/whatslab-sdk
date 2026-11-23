from __future__ import annotations

import os
import socket
import subprocess
import sys
import time

import numpy as np
import pytest
from airglove_client import AirGloveError, Client, DeviceStatus, HandFrame, Side
from scipy.spatial.transform import Rotation

import whatslab.receiver
import whatslab.receiver.glove
from whatslab.core.constants import XR_JOINT_COUNT, XR_JOINT_FIELDS, XR_JOINT_NAMES, XR_WRIST
from whatslab.core.interfaces import Receiver
from whatslab.receiver.glove import airglove
from whatslab.receiver.glove.airglove import (
    AirGloveReceiver,
    haptic_levels,
)
from whatslab.teleop import HandModel

CORE_ENV = "AIRGLOVE_CORE_DIR"
REPLAYER = "tools/spine_replay.py"
CAPTURE = "captures/spine_both_192s.agrec"
MIN_LIBRARY = (0, 4)
TEST_PORTS = range(47500, 47598)


def _frame(age=0.01, valid=True, seed=0):
    rng = np.random.default_rng(seed)
    ori = Rotation.random(XR_JOINT_COUNT, random_state=seed).as_quat().astype(np.float32)
    ori[XR_WRIST] = [0.0, 0.0, 0.0, 1.0]
    pos = rng.normal(scale=0.05, size=(XR_JOINT_COUNT, 3)).astype(np.float32)
    pos[XR_WRIST] = 0.0
    radii = np.full(XR_JOINT_COUNT, 0.01, dtype=np.float32)
    ok = np.full(XR_JOINT_COUNT, bool(valid))
    return HandFrame(ori, pos, radii, ok, 7, 123456, age)


class _FakeClient:
    instances: list = []

    def __init__(self, listen_address="127.0.0.1", listen_port=4040,
                 spine_address="127.0.0.1", spine_port=4042):
        self.args = (listen_address, listen_port, spine_address, spine_port)
        self.hands = {Side.LEFT: None, Side.RIGHT: None}
        self.wrists = {Side.LEFT: None, Side.RIGHT: None}
        self.status = None
        self.haptics = []
        self.stale = None
        self.closed = False
        self.fail_haptics = False
        self.protocol = None
        self.pair = None
        _FakeClient.instances.append(self)

    @staticmethod
    def version():
        return "0.4.0"

    def hand(self, side):
        return self.hands[Side(side)]

    def wrist(self, side):
        return self.wrists[Side(side)]

    def device_status(self):
        return self.status

    def set_haptics(self, side, strengths):
        if self.fail_haptics:
            raise AirGloveError("busy")
        self.haptics.append((Side(side), list(strengths)))

    def set_stale_threshold(self, seconds):
        self.stale = seconds

    def spine_protocol(self):
        return self.protocol

    def set_pair(self, pair):
        self.pair = pair

    def alarms(self):
        out, self.pending_alarms = list(getattr(self, "pending_alarms", [])), []
        return out

    def stats(self):
        return {"datagrams": 3, "malformed": 0, "dropped_seq": 0}

    def close(self):
        self.closed = True


@pytest.fixture
def fake_client(monkeypatch):
    _FakeClient.instances = []
    monkeypatch.setattr(airglove, "Client", _FakeClient)
    return _FakeClient


def _started(**kw):
    rx = AirGloveReceiver(**kw)
    rx.start()
    return rx, _FakeClient.instances[-1]


def test_exports_and_protocol():
    assert whatslab.receiver.AirGloveReceiver is AirGloveReceiver
    assert whatslab.receiver.glove.AirGloveReceiver is AirGloveReceiver
    assert isinstance(AirGloveReceiver(), Receiver)
    assert XR_JOINT_NAMES[0] == "palm" and XR_JOINT_NAMES[XR_WRIST] == "wrist"
    assert len(XR_JOINT_NAMES) == XR_JOINT_COUNT == 26


def test_ports_map_to_client_and_stale_is_local(fake_client):
    rx, c = _started(glove_port=47040, client_port=47042, target_ip="10.0.0.2",
                     stale_timeout=0.3)
    assert c.args == ("127.0.0.1", 47040, "10.0.0.2", 47042)
    assert c.stale is None and c.pair is None
    rx.start()
    assert len(fake_client.instances) == 1
    rx.stop()
    assert c.closed and not rx.running


def test_pair_is_forwarded_and_validated(fake_client):
    rx, c = _started(pair=2)
    assert c.pair == 2
    with pytest.raises(ValueError):
        AirGloveReceiver(pair=0)


def test_old_library_warns(fake_client, monkeypatch, caplog):
    monkeypatch.setattr(_FakeClient, "version", staticmethod(lambda: "0.3.0"))
    with caplog.at_level("WARNING", logger=airglove.__name__):
        _started()
    assert "0.4.0" in caplog.text


def test_protocol_mismatch_warns_once(fake_client, caplog):
    rx, c = _started()
    c.hands[Side.RIGHT] = _frame()
    assert rx.spine_protocol() is None
    c.protocol = (2, 0.1)
    with caplog.at_level("WARNING", logger=airglove.__name__):
        rx.get("right")
        rx.get("right")
    assert caplog.text.count("Spine 프로토콜 2") == 1
    assert rx.spine_protocol() == 2
    assert airglove.SPINE_PROTOCOL == whatslab.receiver.glove.SPINE_PROTOCOL == 3


def test_get_before_start_is_untracked_without_binding(fake_client):
    rx = AirGloveReceiver()
    s = rx.get("right")
    assert s.tracked is False and s.hand is None
    assert fake_client.instances == []
    assert rx.connected("right") is False
    assert rx.send_haptic("right", [1, 1, 1, 1, 1]) is False


def test_get_packs_xr_joints_as_received(fake_client):
    rx, c = _started()
    f = _frame()
    c.hands[Side.RIGHT] = f
    s = rx.get("right")
    assert s.tracked is True and s.hand.tracked is True
    xj = s.hand.xr_joints
    assert xj.shape == (XR_JOINT_COUNT, XR_JOINT_FIELDS)
    assert np.allclose(xj[:, 0:4], f.orientations)
    assert np.allclose(xj[:, 4:7], f.positions)
    assert np.allclose(xj[:, 7], f.radii)
    assert s.hand.xr_valid is not None and s.hand.xr_valid.all()
    assert rx.get("left").hand is None
    assert s.timestamp == pytest.approx(time.monotonic() - f.age_s, abs=0.05)


def test_stale_or_invalid_frame_is_untracked(fake_client):
    rx, c = _started(stale_timeout=0.2)
    c.hands[Side.RIGHT] = _frame(age=0.25)
    s = rx.get("right")
    assert s.tracked is False and s.hand.tracked is False
    assert s.hand.xr_joints is not None
    c.hands[Side.RIGHT] = _frame(age=0.01, valid=False)
    assert rx.get("right").tracked is False
    c.hands[Side.RIGHT] = _frame(age=0.19)
    assert rx.get("right").tracked is True


def test_haptic_levels_mapping():
    assert haptic_levels([0, 1, 2, 3, 4]) == [0, 1, 2, 3, 3]
    assert haptic_levels([-1, 0.4, 0.6, 2.5, float("nan")]) == [0, 0, 1, 2, 0]
    assert haptic_levels([1, 1, 1, 1, 1, 1, 1]) == [1, 1, 1, 1, 1]
    assert haptic_levels(["x", None, 2]) == [0, 0, 2, 0, 0]
    assert haptic_levels([]) == [0, 0, 0, 0, 0]


def test_send_haptic_clamps_pads_and_reports(fake_client):
    rx, c = _started()
    assert rx.send_haptic("right", [10, 20, 30, 40, 50]) is True
    assert rx.send_haptic("left", [2]) is True
    assert rx.send_haptic("left", []) is True
    assert c.haptics == [(Side.RIGHT, [3, 3, 3, 3, 3]), (Side.LEFT, [2, 0, 0, 0, 0]),
                         (Side.LEFT, [0, 0, 0, 0, 0])]
    c.fail_haptics = True
    assert rx.send_haptic("left", [1]) is False


def test_connected_uses_device_status_then_fresh_frame(fake_client):
    rx, c = _started()
    assert rx.connected("right") is False
    c.status = DeviceStatus(False, True, 0.1)
    assert rx.connected("right") is True and rx.connected("left") is False
    c.hands[Side.LEFT] = _frame(age=0.05)
    assert rx.connected("left") is True


def test_frame_exposes_receive_metadata_without_stale_filtering(fake_client):
    rx = AirGloveReceiver()
    assert rx.frame("right") is None
    assert rx.alarms() == [] and rx.stats() == {} and rx.device_status() is None
    rx.start()
    c = fake_client.instances[-1]
    assert rx.frame("right") is None
    raw = _frame(age=5.0, valid=False)
    c.hands[Side.RIGHT] = raw
    f = rx.frame("right")
    assert (f.side, f.seq, f.sender_time_us, f.age_s) == ("right", 7, 123456, 5.0)
    assert f.xr_joints.shape == (26, 8) and f.valid.dtype == bool and not f.valid.any()
    np.testing.assert_allclose(f.xr_joints[:, :4], raw.orientations)
    np.testing.assert_allclose(f.xr_joints[:, 4:7], raw.positions)
    np.testing.assert_allclose(f.xr_joints[:, 7], raw.radii)


def test_status_alarms_and_stats_pass_through(fake_client):
    rx, c = _started()
    c.status = DeviceStatus(False, True, 0.3)
    c.pending_alarms = ["RESTART"]
    assert rx.device_status().right_connected is True
    assert rx.alarms() == ["RESTART"] and rx.alarms() == []
    assert rx.stats()["datagrams"] == 3


def test_connected_ignores_old_device_status(fake_client):
    rx, c = _started()
    c.status = DeviceStatus(True, True, 1.9)
    assert rx.connected("right") is True
    c.status = DeviceStatus(True, True, 2.5)
    assert rx.connected("right") is False
    c.hands[Side.RIGHT] = _frame(age=0.05)
    assert rx.connected("right") is True


def test_hand_model_defaults_to_airglove_with_receiver_kwargs(fake_client):
    m = HandModel("geort:orca_right", hand_kwargs={"glove_port": 47140, "client_port": 47142})
    assert isinstance(m.hand_source, AirGloveReceiver)
    m.start()
    c = fake_client.instances[-1]
    assert c.args[1] == 47140 and c.args[3] == 47142
    assert m.get_q() == {"right": {}}
    c.hands[Side.RIGHT] = _frame()
    q = m.get_q()["right"]
    assert list(q) == m.retarget.joint_names
    m.stop()
    assert c.closed


def test_hand_model_rejects_source_with_kwargs():
    with pytest.raises(ValueError):
        HandModel(hand_source=object(), hand_kwargs={"glove_port": 4040})


def _free_port_pair():
    for p in TEST_PORTS:
        socks = []
        try:
            for port in (p, p + 2):
                sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                socks.append(sock)
                sock.bind(("127.0.0.1", port))
        except OSError:
            continue
        finally:
            for sock in socks:
                sock.close()
        return p
    pytest.skip("47500-47599 에서 빈 UDP 포트 쌍을 못 찾음")


def _wait(pred, timeout):
    t_end = time.monotonic() + timeout
    while time.monotonic() < t_end:
        v = pred()
        if v:
            return v
        time.sleep(0.02)
    return None


def _library_version(version):
    return tuple(int(x) for x in version.split(".")[:2])


def _core_files():
    core = os.environ.get(CORE_ENV)
    if not core:
        pytest.skip(f"{CORE_ENV} 미설정 — AirGlove-Client-Core 체크아웃 경로가 필요")
    replayer, capture = os.path.join(core, REPLAYER), os.path.join(core, CAPTURE)
    if not (os.path.exists(replayer) and os.path.exists(capture)):
        pytest.skip(f"{CORE_ENV}={core} 에 Spine 리플레이어/캡처 없음")
    if _library_version(Client.version()) < MIN_LIBRARY:
        pytest.skip(f"airglove_client {Client.version()} 는 Spine 프로토콜 3 이전 — "
                    "0.4.0 이상 라이브러리(AIRGLOVE_CLIENT_LIBRARY)가 필요")
    return replayer, capture


def test_library_version_parse():
    assert _library_version("0.1.0") < MIN_LIBRARY
    assert _library_version("0.3.0") < MIN_LIBRARY
    assert _library_version("0.4.0") >= MIN_LIBRARY
    assert _library_version("1.0.3") >= MIN_LIBRARY


def test_end_to_end_against_spine_replayer():
    replayer, capture = _core_files()
    port = _free_port_pair()
    rx = AirGloveReceiver(glove_port=port, client_port=port + 2)
    rx.start()
    proc = subprocess.Popen([sys.executable, replayer, capture, "--port", str(port + 2),
                             "--client-port", str(port)],
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    try:
        samples = {s: _wait(lambda s=s: (lambda x: x if x.tracked else None)(rx.get(s)), 5.0)
                   for s in ("left", "right")}
        for side, s in samples.items():
            assert s is not None, f"{side}: 리플레이어 프레임 수신 실패"
            xj = s.hand.xr_joints
            assert xj.shape == (26, 8) and np.all(np.isfinite(xj))
            assert np.allclose(xj[XR_WRIST, 0:4], [0, 0, 0, 1], atol=1e-5)
            assert np.allclose(xj[XR_WRIST, 4:7], 0.0, atol=1e-6)
            assert np.allclose(np.linalg.norm(xj[:, 0:4], axis=1), 1.0, atol=1e-4)
            tip = XR_JOINT_NAMES.index("middle_tip")
            assert 0.10 < np.linalg.norm(xj[tip, 4:7]) < 0.25
        assert _wait(lambda: rx.spine_protocol() == 3, 3.0)
        assert _wait(lambda: rx.connected("right") and rx.connected("left"), 3.0)
        assert rx.send_haptic("right", [0, 2, 9]) is True
        result = _wait(lambda: rx._client.haptics_result(Side.RIGHT), 2.0)
        assert result is not None and result[0] is True
        assert rx.send_haptic("right", []) is True
        time.sleep(0.3)
    finally:
        rx.stop()
        proc.terminate()
        try:
            out, _ = proc.communicate(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()
            out, _ = proc.communicate()
    assert "haptics right: [0, 2, 3, 0, 0]" in out
    assert "haptics right: [0, 0, 0, 0, 0]" in out


def test_unused_invalid_joint_keeps_frame_tracked(fake_client):
    """palm 하나만 무효면 추적 중 — 관절별 유효성은 xr_valid 로 넘긴다."""
    rx, c = _started(stale_timeout=0.2)
    frame = _frame(age=0.01)
    frame.valid[0] = False
    c.hands[Side.RIGHT] = frame
    s = rx.get("right")
    assert s.tracked and not s.hand.xr_valid[0] and s.hand.xr_valid[1:].all()


def test_invalid_used_joint_is_untracked(fake_client):
    """리타게팅이 쓰는 21점 중 하나라도 무효면 추적 아님(손목 등)."""
    rx, c = _started(stale_timeout=0.2)
    frame = _frame(age=0.01)
    frame.valid[1] = False  # 손목
    c.hands[Side.RIGHT] = frame
    assert rx.get("right").tracked is False
