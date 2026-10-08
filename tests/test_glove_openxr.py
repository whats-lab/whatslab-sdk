from __future__ import annotations

import os
import socket
import subprocess
import sys
import time

import numpy as np
import pytest
from airglove_client import AirGloveError, Client, DeviceStatus, HandFrame, Side, WristFrame
from pythonosc.osc_message_builder import OscMessageBuilder
from scipy.spatial.transform import Rotation

import whatslab.receiver as receiver_pkg
import whatslab.receiver.glove as glove_pkg
from whatslab.core.interfaces import Receiver
from whatslab.receiver.glove import openxr as oxr
from whatslab.receiver.glove.human_hand import GloveHumanAnglesReceiver
from whatslab.receiver.glove.openxr import (XR_JOINT_COUNT, XR_JOINT_FIELDS, XR_JOINT_NAMES,
                                            XR_WRIST_INDEX, GloveOpenXRReceiver, haptic_levels,
                                            openxr_quat_to_canonical)
from whatslab.receiver.webxr import WebXRControllerReceiver
from whatslab.receiver.webxr.base import _CANONICAL_M
from whatslab.teleop.models.glove import HAND_SOURCES, GloveModel

CORE_ENV = "AIRGLOVE_CORE_DIR"
REPLAYER = "tools/spine_replay.py"
CAPTURE = "captures/spine_both_192s.agrec"
MIN_LIBRARY = (0, 3)
TEST_PORTS = range(47500, 47598)


def _frame(age=0.01, valid=True, seed=0):
    rng = np.random.default_rng(seed)
    ori = Rotation.random(XR_JOINT_COUNT, random_state=seed).as_quat().astype(np.float32)
    ori[XR_WRIST_INDEX] = [0.0, 0.0, 0.0, 1.0]
    pos = rng.normal(scale=0.05, size=(XR_JOINT_COUNT, 3)).astype(np.float32)
    pos[XR_WRIST_INDEX] = 0.0
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
        _FakeClient.instances.append(self)

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

    def close(self):
        self.closed = True


@pytest.fixture
def fake_client(monkeypatch):
    _FakeClient.instances = []
    monkeypatch.setattr(oxr, "Client", _FakeClient)
    return _FakeClient


def _started(**kw):
    rx = GloveOpenXRReceiver(**kw)
    rx.start()
    return rx, _FakeClient.instances[-1]


def test_exports_and_protocol():
    assert receiver_pkg.GloveOpenXRReceiver is GloveOpenXRReceiver
    assert glove_pkg.GloveOpenXRReceiver is GloveOpenXRReceiver
    assert isinstance(GloveOpenXRReceiver(), Receiver)
    assert XR_JOINT_NAMES[0] == "palm" and XR_JOINT_NAMES[XR_WRIST_INDEX] == "wrist"
    assert len(XR_JOINT_NAMES) == XR_JOINT_COUNT == 26


def test_ports_map_to_client_and_stale_threshold(fake_client):
    rx, c = _started(glove_port=47040, client_port=47042, target_ip="10.0.0.2",
                     stale_timeout=0.3)
    assert c.args == ("127.0.0.1", 47040, "10.0.0.2", 47042)
    assert c.stale == pytest.approx(0.3)
    rx.start()
    assert len(fake_client.instances) == 1
    rx.stop()
    assert c.closed and not rx.running


def test_get_before_start_is_untracked_without_binding(fake_client):
    rx = GloveOpenXRReceiver()
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
    assert s.hand.joint_angles == {}
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


def test_wrist_converted_to_canonical_and_held_when_stale(fake_client):
    rx, c = _started()
    q = Rotation.from_euler("xyz", [0.3, -0.7, 1.1]).as_quat()
    c.wrists[Side.LEFT] = WristFrame(q.astype(np.float32), 3, 1, 5.0)
    s = rx.get("left")
    assert s.tracked is False
    assert s.hand.xr_joints is None
    expect = Rotation.from_matrix(_CANONICAL_M @ Rotation.from_quat(q).as_matrix()
                                  @ _CANONICAL_M.T)
    got = Rotation.from_quat(s.hand.wrist.quat)
    assert (got.inv() * expect).magnitude() < 1e-5


def test_degenerate_wrist_is_dropped(fake_client):
    rx, c = _started()
    c.wrists[Side.RIGHT] = WristFrame(np.zeros(4, np.float32), 1, 1, 0.0)
    c.hands[Side.RIGHT] = _frame()
    s = rx.get("right")
    assert s.hand.wrist is None and s.tracked is True


def test_glove_wrist_matches_webxr_conversion():
    webxr = WebXRControllerReceiver(port=0, tls=False)
    for seed in range(20):
        q = Rotation.random(random_state=seed).as_quat()
        webxr._set_side("right", {"pos": [0.0, 0.0, 0.0], "quat": q.tolist()})
        w = webxr.get("right").controller.quat
        g = openxr_quat_to_canonical(q)
        assert (Rotation.from_quat(w).inv() * Rotation.from_quat(g)).magnitude() < 1e-9


def test_openxr_axes_land_on_canonical_axes():
    up = openxr_quat_to_canonical(Rotation.identity().as_quat())
    assert np.allclose(Rotation.from_quat(up).as_matrix(), np.eye(3))
    yaw = Rotation.from_rotvec([0.0, 0.4, 0.0]).as_quat()
    R = Rotation.from_quat(openxr_quat_to_canonical(yaw))
    assert np.allclose(R.as_rotvec(), [0.0, 0.0, 0.4])


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


def test_connected_ignores_old_device_status(fake_client):
    rx, c = _started()
    c.status = DeviceStatus(True, True, 1.9)
    assert rx.connected("right") is True
    c.status = DeviceStatus(True, True, 2.5)
    assert rx.connected("right") is False
    c.hands[Side.RIGHT] = _frame(age=0.05)
    assert rx.connected("right") is True


class _FakeRig:
    class solver:
        reach_max = None

    hand = None

    class calibration:
        input_reach = None
        enabled = True


class _FakeRobot:
    has_arm = True
    has_hand = False
    arm_joint_names = ["arm1", "arm2"]

    def __init__(self):
        self.rig = _FakeRig()

    class solver:
        @staticmethod
        def solve(T):
            return np.array([T[0, 3], T[1, 3]])

    def to_base(self, T):
        return np.asarray(T, dtype=float)


def _send(disp, address, *args):
    b = OscMessageBuilder(address=address)
    for a in args:
        b.add_arg(a)
    disp.call_handlers_for_packet(b.build().dgram, ("127.0.0.1", 0))


def test_glove_model_defaults_to_openxr_with_receiver_kwargs(fake_client):
    assert "openxr" in HAND_SOURCES
    m = GloveModel(_FakeRobot(), hand_kwargs={"glove_port": 47140, "client_port": 47142},
                   arm_kwargs={"quest_port": 47160})
    assert isinstance(m.hand_source, GloveOpenXRReceiver)
    m.hand_source.start()
    c = fake_client.instances[-1]
    assert c.args[1] == 47140 and c.args[3] == 47142

    q = Rotation.from_euler("zyx", [0.4, 0.2, -0.3]).as_quat()
    c.wrists[Side.RIGHT] = WristFrame(q.astype(np.float32), 1, 1, 0.01)
    c.hands[Side.RIGHT] = _frame()
    _send(m.arm_source._srv.dispatcher, "/controller/left/pos", 1.0, 2.0, 3.0)
    _send(m.arm_source._srv.dispatcher, "/controller/left/rot", 0.0, 0.0, 0.0, 1.0)

    data = m.get_data()["right"]
    assert data["fingers"].hand.xr_joints.shape == (26, 8)
    got = Rotation.from_quat(data["arm_pose"].quat)
    want = Rotation.from_quat(openxr_quat_to_canonical(q))
    assert (got.inv() * want).magnitude() < 1e-6
    assert data["tracked"] is True
    m.send_feedback({"side": "right", "forces": [0, 1, 2, 3, 9]})
    assert c.haptics == [(Side.RIGHT, [0, 1, 2, 3, 3])]
    m.stop()
    assert c.closed


def test_glove_model_legacy_source_still_selectable():
    m = GloveModel(_FakeRobot(), hand_source="angles",
                   hand_kwargs={"glove_port": 47240}, arm_kwargs={"quest_port": 47260})
    assert isinstance(m.hand_source, GloveHumanAnglesReceiver)
    with pytest.raises(ValueError):
        GloveModel(_FakeRobot(), hand_source="nope")


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
        pytest.skip(f"airglove_client {Client.version()} 는 Spine 프로토콜 v2 이전 — "
                    "0.3.0 이상 라이브러리(AIRGLOVE_CLIENT_LIBRARY)가 필요")
    return replayer, capture


def test_library_version_parse():
    assert _library_version("0.1.0") < MIN_LIBRARY
    assert _library_version("0.2.0") < MIN_LIBRARY
    assert _library_version("0.3.0") >= MIN_LIBRARY
    assert _library_version("1.0.3") >= MIN_LIBRARY


def test_end_to_end_against_spine_replayer():
    replayer, capture = _core_files()
    port = _free_port_pair()
    rx = GloveOpenXRReceiver(glove_port=port, client_port=port + 2)
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
            assert np.allclose(xj[XR_WRIST_INDEX, 0:4], [0, 0, 0, 1], atol=1e-5)
            assert np.allclose(xj[XR_WRIST_INDEX, 4:7], 0.0, atol=1e-6)
            assert np.allclose(np.linalg.norm(xj[:, 0:4], axis=1), 1.0, atol=1e-4)
            tip = XR_JOINT_NAMES.index("middle_tip")
            assert 0.10 < np.linalg.norm(xj[tip, 4:7]) < 0.25
            assert s.hand.wrist is not None
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
