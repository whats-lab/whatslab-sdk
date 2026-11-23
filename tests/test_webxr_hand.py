import json
import time

import numpy as np
from scipy.spatial.transform import Rotation

from whatslab.core.constants import XR_JOINT_COUNT, XR_JOINT_FIELDS
from whatslab.receiver.webxr import (
    WebXRHandReceiver,
    webxr_hand_to_openxr26,
)
from whatslab.receiver.webxr.hand import WEBXR_MIDDLE_METACARPAL, WEBXR_MIDDLE_PROXIMAL

from test_webxr_receiver import _handshake, _mask, _wait


def _hand26(seed=0):
    """손목 기준 OpenXR 26관절 (palm 은 중지 중수골·근위 중점, 방향 = 중수골)."""
    rng = np.random.default_rng(seed)
    j = np.zeros((XR_JOINT_COUNT, XR_JOINT_FIELDS))
    j[:, 0:4] = Rotation.random(XR_JOINT_COUNT, random_state=seed).as_quat()
    j[:, 4:7] = rng.normal(scale=0.05, size=(XR_JOINT_COUNT, 3))
    j[:, 7] = rng.uniform(0.005, 0.02, size=XR_JOINT_COUNT)
    j[1, 0:4] = (0.0, 0.0, 0.0, 1.0)
    j[1, 4:7] = 0.0
    mc, px = 1 + WEBXR_MIDDLE_METACARPAL, 1 + WEBXR_MIDDLE_PROXIMAL
    j[0, 0:4] = j[mc, 0:4]
    j[0, 4:7] = 0.5 * (j[mc, 4:7] + j[px, 4:7])
    j[0, 7] = j[mc, 7]
    return j


def _to_world25(j26, wrist_pos, wrist_rot):
    """손목 기준 26관절 → 세션 좌표의 WebXR 25관절 (palm 제외)."""
    w = Rotation.from_quat(wrist_rot)
    out = np.zeros((25, 8))
    rot = w * Rotation.from_quat(j26[1:, 0:4])
    out[:, 0:4] = rot.as_quat()
    out[:, 4:7] = w.apply(j26[1:, 4:7]) + wrist_pos
    out[:, 7] = j26[1:, 7]
    return out


def _same_rotations(a, b):
    return np.allclose(
        (Rotation.from_quat(a).inv() * Rotation.from_quat(b)).magnitude(), 0.0, atol=1e-6
    )


def test_webxr25_to_openxr26_recovers_wrist_relative_joints():
    j26 = _hand26()
    wrist_rot = Rotation.from_euler("xyz", [0.3, -0.7, 1.1]).as_quat()
    j25 = _to_world25(j26, np.array([0.2, 1.1, -0.4]), wrist_rot)
    got = webxr_hand_to_openxr26(j25)
    assert got.shape == (XR_JOINT_COUNT, XR_JOINT_FIELDS)
    assert np.allclose(got[:, 4:7], j26[:, 4:7], atol=1e-9)
    assert _same_rotations(got[:, 0:4], j26[:, 0:4])
    assert np.allclose(got[:, 7], j26[:, 7])
    assert np.allclose(got[1], [0, 0, 0, 1, 0, 0, 0, j26[1, 7]])


def test_non_finite_hand_is_rejected():
    j25 = _to_world25(_hand26(), np.zeros(3), [0.0, 0.0, 0.0, 1.0])
    j25[3, 5] = np.nan
    assert webxr_hand_to_openxr26(j25) is None
    j25 = _to_world25(_hand26(), np.zeros(3), [0.0, 0.0, 0.0, 1.0])
    j25[0, 0:4] = 0.0
    assert webxr_hand_to_openxr26(j25) is None


def test_receiver_gives_wrist_relative_xr_joints():
    rx = WebXRHandReceiver(port=0, tls=False)
    j26 = _hand26(3)
    wrist_pos = np.array([0.1, 1.2, -0.5])
    j25 = _to_world25(j26, wrist_pos, [0.0, 0.0, 0.0, 1.0])
    rx._on_message({"hands": {"right": j25.reshape(-1).tolist()}})
    s = rx.get("right")
    assert s.tracked and s.hand.tracked
    assert np.allclose(s.hand.xr_joints[:, 4:7], j26[:, 4:7], atol=1e-9)
    assert s.hand.xr_valid.all()
    assert not rx.get("left").tracked and rx.get("left").hand is None


def test_bad_hand_payloads_are_ignored():
    rx = WebXRHandReceiver(port=0, tls=False)
    rx._on_message({"hands": {"right": [0.0] * 10}})
    rx._on_message({"hands": {"right": "x"}})
    rx._on_message({"hands": [1, 2]})
    assert rx.get("right").hand is None


def test_hand_goes_stale():
    rx = WebXRHandReceiver(port=0, tls=False, stale_timeout=0.05)
    j25 = _to_world25(_hand26(), np.zeros(3), [0.0, 0.0, 0.0, 1.0])
    rx._on_message({"hands": {"left": j25.reshape(-1).tolist()}})
    assert rx.get("left").tracked
    time.sleep(0.08)
    assert not rx.get("left").tracked


def test_hand_roundtrip_over_socket():
    hands = WebXRHandReceiver(port=0, tls=False)
    hands.start()
    try:
        sock = _handshake(hands.port)
        j25 = _to_world25(_hand26(5), np.array([0.0, 1.0, -0.3]), [0.0, 0.0, 0.0, 1.0])
        msg = {"t": 1, "hands": {"right": j25.reshape(-1).tolist()}}
        sock.sendall(_mask(json.dumps(msg).encode()))
        assert _wait(lambda: hands.get("right").tracked)
        assert hands.get("right").hand.xr_joints.shape == (XR_JOINT_COUNT, XR_JOINT_FIELDS)
        sock.close()
    finally:
        hands.stop()


def test_page_requests_hand_tracking():
    import pathlib

    from whatslab.receiver import webxr

    html = (pathlib.Path(webxr.__file__).parent / "static" / "index.html").read_text(
        encoding="utf-8"
    )
    assert '"hand-tracking"' in html and "getJointPose" in html and "hands: hands" in html


def test_receiver_start_stop_is_idempotent():
    rx = WebXRHandReceiver(port=0, tls=False)
    rx.start()
    rx.start()
    srv = rx._srv
    rx.stop()
    assert not srv.is_running
    rx.stop()


def test_failed_start_leaves_no_server_reference():
    import socket

    blocker = socket.socket()
    blocker.bind(("0.0.0.0", 0))
    blocker.listen(1)
    port = blocker.getsockname()[1]
    rx = WebXRHandReceiver(port=port, tls=False)
    try:
        try:
            rx.start()
        except OSError:
            pass
        else:
            raise AssertionError("포트가 점유돼 있으면 start 가 실패해야 한다")
    finally:
        blocker.close()
    rx.start()
    srv = rx._srv
    assert srv.is_running
    rx.stop()
    assert not srv.is_running


def test_webxr_path_imports_without_airglove_client():
    """선택 의존성(airglove-client·dex·pinocchio) 없이 WebXR 경로와 패키지 star import 가 된다."""
    import subprocess
    import sys

    code = (
        "import sys; sys.modules['airglove_client'] = None\n"
        "from whatslab.teleop import HandModel\n"
        "from whatslab.receiver import WebXRHandReceiver\n"
        "from whatslab.receiver import *\n"
        "from whatslab.teleop import *\n"
        "sys.modules['dex_retargeting'] = None; sys.modules['pinocchio'] = None\n"
        "from whatslab.solvers.hand import *\n"
    )
    proc = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True,
                          timeout=60)
    assert proc.returncode == 0, proc.stderr
