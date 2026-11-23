import json
import time

import numpy as np
from scipy.spatial.transform import Rotation
from webxr_pose_receiver import PoseReceiver

from whatslab.core.constants import XR_JOINT_COUNT, XR_JOINT_FIELDS
from whatslab.receiver.base import Y_UP_RH_TO_CANONICAL
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


def test_receiver_gives_xr_joints_and_canonical_wrist():
    rx = WebXRHandReceiver(port=0, tls=False, pos_frame="room")
    j26 = _hand26(3)
    wrist_pos = np.array([0.1, 1.2, -0.5])
    j25 = _to_world25(j26, wrist_pos, [0.0, 0.0, 0.0, 1.0])
    rx._on_message({"hands": {"right": j25.reshape(-1).tolist()}})
    s = rx.get("right")
    assert s.tracked and s.hand.tracked
    assert np.allclose(s.hand.xr_joints[:, 4:7], j26[:, 4:7], atol=1e-9)
    assert s.hand.xr_valid.all()
    assert np.allclose(s.hand.wrist.pos, Y_UP_RH_TO_CANONICAL @ wrist_pos)
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


def test_controller_and_hand_receivers_share_one_port_over_socket(free_port):
    hands = WebXRHandReceiver(port=free_port, tls=False)
    hands.start()
    ctrl = PoseReceiver(port=free_port, tls=False)
    ctrl.start()
    try:
        sock = _handshake(hands.port)
        j25 = _to_world25(_hand26(5), np.array([0.0, 1.0, -0.3]), [0.0, 0.0, 0.0, 1.0])
        msg = {
            "hmd": {"pos": [0.0, 1.6, 0.0], "quat": [0.0, 0.0, 0.0, 1.0]},
            "left": {"pos": [0.1, 1.2, -0.5], "quat": [0.0, 0.0, 0.0, 1.0]},
            "hands": {"right": j25.reshape(-1).tolist()},
        }
        sock.sendall(_mask(json.dumps(msg).encode()))
        assert _wait(lambda: hands.get("right").tracked and ctrl.get("left").tracked)
        assert hands.get("right").hand.xr_joints.shape == (XR_JOINT_COUNT, XR_JOINT_FIELDS)
        assert ctrl.get("left").controller is not None
        sock.close()
    finally:
        ctrl.stop()
        hands.stop()


def test_page_requests_hand_tracking():
    import pathlib

    from whatslab.receiver import webxr

    html = (pathlib.Path(webxr.__file__).parent / "static" / "index.html").read_text(
        encoding="utf-8"
    )
    assert '"hand-tracking"' in html and "getJointPose" in html and "msg.hands" in html


def test_stopped_receiver_no_longer_gets_messages(free_port):
    first = WebXRHandReceiver(port=free_port, tls=False)
    first.start()
    second = WebXRHandReceiver(port=free_port, tls=False)
    second.start()
    first.stop()
    srv = second._srv
    assert first._on_message not in srv._handlers and second._on_message in srv._handlers
    second.stop()
    assert srv._handlers == []


def test_one_failing_handler_does_not_block_others(free_port):
    hands = WebXRHandReceiver(port=free_port, tls=False)
    hands.start()
    ctrl = PoseReceiver(port=free_port, tls=False)
    ctrl.start()
    try:
        j25 = _to_world25(_hand26(), np.zeros(3), [0.0, 0.0, 0.0, 1.0])
        bad = {
            "left": {"pos": 1.0, "quat": [0.0, 0.0, 0.0, 1.0]},  # 컨트롤러 수신기가 예외를 낸다
            "hands": {"right": j25.reshape(-1).tolist()},
        }
        hands._srv._dispatch(json.dumps(bad).encode())
        assert hands.get("right").tracked
    finally:
        ctrl.stop()
        hands.stop()


def test_controller_goes_stale_by_default():
    ctrl = PoseReceiver(port=0, tls=False)
    ctrl._on_message({"right": {"pos": [0.0, 1.0, 0.0], "quat": [0.0, 0.0, 0.0, 1.0]}})
    assert ctrl.get("right").tracked
    time.sleep(0.25)
    assert not ctrl.get("right").tracked


def test_hmd_yaw_frame_rotates_wrist_orientation_like_position():
    from whatslab.receiver.webxr.base import yaw_rz

    rx = WebXRHandReceiver(port=0, tls=False, pos_frame="hmd_yaw")
    room = WebXRHandReceiver(port=0, tls=False, pos_frame="room")
    hmd_quat = Rotation.from_euler("y", 90, degrees=True).as_quat()  # WebXR y-up: 고개를 90° 돌림
    wrist_rot = Rotation.from_euler("xyz", [0.2, 0.5, -0.3]).as_quat()
    j25 = _to_world25(_hand26(), np.array([0.3, 1.1, -0.4]), wrist_rot)
    msg = {
        "hmd": {"pos": [0.0, 1.6, 0.0], "quat": hmd_quat.tolist()},
        "hands": {"right": j25.reshape(-1).tolist()},
    }
    rx._on_message(msg)
    room._on_message(msg)
    _, canon_hmd, _ = rx.get_hmd()
    rot_z = yaw_rz(canon_hmd)
    got = Rotation.from_quat(rx.get("right").hand.wrist.quat).as_matrix()
    ref = Rotation.from_quat(room.get("right").hand.wrist.quat).as_matrix()
    assert np.allclose(got, rot_z @ ref, atol=1e-9)
    assert not np.allclose(got, ref, atol=1e-3)


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


def test_pos_frame_is_keyword_only_on_base():
    """기반 클래스의 위치 인자 순서는 (port, listen_ip, tls, certfile, keyfile) 그대로다."""
    import inspect

    from whatslab.receiver.webxr import WebXRReceiverBase

    params = inspect.signature(WebXRReceiverBase.__init__).parameters
    assert list(params)[1:6] == ["port", "listen_ip", "tls", "certfile", "keyfile"]
    assert params["pos_frame"].kind is inspect.Parameter.KEYWORD_ONLY


def test_start_failure_of_any_kind_detaches_handler(monkeypatch):
    """bind 외의 실패(예: 인증서 생성 RuntimeError)에도 핸들러를 뗀다."""
    rx = WebXRHandReceiver(port=0, tls=False)

    def boom():
        raise RuntimeError("openssl 없음")

    monkeypatch.setattr(rx._srv, "start", boom)
    try:
        rx.start()
    except RuntimeError:
        pass
    else:
        raise AssertionError("start 가 실패해야 한다")
    assert rx._on_message not in rx._srv._handlers
