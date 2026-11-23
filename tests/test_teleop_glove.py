from __future__ import annotations

import threading
import time
from types import SimpleNamespace

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from whatslab.core.constants import SIDES
from whatslab.core.types import HandCommand, HandPose, InputSample
from whatslab.receiver.base import Y_UP_RH_TO_CANONICAL
from whatslab.teleop.base import TeleopModel
from whatslab.teleop.models.glove import GloveModel

from conftest import require_sensor_urdf


def _ctrl(receiver, side, pos=None, quat=None, hmd_quat=None):
    """WebXR 컨트롤러 프레임을 정준 좌표로 지정해 넣는다 (WebXR 좌표 = Mᵀ · 정준)."""
    basis = Y_UP_RH_TO_CANONICAL
    msg = {}
    if hmd_quat is not None:
        rot = basis.T @ Rotation.from_quat(hmd_quat).as_matrix() @ basis
        msg["hmd"] = {"pos": [0.0, 0.0, 0.0], "quat": Rotation.from_matrix(rot).as_quat().tolist()}
    if pos is not None:
        q = [0.0, 0.0, 0.0, 1.0] if quat is None else quat
        rot = basis.T @ Rotation.from_quat(q).as_matrix() @ basis
        msg[side] = {"pos": (basis.T @ np.asarray(pos, dtype=float)).tolist(),
                     "quat": Rotation.from_matrix(rot).as_quat().tolist()}
    receiver._on_message(msg)


class _FakeHandCtrl:
    joint_names = ["f1", "f2"]

    def compute(self, sample: InputSample) -> HandCommand:
        if sample.hand is None or not sample.hand.tracked:
            return HandCommand(joint_names=self.joint_names, joint_angles=np.zeros(2))
        return HandCommand(joint_names=self.joint_names, joint_angles=np.array([0.5, 0.6]))


class _FakeSolver:

    def __init__(self):
        self.calls = 0

    def solve(self, tf):
        self.calls += 1
        return np.array([tf[0, 3], tf[1, 3]])


class _FakeRig:
    solver = SimpleNamespace(reach_max=None)
    hand = SimpleNamespace(retarget="fake_hand")
    calibration = SimpleNamespace(input_reach=None, enabled=True)


class _FakeRobot:

    has_arm = True
    has_hand = True
    arm_joint_names = ["arm1", "arm2"]

    def __init__(self):
        self.solver = _FakeSolver()
        self.rig = _FakeRig()

    def to_base(self, tf):
        return np.asarray(tf, dtype=float)

    def make_hand_controller(self, config_name, side):
        return _FakeHandCtrl()

    @property
    def solve_calls(self):
        return self.solver.calls


class _FakeGlove:

    def __init__(self):
        self.tracked = {"left": False, "right": False}
        self.haptics = []
        self.started = False

    def push(self, side):
        self.tracked[side] = True

    def get(self, side):
        if not self.tracked[side]:
            return InputSample(tracked=False, timestamp=0.0)
        hand = HandPose(wrist=None, xr_joints=np.zeros((26, 8)), tracked=True, timestamp=1.0)
        return InputSample(hand=hand, tracked=True, timestamp=1.0)

    def send_haptic(self, side, values):
        self.haptics.append((side, list(values)))
        return True

    def start(self):
        self.started = True

    def stop(self):
        self.started = False


def _make_model():
    m = GloveModel(_FakeRobot(), arm_kwargs={"port": 47360, "tls": False})
    m.hand_source = _FakeGlove()
    return m


def test_no_input_returns_empty_q():
    m = _make_model()
    q = m.get_q()
    assert q["right"] == {} and q["left"] == {}
    assert m.robot.solve_calls == 0


def test_start_stop_delegate_to_both_receivers():
    from whatslab.receiver.webxr.controller import WebXRControllerReceiver
    m = _make_model()
    m.arm_source = WebXRControllerReceiver(port=9759, tls=False)
    assert set(m._receivers) == {m.hand_source, m.arm_source}
    m.start()
    assert m.hand_source.started
    m.stop()
    assert not m.hand_source.started


def test_hand_and_arm_combine_into_single_q():
    m = _make_model()


    _ctrl(m.arm_source, "left", pos=(3.0, -1.0, 2.0))

    m.hand_source.push("right")

    assert m.robot.solve_calls == 0

    q = m.get_q()["right"]
    assert m.robot.solve_calls == 1
    assert q["arm1"] == pytest.approx(3.0)
    assert q["arm2"] == pytest.approx(-1.0)
    assert q["f1"] == pytest.approx(0.5)
    assert q["f2"] == pytest.approx(0.6)


def test_new_controller_pose_reflected_in_q():
    m = _make_model()
    m.hand_source.push("right")

    _ctrl(m.arm_source, "left", pos=(3.0, -1.0, 2.0))
    assert m.get_q()["right"]["arm1"] == pytest.approx(3.0)

    _ctrl(m.arm_source, "left", pos=(7.0, -9.0, 8.0))
    assert m.get_q()["right"]["arm1"] == pytest.approx(7.0)


def test_crosshand_output_on_glove_side():
    m = _make_model()

    _ctrl(m.arm_source, "left", pos=(3.0, -1.0, 2.0))
    m.hand_source.push("right")

    q = m.get_q()
    assert q["right"]["arm1"] == pytest.approx(3.0)
    assert q["right"]["f1"] == pytest.approx(0.5)
    assert q["left"] == {}


def test_arm_omitted_when_controller_untracked_hand_still_retargets():
    m = _make_model()

    m.hand_source.push("right")

    q = m.get_q()["right"]
    assert "arm1" not in q and "arm2" not in q
    assert q["f1"] == pytest.approx(0.5)
    assert q["f2"] == pytest.approx(0.6)
    assert m.robot.solve_calls == 0


def test_joint_q_bypass_skips_ik_and_retarget():
    m = _make_model()
    direct = {"custom_joint": 1.23}

    class _JointQSource:
        def get(self, side):
            return InputSample(joint_q=direct, tracked=True, timestamp=1.0)

    m.hand_source = _JointQSource()

    q = m.get_q()["right"]
    assert q == direct
    assert m.robot.solve_calls == 0


def test_calibrate_yaw_captures_arm_target_side():
    m = _make_model()
    assert m.calibrate_yaw()["right"] is False

    _ctrl(m.arm_source, "left", pos=(0.0, 0.0, 0.0), quat=Rotation.from_euler("x", 0.1).as_quat(),
          hmd_quat=Rotation.from_euler("x", 0.3).as_quat())
    m.hand_source.push("right")

    out = m.calibrate_yaw()
    assert out["right"] is True and out["left"] is False
    assert m.sides["right"].calib.ready


@pytest.mark.parametrize("pos_frame, removes_yaw", [("hmd_yaw", True), ("room", False),
                                                    ("hmd", False)])
def test_arm_orientation_follows_pos_frame(pos_frame, removes_yaw):
    """방위는 위치와 같은 기준 — 머리 yaw 는 hmd_yaw 일 때만 뺀다."""
    m = GloveModel(_FakeRobot(), arm_kwargs={"port": 47361, "tls": False,
                                              "pos_frame": pos_frame})
    m.hand_source = _FakeGlove()
    m.hand_source.push("right")
    _ctrl(m.arm_source, "left", pos=(0.3, 0.0, 0.0),
          hmd_quat=Rotation.from_euler("z", 0.7).as_quat())
    m.get_q()
    yaw = Rotation.from_quat(m.sides["right"].raw_target.quat).as_euler("xyz")[2]
    assert yaw == pytest.approx(-0.7 if removes_yaw else 0.0, abs=1e-9)


def test_send_feedback_forwards_to_glove():
    m = _make_model()
    m.send_feedback({"side": "right", "forces": [10, 20, 30, 40, 50]})
    m.send_feedback([1, 2, 3, 4, 5])
    assert m.hand_source.haptics == [("right", [10, 20, 30, 40, 50]), ("right", [1, 2, 3, 4, 5])]


def test_send_feedback_ignores_empty_or_none():
    m = _make_model()
    m.send_feedback(None)
    m.send_feedback({"side": "right"})
    m.send_feedback({"side": "right", "forces": []})


def test_teleop_model_still_accepts_duck_typed_robot():
    class _Passthrough(TeleopModel):
        def _get_raw_target(self):
            return {s: None for s in SIDES}

    duck = _FakeRobot()
    m = _Passthrough(duck)
    assert m.sides["right"].robot is duck
    assert m.sides["left"].robot is duck


def test_every_exported_preset_is_instantiable():
    require_sensor_urdf("orca_hand")
    pytest.importorskip("pinocchio")
    pytest.importorskip("dex_retargeting")
    import whatslab.teleop
    from whatslab.receiver.webxr import WebXRControllerReceiver, WebXRHandReceiver

    rig = "rigs/nero_orca_right.yaml"
    made = [
        whatslab.teleop.QuestModel(rig),
        whatslab.teleop.GloveModel(rig),
        whatslab.teleop.HandModel(),
    ]
    for m in made:
        assert set(m.sides) >= set(SIDES)
        q = m.get_q()
        assert set(q) == set(SIDES)
        assert all(isinstance(v, dict) for v in q.values())
    assert made[2].sides["right"].retarget is not None
    assert made[0].hand_source is not None and made[1].hand_source is not None
    for r in (WebXRControllerReceiver, WebXRHandReceiver):
        assert r is not None


def test_retarget_reset_once_when_hand_tracking_is_lost():
    m = _make_model()
    resets = []
    m.sides["right"].retarget.reset = lambda: resets.append(1)
    m.hand_source.push("right")
    m.get_q()
    m.hand_source.tracked["right"] = False
    m.get_q()
    m.get_q()
    assert resets == [1]


def test_stale_controller_is_not_an_arm_target():
    m = _make_model()
    m.arm_source._stale_timeout = 0.05
    _ctrl(m.arm_source, "left", pos=(3.0, -1.0, 2.0))
    m.hand_source.push("right")
    assert "arm1" in m.get_q()["right"]
    time.sleep(0.08)
    q = m.get_q()["right"]
    assert "arm1" not in q and q["f1"] == pytest.approx(0.5)


def test_tick_samples_are_per_thread():
    """다른 스레드의 get_data() 가 끝나도 이 스레드의 틱 샘플은 그대로다."""
    m = _make_model()
    m.hand_source.push("right")
    outer = []

    def raw_target():
        if threading.current_thread() is threading.main_thread():
            mine = m.samples()
            worker = threading.Thread(target=m.get_data)
            worker.start()
            worker.join()
            outer.append(m.samples() is mine)
        return {s: None for s in SIDES}

    m._get_raw_target = raw_target
    m.get_data()
    assert outer == [True]


def test_calibration_waits_for_running_tick():
    """캘리브(capture·reseed)는 진행 중인 틱의 IK 가 끝난 뒤에 들어간다."""
    m = _make_model()
    _ctrl(m.arm_source, "left", pos=(0.3, 0.0, 0.0),
          hmd_quat=Rotation.from_euler("z", 0.0).as_quat())
    m.hand_source.push("right")
    in_solve, release, order = threading.Event(), threading.Event(), []
    solve = m.robot.solver.solve

    def slow_solve(tf):
        in_solve.set()
        release.wait(5.0)
        order.append("solve")
        return solve(tf)

    m.robot.solver.solve = slow_solve
    tick = threading.Thread(target=m.get_q)
    tick.start()
    assert in_solve.wait(5.0)
    calib = threading.Thread(target=lambda: order.append(("calib", m.calibrate_yaw()["right"])))
    calib.start()
    calib.join(0.2)
    assert calib.is_alive()  # 틱이 끝날 때까지 기다린다
    release.set()
    tick.join(5.0)
    calib.join(5.0)
    assert order == ["solve", ("calib", True)]


def test_calibration_does_not_publish_raw_target():
    """캘리브 스레드의 샘플이 메인 루프의 raw_target(진단·기록용)을 덮지 않는다."""
    m = _make_model()
    m.hand_source.push("right")
    _ctrl(m.arm_source, "left", pos=(0.3, 0.0, 0.0),
          hmd_quat=Rotation.from_euler("z", 0.0).as_quat())
    m.calibrate_yaw()
    assert m.sides["right"].raw_target is None
    m.get_q()
    assert m.sides["right"].raw_target is not None


def test_start_rolls_back_when_a_receiver_fails():
    """뒤 수신기 시작이 실패하면 먼저 시작한 수신기를 멈춘다(포트를 붙잡지 않게)."""
    m = _make_model()

    def fail():
        raise OSError("포트 사용 중")

    m.hand_source.start = fail  # 순서는 arm_source → hand_source
    with pytest.raises(OSError):
        m.start()
    assert not m.arm_source._srv.is_running


def test_failed_restart_stops_receivers_before_failure():
    """다시 부른 start() 가 실패하면 그 앞의 수신기는 이미 돌고 있었어도 멈춘다."""
    m = _make_model()
    m.start()
    assert m.arm_source.running

    def fail():
        raise OSError("포트 사용 중")

    m.hand_source.start = fail
    with pytest.raises(OSError):
        m.start()
    assert not m.arm_source.running and not m.arm_source._srv.is_running


def test_start_restarts_receiver_stopped_elsewhere():
    """밖에서 멈춘 수신기는 다음 start() 가 다시 시작한다."""
    m = _make_model()
    m.start()
    m.arm_source.stop()
    m.start()
    try:
        assert m.arm_source.running
    finally:
        m.stop()
