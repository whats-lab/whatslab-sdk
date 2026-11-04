from __future__ import annotations


import numpy as np
import pytest

from conftest import require_sensor_urdf
from pythonosc.osc_message_builder import OscMessageBuilder

from whatslab.core.types import HandCommand, HandPose, InputSample
from whatslab.teleop.base import TeleopModel
from whatslab.teleop.models.glove import GloveModel


def _packet(address: str, *args) -> bytes:
    b = OscMessageBuilder(address=address)
    for a in args:
        b.add_arg(a)
    return b.build().dgram


def _send(disp, address, *args):
    disp.call_handlers_for_packet(_packet(address, *args), ("127.0.0.1", 0))


class _FakeHandCtrl:
    joint_names = ["f1", "f2"]

    def compute(self, sample: InputSample) -> HandCommand:
        if sample.hand is None or not sample.hand.tracked:
            return HandCommand(joint_names=self.joint_names, joint_angles=np.zeros(2))
        return HandCommand(joint_names=self.joint_names, joint_angles=np.array([0.5, 0.6]))


class _FakeSolver:

    def __init__(self):
        self.calls = 0

    def solve(self, T):
        self.calls += 1
        return np.array([T[0, 3], T[1, 3]])


class _FakeRig:
    class solver:
        reach_max = None

    class hand:
        retarget = "fake_hand"

    class calibration:
        input_reach = None
        enabled = True


class _FakeRobot:

    has_arm = True
    has_hand = True
    arm_joint_names = ["arm1", "arm2"]

    def __init__(self):
        self.solver = _FakeSolver()
        self.rig = _FakeRig()

    def to_base(self, T):
        return np.asarray(T, dtype=float)

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
    m = GloveModel(_FakeRobot(), arm_kwargs={"quest_port": 47360})
    m.hand_source = _FakeGlove()
    return m


def test_no_input_returns_empty_q():
    m = _make_model()
    q = m.get_q()
    assert q["right"] == {} and q["left"] == {}
    assert m.robot.solve_calls == 0


def test_start_stop_delegate_to_both_receivers():
    from whatslab.receiver.quest.controller import QuestControllerReceiver
    m = _make_model()
    m.arm_source = QuestControllerReceiver(quest_port=9759)
    assert set(m._receivers) == {m.hand_source, m.arm_source}
    m.start()
    assert m.hand_source.started
    m.stop()
    assert not m.hand_source.started


def test_hand_and_arm_combine_into_single_q():
    m = _make_model()

    arm_disp = m.arm_source._srv.dispatcher

    _send(arm_disp, "/controller/left/pos", 1.0, 2.0, 3.0)
    _send(arm_disp, "/controller/left/rot", 0.0, 0.0, 0.0, 1.0)

    m.hand_source.push("right")

    assert m.robot.solve_calls == 0

    q = m.get_q()["right"]
    assert m.robot.solve_calls == 1
    assert q["arm1"] == pytest.approx(3.02)
    assert q["arm2"] == pytest.approx(-1.04)
    assert q["f1"] == pytest.approx(0.5)
    assert q["f2"] == pytest.approx(0.6)


def test_new_controller_pose_reflected_in_q():
    m = _make_model()
    arm_disp = m.arm_source._srv.dispatcher
    m.hand_source.push("right")

    _send(arm_disp, "/controller/left/pos", 1.0, 2.0, 3.0)
    _send(arm_disp, "/controller/left/rot", 0.0, 0.0, 0.0, 1.0)
    assert m.get_q()["right"]["arm1"] == pytest.approx(3.02)

    _send(arm_disp, "/controller/left/pos", 9.0, 8.0, 7.0)
    assert m.get_q()["right"]["arm1"] == pytest.approx(7.02)


def test_crosshand_output_on_glove_side():
    m = _make_model()
    arm_disp = m.arm_source._srv.dispatcher

    _send(arm_disp, "/controller/left/pos", 1.0, 2.0, 3.0)
    _send(arm_disp, "/controller/left/rot", 0.0, 0.0, 0.0, 1.0)
    m.hand_source.push("right")

    q = m.get_q()
    assert q["right"]["arm1"] == pytest.approx(3.02)
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
    from scipy.spatial.transform import Rotation

    m = _make_model()
    assert m.calibrate_yaw()["right"] is False

    arm_disp = m.arm_source._srv.dispatcher
    _send(arm_disp, "/hmd/rot", *Rotation.from_euler("z", 0.3).as_quat().tolist())
    _send(arm_disp, "/controller/left/pos", 0.0, 0.0, 0.0)
    _send(arm_disp, "/controller/left/rot", *Rotation.from_euler("z", 0.1).as_quat().tolist())
    m.hand_source.push("right")

    out = m.calibrate_yaw()
    assert out["right"] is True and out["left"] is False
    assert m.sides["right"].calib.ready


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
            return {s: None for s in self.SIDES}

    duck = _FakeRobot()
    m = _Passthrough(duck)
    assert m.sides["right"].robot is duck
    assert m.sides["left"].robot is duck


def test_every_exported_preset_is_instantiable():
    require_sensor_urdf("orca_hand")
    pytest.importorskip("pinocchio")
    pytest.importorskip("onnxruntime")
    import whatslab.teleop as T

    from whatslab.receiver.quest.controller import QuestControllerReceiver
    from whatslab.receiver.quest.hand import QuestHandReceiver

    rig = "rigs/nero_orca_right.yaml"
    made = [T.QuestModel(rig), T.GloveModel(rig), T.HandModel()]
    for m in made:
        assert set(m.sides) >= set(m.SIDES)
        q = m.get_q()
        assert set(q) == set(m.SIDES)
        assert all(isinstance(v, dict) for v in q.values())
    assert made[2].sides["right"].retarget is not None
    assert made[0].hand_source is not None and made[1].hand_source is not None
    for r in (QuestControllerReceiver, QuestHandReceiver):
        assert r is not None
