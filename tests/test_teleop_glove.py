from __future__ import annotations


import numpy as np
import pytest
from pythonosc.osc_message_builder import OscMessageBuilder

from whatslab.core.types import HandCommand, InputSample
from whatslab.teleop.base import TeleopModel
from whatslab.teleop.models.glove import GloveModel
from whatslab.receiver.glove.robot_hand import GloveRobotHandReceiver


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


def _make_model():
    return GloveModel(_FakeRobot())


def test_no_input_returns_empty_q():
    m = _make_model()
    q = m.get_q()
    assert q["right"] == {} and q["left"] == {}
    assert m.robot.solve_calls == 0


def test_start_stop_delegate_to_both_receivers():
    from whatslab.receiver.glove.human_hand import GloveHumanHandReceiver
    from whatslab.receiver.quest.controller import QuestControllerReceiver
    m = _make_model()
    m.hand_source = GloveHumanHandReceiver(glove_port=4759)
    m.arm_source = QuestControllerReceiver(quest_port=9759)
    assert set(m._receivers) == {m.hand_source, m.arm_source}
    m.start()
    m.stop()


def test_hand_and_arm_combine_into_single_q():
    m = _make_model()

    hand_disp = m.hand_source._srv.dispatcher
    arm_disp = m.arm_source._srv.dispatcher

    _send(arm_disp, "/controller/left/pos", 1.0, 2.0, 3.0)
    _send(arm_disp, "/controller/left/rot", 0.0, 0.0, 0.0, 1.0)

    raw = np.zeros(72, dtype=np.float32)
    raw[3::4] = 1.0
    _send(hand_disp, "/right/quat/get", "1", *raw.tolist())

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
    hand_disp = m.hand_source._srv.dispatcher
    raw = np.zeros(72, dtype=np.float32); raw[3::4] = 1.0
    _send(hand_disp, "/right/quat/get", "1", *raw.tolist())

    _send(arm_disp, "/controller/left/pos", 1.0, 2.0, 3.0)
    _send(arm_disp, "/controller/left/rot", 0.0, 0.0, 0.0, 1.0)
    assert m.get_q()["right"]["arm1"] == pytest.approx(3.02)

    _send(arm_disp, "/controller/left/pos", 9.0, 8.0, 7.0)
    assert m.get_q()["right"]["arm1"] == pytest.approx(7.02)


def test_crosshand_output_on_glove_side():
    m = _make_model()
    arm_disp = m.arm_source._srv.dispatcher
    hand_disp = m.hand_source._srv.dispatcher

    _send(arm_disp, "/controller/left/pos", 1.0, 2.0, 3.0)
    _send(arm_disp, "/controller/left/rot", 0.0, 0.0, 0.0, 1.0)
    raw = np.zeros(72, dtype=np.float32); raw[3::4] = 1.0
    _send(hand_disp, "/right/quat/get", "1", *raw.tolist())

    q = m.get_q()
    assert q["right"]["arm1"] == pytest.approx(3.02)
    assert q["right"]["f1"] == pytest.approx(0.5)
    assert q["left"] == {}


def test_arm_omitted_when_controller_untracked_hand_still_retargets():
    m = _make_model()
    hand_disp = m.hand_source._srv.dispatcher

    raw = np.zeros(72, dtype=np.float32)
    raw[3::4] = 1.0
    _send(hand_disp, "/right/quat/get", "1", *raw.tolist())

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
    hand_disp = m.hand_source._srv.dispatcher
    _send(arm_disp, "/hmd/rot", *Rotation.from_euler("z", 0.3).as_quat().tolist())
    _send(arm_disp, "/controller/left/pos", 0.0, 0.0, 0.0)
    _send(arm_disp, "/controller/left/rot", *Rotation.from_euler("z", 0.1).as_quat().tolist())
    raw = np.zeros(72, dtype=np.float32); raw[3::4] = 1.0
    _send(hand_disp, "/right/quat/get", "1", *raw.tolist())

    out = m.calibrate_yaw()
    assert out["right"] is True and out["left"] is False
    assert m.sides["right"].calib.ready


def test_send_feedback_sends_osc_to_mock_glove_client():
    m = _make_model()

    hand_disp = m.hand_source._srv.dispatcher
    _send(hand_disp, "/device/status/get", "4", False, True)
    assert m.hand_source.connected("right")

    sent = []

    class _MockClient:
        def send_message(self, address, value):
            sent.append((address, value))

    m.hand_source._udp_client = _MockClient()

    m.send_feedback({"side": "right", "forces": [10, 20, 30, 40, 50]})

    assert len(sent) == 1
    address, packet = sent[0]
    assert address == "/right/hapt/set"
    assert packet[0] == "10"
    assert packet[1:] == [0, 10, 1, 20, 2, 30, 3, 40, 4, 50]


def test_send_feedback_noop_when_not_connected():
    m = _make_model()
    sent = []

    class _MockClient:
        def send_message(self, address, value):
            sent.append((address, value))

    m.hand_source._udp_client = _MockClient()
    m.send_feedback({"side": "right", "forces": [1, 2, 3, 4, 5]})
    assert sent == []


def test_send_feedback_ignores_empty_or_none():
    m = _make_model()
    m.send_feedback(None)
    m.send_feedback({"side": "right"})
    m.send_feedback({"side": "right", "forces": []})


SPINE_PAIRS = [("right_index_mcp_z", 0.1), ("right_index_mcp_y", -0.2),
               ("right_index_pip", 0.3), ("right_thumb_ip", 1.5)]


def _flat(pairs):
    out = []
    for name, val in pairs:
        out.extend([name, float(val)])
    return out


def _send_joint_angles(disp, side, pairs, msg_type="17"):
    _send(disp, f"/{side}/joint_angles/get", msg_type, *_flat(pairs))


def test_glove_robot_hand_parses_name_value_pairs():
    recv = GloveRobotHandReceiver(glove_port=4840)
    _send_joint_angles(recv._srv.dispatcher, "right", SPINE_PAIRS)

    sample = recv.get("right")
    assert sample.tracked is True
    assert sample.joint_q == {"index_mcp_z": pytest.approx(0.1),
                              "index_mcp_y": pytest.approx(-0.2),
                              "index_pip": pytest.approx(0.3),
                              "thumb_ip": pytest.approx(1.5)}


def test_glove_robot_hand_joint_map_renames_and_filters():
    recv = GloveRobotHandReceiver(
        joint_map={"index_mcp_z": "I_flex", "thumb_ip": "T_ip"}, glove_port=4841)
    _send_joint_angles(recv._srv.dispatcher, "right", SPINE_PAIRS)

    assert recv.get("right").joint_q == {"I_flex": pytest.approx(0.1),
                                         "T_ip": pytest.approx(1.5)}


def test_glove_robot_hand_omits_q_before_any_packet():
    recv = GloveRobotHandReceiver(glove_port=4842)
    sample = recv.get("left")
    assert sample.tracked is False
    assert sample.joint_q is None
    assert sample.hand is None


def test_glove_robot_hand_rejects_misaligned_pairs():
    recv = GloveRobotHandReceiver(glove_port=4843)
    disp = recv._srv.dispatcher
    _send(disp, "/right/joint_angles/get", "17", 0.1, "right_index_pip")
    assert recv.get("right").joint_q is None


def test_glove_robot_hand_sides_independent():
    recv = GloveRobotHandReceiver(glove_port=4844)
    disp = recv._srv.dispatcher
    _send_joint_angles(disp, "left", [("left_index_pip", 0.0)], msg_type="16")
    _send_joint_angles(disp, "right", [("right_index_pip", 1.0)])

    assert recv.get("left").joint_q == {"index_pip": pytest.approx(0.0)}
    assert recv.get("right").joint_q == {"index_pip": pytest.approx(1.0)}


def test_glove_robot_hand_on_update_callback_fires():
    calls = []
    recv = GloveRobotHandReceiver(glove_port=4845, on_update=calls.append)
    _send_joint_angles(recv._srv.dispatcher, "right", SPINE_PAIRS)
    assert calls == ["right"]


def test_glove_robot_hand_wrist_becomes_canonical_hand_pose():
    from whatslab.receiver.glove.human_hand import wrist_to_canonical
    from whatslab.receiver.glove.robot_hand import spine_lh_xyzw, unpack_wrist

    recv = GloveRobotHandReceiver(glove_port=4846)
    raw_wxyz = np.array([0.5, 0.5, 0.5, 0.5])
    w, x, y, z = raw_wxyz
    wire = [y, x, z, -w]
    _send(recv._srv.dispatcher, "/right/wrist/get", "19", *(float(v) for v in wire))

    sample = recv.get("right")
    assert sample.hand is not None
    assert sample.hand.tracked is False
    assert sample.joint_q is None
    expected = wrist_to_canonical(spine_lh_xyzw(unpack_wrist(wire)))
    assert sample.hand.wrist.quat == pytest.approx(expected)
    assert unpack_wrist(wire) == pytest.approx(raw_wxyz)


def test_glove_robot_hand_wrist_rejects_degenerate_quat():
    recv = GloveRobotHandReceiver(glove_port=4847)
    disp = recv._srv.dispatcher
    _send(disp, "/right/wrist/get", "19", 0.0, 0.0, 0.0, 0.0)
    _send(disp, "/right/wrist/get", "19", float("nan"), 0.0, 0.0, 1.0)
    assert recv.get("right").hand is None


def test_glove_robot_hand_model_bypass_end_to_end():
    class _RobotHandTestModel(TeleopModel):
        def __init__(self, robot):
            self._receiver = GloveRobotHandReceiver(glove_port=4848)
            self.hand_source = self._receiver
            super().__init__(robot)

        def _get_raw_target(self):
            return {s: None for s in self.SIDES}

    m = _RobotHandTestModel(_FakeRobot())
    _send_joint_angles(m._receiver._srv.dispatcher, "right", SPINE_PAIRS)

    q = m.get_q()["right"]
    assert q == {"index_mcp_z": pytest.approx(0.1), "index_mcp_y": pytest.approx(-0.2),
                 "index_pip": pytest.approx(0.3), "thumb_ip": pytest.approx(1.5)}
    assert m.robot.solve_calls == 0


def test_teleop_model_still_accepts_duck_typed_robot():
    class _Passthrough(TeleopModel):
        def _get_raw_target(self):
            return {s: None for s in self.SIDES}

    duck = _FakeRobot()
    m = _Passthrough(duck)
    assert m.sides["right"].robot is duck
    assert m.sides["left"].robot is duck


def test_every_exported_preset_is_instantiable():
    pytest.importorskip("pinocchio")
    pytest.importorskip("dex_retargeting")
    import whatslab.teleop as T

    from whatslab.receiver.glove.human_hand import GloveHumanHandReceiver
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
    for r in (GloveHumanHandReceiver, QuestControllerReceiver, QuestHandReceiver):
        assert r is not None
