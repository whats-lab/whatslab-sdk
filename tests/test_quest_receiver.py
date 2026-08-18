import numpy as np
import pytest
from pythonosc.osc_message_builder import OscMessageBuilder

from scipy.spatial.transform import Rotation

from whatslab.receiver.base import NUM_FINGER_JOINTS
from whatslab.receiver.quest.base import _CANONICAL_M as _M
from whatslab.receiver.quest.controller import CONTROLLER_POS_OFFSET, QuestControllerReceiver
from whatslab.receiver.quest.hand import QuestHandReceiver


def _to_c(pos, quat):
    p = _M @ np.asarray(pos, dtype=float)
    R = Rotation.from_quat(np.asarray(quat, dtype=float)).as_matrix()
    return p, Rotation.from_matrix(_M @ R @ _M.T).as_quat()


def _packet(address: str, *args) -> bytes:
    b = OscMessageBuilder(address=address)
    for a in args:
        b.add_arg(a)
    return b.build().dgram


def _send(disp, address, *args):
    disp.call_handlers_for_packet(_packet(address, *args), ("127.0.0.1", 0))


def test_controller_get_untracked_before_any_packet():
    recv = QuestControllerReceiver(quest_port=9996)
    sample = recv.get("left")
    assert sample.controller is None
    assert sample.hmd is None
    assert not sample.tracked


def test_controller_get_independent_per_side():
    recv = QuestControllerReceiver(quest_port=9997)
    disp = recv._srv.dispatcher

    _send(disp, "/controller/left/pos", 1.0, 2.0, 3.0)
    _send(disp, "/controller/left/rot", 0.0, 0.0, 0.0, 1.0)
    _send(disp, "/controller/right/pos", 4.0, 5.0, 6.0)
    _send(disp, "/controller/right/rot", 0.0, 1.0, 0.0, 0.0)
    _send(disp, "/hmd/rot", 0.0, 0.0, 1.0, 0.0)

    left = recv.get("left")
    right = recv.get("right")

    lp, lq = _to_c([1.0, 2.0, 3.0], [0.0, 0.0, 0.0, 1.0])
    rp, rq = _to_c([4.0, 5.0, 6.0], [0.0, 1.0, 0.0, 0.0])
    assert left.controller is not None
    assert np.allclose(left.controller.pos, lp + CONTROLLER_POS_OFFSET)
    assert np.allclose(left.controller.quat, lq)
    assert left.controller.pos[0] == pytest.approx(3.0 + CONTROLLER_POS_OFFSET[0])
    assert left.tracked

    assert right.controller is not None
    assert np.allclose(right.controller.pos, rp + CONTROLLER_POS_OFFSET)
    assert np.allclose(right.controller.quat, rq)
    assert right.tracked

    assert not np.allclose(left.controller.pos, right.controller.pos)

    _, hq = _to_c([0.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0])
    assert left.hmd is not None and right.hmd is not None
    assert np.allclose(left.hmd.quat, hq)
    assert np.allclose(right.hmd.quat, hq)


def test_two_quest_receivers_share_same_port_server():
    a = QuestControllerReceiver(quest_port=9998)
    b = QuestControllerReceiver(quest_port=9998)
    assert a._srv is b._srv


def test_hand_get_untracked_before_any_packet():
    recv = QuestHandReceiver(quest_port=9995)
    sample = recv.get("left")
    assert sample.hand is not None
    assert not sample.hand.tracked
    assert sample.hmd is None
    assert not sample.tracked


def test_hand_get_wrist_and_fingers_per_side():
    recv = QuestHandReceiver(quest_port=9994)
    disp = recv._srv.dispatcher

    _send(disp, "/hand/left/pos", 0.1, 0.2, 0.3)
    _send(disp, "/hand/left/rot", 0.0, 0.0, 0.0, 1.0)
    left_joint_rots = [0.0, 0.0, 0.0, 1.0] * NUM_FINGER_JOINTS
    _send(disp, "/hand/left/joints/pos", *([0.0] * (NUM_FINGER_JOINTS * 3)))
    _send(disp, "/hand/left/joints/rot", *left_joint_rots)

    _send(disp, "/hand/right/pos", 0.4, 0.5, 0.6)
    _send(disp, "/hand/right/rot", 0.0, 1.0, 0.0, 0.0)
    right_joint_rots = [0.0, 1.0, 0.0, 0.0] * NUM_FINGER_JOINTS
    _send(disp, "/hand/right/joints/pos", *([0.0] * (NUM_FINGER_JOINTS * 3)))
    _send(disp, "/hand/right/joints/rot", *right_joint_rots)

    _send(disp, "/hmd/rot", 0.0, 0.0, 1.0, 0.0)

    left = recv.get("left")
    right = recv.get("right")

    lwp, lwq = _to_c([0.1, 0.2, 0.3], [0.0, 0.0, 0.0, 1.0])
    rwp, rwq = _to_c([0.4, 0.5, 0.6], [0.0, 1.0, 0.0, 0.0])
    assert left.hand is not None and left.hand.tracked
    assert left.hand.wrist is not None
    assert np.allclose(left.hand.wrist.pos, lwp)
    assert np.allclose(left.hand.wrist.quat, lwq)
    assert len(left.hand.joint_rot) == NUM_FINGER_JOINTS
    for q in left.hand.joint_rot.values():
        assert np.allclose(q, [0.0, 0.0, 0.0, 1.0])

    assert right.hand is not None and right.hand.tracked
    assert np.allclose(right.hand.wrist.pos, rwp)
    assert np.allclose(right.hand.wrist.quat, rwq)
    for q in right.hand.joint_rot.values():
        assert np.allclose(q, [0.0, 1.0, 0.0, 0.0])

    assert not np.allclose(left.hand.wrist.pos, right.hand.wrist.pos)

    _, hq = _to_c([0.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0])
    assert left.hmd is not None and right.hmd is not None
    assert np.allclose(left.hmd.quat, hq)


def test_hand_on_update_callback_fires_on_new_joints_frame():
    calls = []
    recv = QuestHandReceiver(quest_port=9993, on_update=lambda side: calls.append(side))
    disp = recv._srv.dispatcher

    _send(disp, "/hand/left/joints/rot", *([0.0, 0.0, 0.0, 1.0] * NUM_FINGER_JOINTS))
    _send(disp, "/hand/right/joints/rot", *([0.0, 0.0, 0.0, 1.0] * NUM_FINGER_JOINTS))

    assert calls == ["left", "right"]


def test_controller_and_hand_receivers_share_same_port_server():
    a = QuestControllerReceiver(quest_port=9992)
    b = QuestHandReceiver(quest_port=9992)
    assert a._srv is b._srv
