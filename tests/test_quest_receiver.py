"""QuestReceiverBase + QuestControllerReceiver — 공유 SharedOscServer 위에서 동작 검증.

실제 UDP 소켓 없이 dispatcher.call_handlers_for_packet 으로 OSC 패킷을 주입한다
(test_osc_transport.py 와 동일한 접근).
"""
import numpy as np
import pytest
from pythonosc.osc_message_builder import OscMessageBuilder

from scipy.spatial.transform import Rotation

from whatslab.receiver.base import NUM_FINGER_JOINTS
from whatslab.receiver.quest_base import _CANONICAL_M as _M
from whatslab.receiver.quest_controller import CONTROLLER_POS_OFFSET, QuestControllerReceiver
from whatslab.receiver.quest_hand import QuestHandReceiver


def _to_c(pos, quat):
    """테스트용 독립 정준 변환(리시버가 적용해야 하는 것): 위치 M·p, 회전 M·R·Mᵀ."""
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

    # 정준 변환(_to_c) 적용 후 + 마운트 오프셋. (raw pass-through 아님을 검증)
    lp, lq = _to_c([1.0, 2.0, 3.0], [0.0, 0.0, 0.0, 1.0])
    rp, rq = _to_c([4.0, 5.0, 6.0], [0.0, 1.0, 0.0, 0.0])
    assert left.controller is not None
    assert np.allclose(left.controller.pos, lp + CONTROLLER_POS_OFFSET)
    assert np.allclose(left.controller.quat, lq)
    assert left.controller.pos[0] == pytest.approx(3.0 + CONTROLLER_POS_OFFSET[0])  # in_z→robot_x
    assert left.tracked

    assert right.controller is not None
    assert np.allclose(right.controller.pos, rp + CONTROLLER_POS_OFFSET)
    assert np.allclose(right.controller.quat, rq)
    assert right.tracked

    # 좌/우 독립 — 서로의 controller pose 가 섞이지 않는다.
    assert not np.allclose(left.controller.pos, right.controller.pos)

    # hmd 도 정준 변환 적용(회전만). side 무관 공유.
    _, hq = _to_c([0.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0])
    assert left.hmd is not None and right.hmd is not None
    assert np.allclose(left.hmd.quat, hq)
    assert np.allclose(right.hmd.quat, hq)


def test_two_quest_receivers_share_same_port_server():
    """QuestReceiverBase 는 SharedOscServer.get(port) 로 서버를 공유한다."""
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

    # 손목(pos + root 회전)만 정준 변환. 손가락 관절 회전은 raw(리타게팅용) 유지.
    lwp, lwq = _to_c([0.1, 0.2, 0.3], [0.0, 0.0, 0.0, 1.0])
    rwp, rwq = _to_c([0.4, 0.5, 0.6], [0.0, 1.0, 0.0, 0.0])
    assert left.hand is not None and left.hand.tracked
    assert left.hand.wrist is not None
    assert np.allclose(left.hand.wrist.pos, lwp)
    assert np.allclose(left.hand.wrist.quat, lwq)
    assert len(left.hand.joint_rot) == NUM_FINGER_JOINTS
    for q in left.hand.joint_rot.values():
        assert np.allclose(q, [0.0, 0.0, 0.0, 1.0])       # 관절: 변환 안 함(raw)

    assert right.hand is not None and right.hand.tracked
    assert np.allclose(right.hand.wrist.pos, rwp)
    assert np.allclose(right.hand.wrist.quat, rwq)
    for q in right.hand.joint_rot.values():
        assert np.allclose(q, [0.0, 1.0, 0.0, 0.0])       # 관절: 변환 안 함(raw)

    # 좌/우 독립 — 섞이지 않음.
    assert not np.allclose(left.hand.wrist.pos, right.hand.wrist.pos)

    # hmd 는 side 무관 공유 + 정준 변환 적용.
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
    """모달리티(컨트롤러/손)가 분리돼도 동일 quest_port 면 서버 인스턴스 1개(T1 refcount 계약)."""
    a = QuestControllerReceiver(quest_port=9992)
    b = QuestHandReceiver(quest_port=9992)
    assert a._srv is b._srv
