"""GloveModel(손=글러브+리타게팅, 팔=Quest 컨트롤러+IK, 햅틱) + GloveRobotHandReceiver.

RobotModel/HandController/RobotArmIK 는 모의(mock)로 주입해 pinocchio 없이 이 로직만
검증한다(수식/IK 자체는 test_robot.py 소관). GloveModel 은 실제
GloveHumanHandReceiver/QuestControllerReceiver 를 구성하되, OSC 패킷 주입은
`dispatcher.call_handlers_for_packet` 으로 실 소켓 없이 수행한다
(test_glove_receiver.py/test_quest_receiver.py 와 동일 접근).
"""
from __future__ import annotations

from typing import List

import numpy as np
import pytest
from pythonosc.osc_message_builder import OscMessageBuilder

from whatslab.core.types import HandCommand, InputSample
from whatslab.model.base import TeleopModel
from whatslab.model.glove import GloveModel
from whatslab.receiver.glove_robot_hand import GloveRobotHandReceiver


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
    """RobotArmIK 가 위임하는 로봇 솔버 대역 — 목표 xy 를 그대로 되돌리는 더미."""

    def __init__(self):
        self.calls = 0

    def solve(self, T):
        self.calls += 1
        return np.array([T[0, 3], T[1, 3]])


class _FakeRig:
    class solver:
        reach_max = None

    class hand:                       # rig.hand.retarget → 손 리타게팅 config 유도
        retarget = "fake_hand"

    class calibration:                # rig.calibration.input_reach → reach 스케일
        input_reach = None


class _FakeRobot:
    """RobotModel 대역 — RobotArmIK 가 기대하는 to_base/rig/solver 계약을 모의."""

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
    # GloveModel: 글러브 side + 반대 side 컨트롤러(크로스핸드). 출력 로봇 side = 글러브 side.
    return GloveModel(_FakeRobot())


# --------------------------------------------------------------------- tests
def test_no_input_returns_empty_q():
    """입력 없음 → 팔 목표/손가락 모두 없어 각 side q 가 빈 dict(관절 생략), IK 미실행."""
    m = _make_model()
    q = m.get_q()
    assert q["right"] == {} and q["left"] == {}
    assert m.robot.solve_calls == 0


def test_start_stop_delegate_to_both_receivers():
    from whatslab.receiver.glove_human_hand import GloveHumanHandReceiver
    from whatslab.receiver.quest_controller import QuestControllerReceiver
    m = _make_model()
    # 실 소켓 bind/close 왕복 검증 — 유니크 포트로 교체해 다른 테스트의 기본 포트
    # (4040/9000)와 절대 충돌하지 않게(결정적).
    m.hand_source = GloveHumanHandReceiver(glove_port=4759)
    m.arm_source = QuestControllerReceiver(quest_port=9759)
    assert set(m._receivers) == {m.hand_source, m.arm_source}
    m.start()
    m.stop()


def test_hand_and_arm_combine_into_single_q():
    """글러브(손가락, 오른손) + 컨트롤러(팔, 왼손) 최신 프레임을 합쳐 오른손 로봇 q 를 낸다."""
    m = _make_model()   # 컨트롤러는 반대(left)

    hand_disp = m.hand_source._srv.dispatcher
    arm_disp = m.arm_source._srv.dispatcher

    # 팔 목표: 컨트롤러(반대 side=left) pos/rot 주입.
    _send(arm_disp, "/controller/left/pos", 1.0, 2.0, 3.0)
    _send(arm_disp, "/controller/left/rot", 0.0, 0.0, 0.0, 1.0)

    # 손가락 회전: 글러브(right) 프레임 주입.
    raw = np.zeros(72, dtype=np.float32)
    raw[3::4] = 1.0  # 항등 쿼터니언(w=1) 채우기
    _send(hand_disp, "/right/quat/get", "1", *raw.tolist())

    assert m.robot.solve_calls == 0   # 폴링 전엔 IK 미실행

    q = m.get_q()["right"]
    assert m.robot.solve_calls == 1
    # QuestControllerReceiver 가 불변 CONTROLLER_POS_OFFSET([0.02, -0.04, 0.08])을
    # 가산하므로 _FakeRobot 의 solver.solve(T) 가 되돌리는 값도 그만큼 오프셋된다.
    assert q["arm1"] == pytest.approx(3.02)
    assert q["arm2"] == pytest.approx(-1.04)
    assert q["f1"] == pytest.approx(0.5)
    assert q["f2"] == pytest.approx(0.6)


def test_new_controller_pose_reflected_in_q():
    """컨트롤러 위치가 바뀌면 다음 get_q 가 새 목표를 반영한다(매 호출 계산)."""
    m = _make_model()
    arm_disp = m.arm_source._srv.dispatcher
    hand_disp = m.hand_source._srv.dispatcher
    raw = np.zeros(72, dtype=np.float32); raw[3::4] = 1.0
    _send(hand_disp, "/right/quat/get", "1", *raw.tolist())   # 손목 quat 유효화

    _send(arm_disp, "/controller/left/pos", 1.0, 2.0, 3.0)
    _send(arm_disp, "/controller/left/rot", 0.0, 0.0, 0.0, 1.0)
    assert m.get_q()["right"]["arm1"] == pytest.approx(3.02)   # 정준 in_z=3 +off

    _send(arm_disp, "/controller/left/pos", 9.0, 8.0, 7.0)
    assert m.get_q()["right"]["arm1"] == pytest.approx(7.02)   # 새 목표(in_z=7) 반영


def test_crosshand_output_on_glove_side():
    """크로스핸드: 글러브(right) + 컨트롤러(left) → 출력 로봇 side = 글러브 side(right).
    반대 side(left)는 구동 안 함(빈 q)."""
    m = _make_model()
    arm_disp = m.arm_source._srv.dispatcher
    hand_disp = m.hand_source._srv.dispatcher

    _send(arm_disp, "/controller/left/pos", 1.0, 2.0, 3.0)     # 팔 = 왼손 컨트롤러
    _send(arm_disp, "/controller/left/rot", 0.0, 0.0, 0.0, 1.0)
    raw = np.zeros(72, dtype=np.float32); raw[3::4] = 1.0
    _send(hand_disp, "/right/quat/get", "1", *raw.tolist())    # 손가락 = 오른손 글러브

    q = m.get_q()
    assert q["right"]["arm1"] == pytest.approx(3.02)   # 왼손 컨트롤러 → 오른손 로봇 팔
    assert q["right"]["f1"] == pytest.approx(0.5)       # 오른손 글러브 → 손가락
    assert q["left"] == {}                              # 반대 side 미구동


def test_arm_omitted_when_controller_untracked_hand_still_retargets():
    """컨트롤러 미수신(팔 미추적) → 팔 관절 생략(B: q 에서 제외). 손가락은 글러브만으로 동작."""
    m = _make_model()
    hand_disp = m.hand_source._srv.dispatcher

    raw = np.zeros(72, dtype=np.float32)
    raw[3::4] = 1.0
    _send(hand_disp, "/right/quat/get", "1", *raw.tolist())

    q = m.get_q()["right"]
    assert "arm1" not in q and "arm2" not in q     # 컨트롤러 없음 → 팔 관절 생략
    assert q["f1"] == pytest.approx(0.5)            # 손가락은 글러브만으로 정상
    assert q["f2"] == pytest.approx(0.6)
    assert m.robot.solve_calls == 0                # 팔 목표 없으니 IK solve 미호출


def test_joint_q_bypass_skips_ik_and_retarget():
    """손 소스가 joint_q 를 주면 (GloveRobotHand 모드) IK/리타게팅을 건너뛴다."""
    m = _make_model()
    direct = {"custom_joint": 1.23}

    class _JointQSource:
        def get(self, side):
            return InputSample(joint_q=direct, tracked=True, timestamp=1.0)

    m.hand_source = _JointQSource()   # 손 소스를 직접-q 소스로 교체(pull)

    q = m.get_q()["right"]
    assert q == direct
    assert m.robot.solve_calls == 0


def test_calibrate_yaw_captures_arm_target_side():
    from scipy.spatial.transform import Rotation

    m = _make_model()
    assert m.calibrate_yaw()["right"] is False   # 입력 없음

    # 팔 목표를 만들려면 컨트롤러(left) + 글러브(right 손목) 둘 다 필요.
    arm_disp = m.arm_source._srv.dispatcher
    hand_disp = m.hand_source._srv.dispatcher
    _send(arm_disp, "/hmd/rot", *Rotation.from_euler("z", 0.3).as_quat().tolist())
    _send(arm_disp, "/controller/left/pos", 0.0, 0.0, 0.0)
    _send(arm_disp, "/controller/left/rot", *Rotation.from_euler("z", 0.1).as_quat().tolist())
    raw = np.zeros(72, dtype=np.float32); raw[3::4] = 1.0
    _send(hand_disp, "/right/quat/get", "1", *raw.tolist())

    out = m.calibrate_yaw()
    assert out["right"] is True and out["left"] is False   # 글러브 side 만 팔 목표 존재
    assert m.calib["right"].ready


def test_send_feedback_sends_osc_to_mock_glove_client():
    """send_feedback(data) → 글러브 햅틱 OSC 송신(모의 클라이언트 캡처)."""
    m = _make_model()

    # 하트비트 없이(연결 상태 주입) 송신 가능하도록 connected=True 로 설정.
    hand_disp = m.hand_source._srv.dispatcher
    _send(hand_disp, "/device/status/get", "4", False, True)  # left=False, right=True
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
    assert packet[0] == "10"                     # OSC_MSG_TYPE_RIGHT_HAPT
    assert packet[1:] == [0, 10, 1, 20, 2, 30, 3, 40, 4, 50]


def test_send_feedback_noop_when_not_connected():
    m = _make_model()
    sent = []

    class _MockClient:
        def send_message(self, address, value):
            sent.append((address, value))

    m.hand_source._udp_client = _MockClient()
    m.send_feedback({"side": "right", "forces": [1, 2, 3, 4, 5]})
    assert sent == []   # connected=False → send_haptic 이 조용히 실패(False 반환)


def test_send_feedback_ignores_empty_or_none():
    m = _make_model()
    m.send_feedback(None)
    m.send_feedback({"side": "right"})
    m.send_feedback({"side": "right", "forces": []})   # 예외 없이 무시


# ------------------------------------------------------- GloveRobotHandReceiver
JOINT_NAMES: List[str] = ["j0", "j1", "j2", "j3", "j4", "j5"]


def test_glove_robot_hand_parses_q_into_joint_q():
    recv = GloveRobotHandReceiver(joint_names=JOINT_NAMES, glove_port=4840)
    disp = recv._srv.dispatcher

    q_values = [0.1, -0.2, 0.3, 0.0, 1.0, -1.5]
    _send(disp, "/glove/right/q", *q_values)

    sample = recv.get("right")
    assert sample.joint_q is not None
    assert sample.tracked is True
    for name, expected in zip(JOINT_NAMES, q_values):
        assert sample.joint_q[name] == pytest.approx(expected)


def test_glove_robot_hand_neutral_before_any_packet():
    recv = GloveRobotHandReceiver(joint_names=JOINT_NAMES, glove_port=4841)
    sample = recv.get("left")
    assert sample.tracked is False
    assert sample.joint_q == {name: 0.0 for name in JOINT_NAMES}


def test_glove_robot_hand_sides_independent():
    recv = GloveRobotHandReceiver(joint_names=JOINT_NAMES, glove_port=4842)
    disp = recv._srv.dispatcher

    _send(disp, "/glove/left/q", *([0.0] * len(JOINT_NAMES)))
    _send(disp, "/glove/right/q", *([1.0] * len(JOINT_NAMES)))

    left = recv.get("left")
    right = recv.get("right")
    assert all(v == pytest.approx(0.0) for v in left.joint_q.values())
    assert all(v == pytest.approx(1.0) for v in right.joint_q.values())


def test_glove_robot_hand_on_update_callback_fires():
    calls = []
    recv = GloveRobotHandReceiver(joint_names=JOINT_NAMES, glove_port=4843,
                                   on_update=lambda side: calls.append(side))
    disp = recv._srv.dispatcher
    _send(disp, "/glove/right/q", *([0.5] * len(JOINT_NAMES)))
    assert calls == ["right"]


def test_glove_robot_hand_requires_joint_names():
    with pytest.raises(ValueError):
        GloveRobotHandReceiver(joint_names=[], glove_port=4844)


def test_glove_robot_hand_model_bypass_end_to_end():
    """GloveRobotHandReceiver 를 Model 에 직접 꽂아 joint_q 를 채우면
    get_q 가 IK/리타게팅 없이 그대로 반환한다(실 콜백 경로, mock robot 사용)."""

    class _RobotHandTestModel(TeleopModel):
        def __init__(self, robot):
            self._receiver = GloveRobotHandReceiver(
                joint_names=JOINT_NAMES, glove_port=4845)
            self.hand_source = self._receiver   # 직접-q 는 손 소스
            super().__init__(robot)

        def _get_raw_target(self):
            return {s: None for s in self.SIDES}    # 팔 없음 — joint_q 우회만

    m = _RobotHandTestModel(_FakeRobot())
    disp = m._receiver._srv.dispatcher
    q_values = [0.42] * len(JOINT_NAMES)
    _send(disp, "/glove/right/q", *q_values)

    q = m.get_q()["right"]
    assert q == {name: pytest.approx(0.42) for name in JOINT_NAMES}
    assert m.robot.solve_calls == 0
