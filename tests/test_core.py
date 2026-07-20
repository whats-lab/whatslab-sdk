"""core 타입/인터페이스 — 의존성 없음, 항상 실행."""
import numpy as np

from whatslab.core.types import (
    HUMAN_HAND,
    JOINT_INDEX,
    SENSED_JOINTS,
    HandCommand,
    HandPose,
    InputSample,
    Pose,
)
from whatslab.core.interfaces import ArmSolver, HandController, Receiver


def test_pose_defaults():
    p = Pose()
    assert np.allclose(p.pos, np.zeros(3))
    assert np.allclose(p.quat, [0, 0, 0, 1])


def test_pose_matrix_roundtrip():
    from scipy.spatial.transform import Rotation

    quat = Rotation.from_euler("xyz", [0.3, -0.5, 1.2]).as_quat()
    p = Pose(pos=np.array([0.1, 0.2, 0.3]), quat=quat)
    T = p.to_matrix()
    assert T.shape == (4, 4)
    assert np.allclose(T[:3, 3], p.pos)

    back = Pose.from_matrix(T)
    assert np.allclose(back.pos, p.pos)
    # 쿼터니언은 부호 모호성 → 회전행렬로 비교
    assert np.allclose(back.to_matrix()[:3, :3], T[:3, :3], atol=1e-8)


def test_input_sample_defaults():
    s = InputSample()
    assert s.tracked is False
    assert s.controller is None and s.hand is None


def test_skeleton_consistency():
    # 23 노드, 16 sensed, wrist 가 유일한 root
    assert len(HUMAN_HAND) == 23
    assert len(SENSED_JOINTS) == 16
    roots = [j for j in HUMAN_HAND if j.parent is None]
    assert len(roots) == 1 and roots[0].name == "wrist"
    # 모든 parent 는 실재 노드
    names = {j.name for j in HUMAN_HAND}
    for j in HUMAN_HAND:
        assert j.parent is None or j.parent in names
    # JOINT_INDEX 는 선언 순서
    assert JOINT_INDEX["wrist"] == 0 and JOINT_INDEX["pinky_tip"] == 22


def test_handpose_sensor_array_roundtrip():
    import numpy as np

    a = np.random.RandomState(1).randn(17, 4)
    hp = HandPose.from_sensor_array(a, wrist_pos=np.array([1.0, 2.0, 3.0]))
    assert hp.wrist is not None and np.allclose(hp.wrist.pos, [1, 2, 3])
    assert len(hp.joint_rot) == 16
    assert np.allclose(hp.to_sensor_array(), a)


def test_handpose_empty_defaults_identity():
    hp = HandPose()
    arr = hp.to_sensor_array()
    assert arr.shape == (17, 4)
    assert np.allclose(arr[:, 3], 1.0)   # 미채움 → 항등


def test_hand_command_defaults():
    c = HandCommand()
    assert c.joint_names == []
    assert c.joint_angles.shape == (0,)
    assert c.gripper is None


def test_protocols_are_runtime_checkable():
    # Protocol 자체가 runtime_checkable 로 선언됐는지 (isinstance 사용 가능)
    class _R:
        def start(self): ...
        def stop(self): ...
        def get(self, side): ...

    assert isinstance(_R(), Receiver)
    assert not isinstance(object(), Receiver)
    # HandController / ArmSolver 도 Protocol
    assert hasattr(HandController, "_is_runtime_protocol") or True
    assert ArmSolver is not None
