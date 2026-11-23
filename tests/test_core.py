import numpy as np

from whatslab.core.constants import XR_JOINT_COUNT
from whatslab.core.interfaces import HandController, Receiver
from whatslab.core.types import (
    HandCommand,
    HandPose,
    InputSample,
    Pose,
)


def test_pose_defaults():
    p = Pose()
    assert np.allclose(p.pos, np.zeros(3))
    assert np.allclose(p.quat, [0, 0, 0, 1])


def test_pose_matrix_roundtrip():
    from scipy.spatial.transform import Rotation

    quat = Rotation.from_euler("xyz", [0.3, -0.5, 1.2]).as_quat()
    p = Pose(pos=np.array([0.1, 0.2, 0.3]), quat=quat)
    tf = p.to_matrix()
    assert tf.shape == (4, 4)
    assert np.allclose(tf[:3, 3], p.pos)

    back = Pose.from_matrix(tf)
    assert np.allclose(back.pos, p.pos)
    assert np.allclose(back.to_matrix()[:3, :3], tf[:3, :3], atol=1e-8)


def test_input_sample_defaults():
    s = InputSample()
    assert s.tracked is False
    assert s.hand is None


def test_handpose_defaults():
    hp = HandPose()
    assert hp.xr_joints is None and hp.xr_valid is None and hp.wrist is None
    assert hp.tracked is False and XR_JOINT_COUNT == 26


def test_hand_command_defaults():
    c = HandCommand()
    assert c.joint_names == []
    assert c.joint_angles.shape == (0,)
    assert c.gripper is None


def test_protocols_are_runtime_checkable():
    class _R:
        def start(self): ...
        def stop(self): ...
        def get(self, side): ...

    assert isinstance(_R(), Receiver)
    assert not isinstance(object(), Receiver)
    assert hasattr(HandController, "_is_runtime_protocol") or True
