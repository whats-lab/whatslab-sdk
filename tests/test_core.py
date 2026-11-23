from whatslab.core.constants import XR_JOINT_COUNT
from whatslab.core.interfaces import HandController, Receiver
from whatslab.core.types import HandCommand, HandPose, InputSample


def test_input_sample_defaults():
    s = InputSample()
    assert s.tracked is False
    assert s.hand is None


def test_handpose_defaults():
    hp = HandPose()
    assert hp.xr_joints is None and hp.xr_valid is None
    assert hp.tracked is False and XR_JOINT_COUNT == 26


def test_hand_command_defaults():
    c = HandCommand()
    assert c.joint_names == []
    assert c.joint_angles.shape == (0,)


def test_protocols_are_runtime_checkable():
    class _R:
        def start(self): ...
        def stop(self): ...
        def get(self, side): ...

    class _H:
        joint_names = ["a"]
        def compute(self, sample): ...

    assert isinstance(_R(), Receiver)
    assert not isinstance(object(), Receiver)
    assert isinstance(_H(), HandController)
