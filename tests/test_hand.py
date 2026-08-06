import numpy as np
import pytest

from whatslab.teleop.hand.hand_configs import CONFIG_REGISTRY


EXPECTED_HANDS = {
    "base_hand", "orca_hand", "robotis_hx5_d20",
    "allegro_hand", "schunk_hand", "tesollo_dg5f", "ability_hand",
}


def test_registry_has_expected_hands():
    assert EXPECTED_HANDS.issubset(set(CONFIG_REGISTRY))


@pytest.mark.parametrize("name", sorted(EXPECTED_HANDS))
def test_configs_construct_and_two_stage(name):
    cfg = CONFIG_REGISTRY[name]()
    for side in ("left", "right"):
        s1, s2 = cfg.get_two_stage_config(side)
        assert s1["target_origin_link_names"], f"{name}/{side} stage1 빈 체인"
        assert s2["target_link_names"], f"{name}/{side} stage2 빈 팁"


def test_config_human_are_skeleton_names():
    from whatslab.core.types import JOINT_INDEX

    for name, C in CONFIG_REGISTRY.items():
        for chains in C._FINGERS.values():
            for f in chains:
                for h in f.human:
                    assert isinstance(h, str), f"{name}: 정수 human 잔존 {h} in {f.links}"
                    assert h in JOINT_INDEX, f"{name}: 알 수 없는 관절명 {h!r}"


def test_finger_chain_consistency():
    for name in EXPECTED_HANDS:
        cfg = CONFIG_REGISTRY[name]()
        for side in ("left", "right"):
            fingers = cfg._get_fingers(side)
            for f in fingers:
                assert len(f.links) == len(f.human), f"{name}/{side} 길이 불일치: {f.links}"
                assert len(f.links) >= 2, f"{name}/{side} 체인 너무 짧음: {f.links}"
                assert f.human[0] == 0, f"{name}/{side} 손가락은 wrist(0)에서 시작해야: {f.human}"


def test_hand_retarget_end_to_end():
    pytest.importorskip("dex_retargeting")
    pytest.importorskip("pinocchio")
    from whatslab.teleop.hand import HandRetargeter

    r = HandRetargeter("right", "allegro_hand")
    assert len(r.joint_names) == 16
    assert r.tip_human_indices
    q = np.tile([0, 0, 0, 1.0], (17, 1))
    qpos = r.compute(q)
    assert qpos.shape == (16,)
    assert np.all(np.isfinite(qpos))
    assert np.allclose(r._wrist_offset, [0, 0, -0.065], atol=1e-6)
    assert np.allclose(r.last_human_positions[0], 0.0)


def test_hand_controller_from_input_sample():
    pytest.importorskip("dex_retargeting")
    pytest.importorskip("pinocchio")
    from whatslab.core.types import HandPose, InputSample
    from whatslab.teleop.hand import HandRetargetController

    ctrl = HandRetargetController("right", "allegro_hand")
    hand = HandPose.from_sensor_array(np.tile([0, 0, 0, 1.0], (17, 1)), tracked=True)
    cmd = ctrl.compute(InputSample(hand=hand, tracked=True))
    assert cmd.joint_names == ctrl.joint_names
    assert cmd.joint_angles.shape == (16,)
    cmd2 = ctrl.compute(InputSample(hand=None, tracked=False))
    assert np.allclose(cmd2.joint_angles, cmd.joint_angles)
