import numpy as np
import pytest

from whatslab.solvers.hand.hand_configs import CONFIG_REGISTRY


EXPECTED_HANDS = {
    "base_hand", "orca_hand", "robotis_hx5_d20",
    "allegro_hand", "schunk_hand", "tesollo_dg5f", "ability_hand",
}


def test_registry_has_expected_hands():
    assert EXPECTED_HANDS.issubset(set(CONFIG_REGISTRY))


def _derivable(name, side):
    pytest.importorskip("pinocchio")
    try:
        CONFIG_REGISTRY[name]()._get_fingers(side)
    except (FileNotFoundError, ValueError) as e:
        pytest.skip(f"{name}/{side}: 센서 프레임 URDF 없음 ({e})")


@pytest.mark.parametrize("name", sorted(EXPECTED_HANDS))
def test_configs_construct_and_two_stage(name):
    for side in ("left", "right"):
        _derivable(name, side)
        cfg = CONFIG_REGISTRY[name]()
        s1, s2 = cfg.get_two_stage_config(side)
        assert s1["target_origin_link_names"], f"{name}/{side} stage1 빈 체인"
        assert s2["target_link_names"], f"{name}/{side} stage2 빈 팁"
        assert all("_sensor_" in n for n in s2["target_link_names"]), \
            f"{name}/{side} 팁이 센서 프레임이 아니다: {s2['target_link_names']}"


def test_config_human_are_skeleton_names():
    from whatslab.core.types import JOINT_INDEX

    for name, C in CONFIG_REGISTRY.items():
        assert C._HUMAN_CHAIN, f"{name}: _HUMAN_CHAIN 이 비어 있다"
        for finger, chain in C._HUMAN_CHAIN.items():
            assert chain[0] == "wrist", f"{name}/{finger}: wrist 에서 시작해야: {chain}"
            for h in chain:
                assert h in JOINT_INDEX, f"{name}/{finger}: 알 수 없는 관절명 {h!r}"


def test_finger_chain_consistency():
    for name in sorted(EXPECTED_HANDS):
        for side in ("left", "right"):
            _derivable(name, side)
            fingers = CONFIG_REGISTRY[name]()._get_fingers(side)
            for f in fingers:
                assert len(f.links) == len(f.human), f"{name}/{side} 길이 불일치: {f.links}"
                assert len(f.links) >= 2, f"{name}/{side} 체인 너무 짧음: {f.links}"
                assert f.human[0] == 0, f"{name}/{side} 손가락은 wrist(0)에서 시작해야: {f.human}"


def test_palm_link_is_derived_not_configured():
    for name in sorted(EXPECTED_HANDS):
        _derivable(name, "right")
        cfg = CONFIG_REGISTRY[name]()
        palm = cfg.get_wrist_link_name("right")
        assert palm and "_sensor_" not in palm, f"{name}: 팜 링크 유도 실패 {palm!r}"
        assert all(f.links[0] == palm for f in cfg._get_fingers("right"))

def test_uni_retargeter_end_to_end():
    pytest.importorskip("onnxruntime")
    from whatslab.solvers.hand import UniRetargeter

    for robot in ("orca", "allegro", "tesollo", "robotis", "human"):
        for side in ("left", "right"):
            r = UniRetargeter(side, robot)
            assert r.joint_names, f"{robot}/{side}"
            neutral = {n: 0.0 for n in r.human_joint_names}
            q = r.compute(neutral)
            assert q.shape == (len(r.joint_names),)
            assert np.all(np.isfinite(q))
            lo, hi = r._feed["lo"], r._feed["hi"]
            slack = 0.02 * (hi - lo)
            assert np.all(q >= lo - slack) and np.all(q <= hi + slack)


def test_uni_retargeter_responds_to_input():
    pytest.importorskip("onnxruntime")
    from whatslab.solvers.hand import UniRetargeter

    r = UniRetargeter("left", "orca")
    q0 = r.compute({n: 0.0 for n in r.human_joint_names})
    q1 = r.compute({n: 0.4 for n in r.human_joint_names})
    assert np.abs(q1 - q0).max() > 0.05, "입력에 무반응 — 그래프가 상수화됐다"


def test_uni_retargeter_config_alias():
    pytest.importorskip("onnxruntime")
    from whatslab.solvers.hand import UniRetargeter

    a = UniRetargeter("left", "orca_hand")
    b = UniRetargeter("left", "orca")
    assert a.joint_names == b.joint_names
    with pytest.raises(ValueError, match="표에 없는"):
        UniRetargeter("left", "nope_hand")


def test_uni_retargeter_accepts_unprefixed_names():
    pytest.importorskip("onnxruntime")
    from whatslab.solvers.hand import UniRetargeter

    r = UniRetargeter("left", "tesollo")
    full = {n: 0.3 for n in r.human_joint_names}
    bare = {n.split("_", 1)[1]: 0.3 for n in r.human_joint_names}
    assert np.allclose(r.compute(full), r.compute(bare))


def test_hand_controller_from_input_sample():
    pytest.importorskip("onnxruntime")
    from whatslab.core.types import HandPose, InputSample
    from whatslab.solvers.hand import HandRetargetController

    ctrl = HandRetargetController("right", "orca_hand")
    angles = {n: 0.0 for n in ctrl.engine.human_joint_names}
    hand = HandPose(joint_angles=angles, tracked=True)
    cmd = ctrl.compute(InputSample(hand=hand, tracked=True))
    assert cmd.joint_names == ctrl.joint_names
    assert cmd.joint_angles.shape == (len(ctrl.joint_names),)
    cmd2 = ctrl.compute(InputSample(hand=None, tracked=False))
    assert np.allclose(cmd2.joint_angles, cmd.joint_angles)
