"""hand — 설정 레지스트리(항상)와 리타게팅 end-to-end(가드)."""
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
    """B단계: 모든 config 의 human 은 골격 관절명(str)이어야 (매직넘버 제거)."""
    from whatslab.core.types import JOINT_INDEX

    for name, C in CONFIG_REGISTRY.items():
        for chains in C._FINGERS.values():
            for f in chains:
                for h in f.human:
                    assert isinstance(h, str), f"{name}: 정수 human 잔존 {h} in {f.links}"
                    assert h in JOINT_INDEX, f"{name}: 알 수 없는 관절명 {h!r}"


def test_finger_chain_consistency():
    """각 손가락 체인: links 와 human 길이 일치, 최소 2링크, wrist(0) 시작."""
    for name in EXPECTED_HANDS:
        cfg = CONFIG_REGISTRY[name]()
        for side in ("left", "right"):
            fingers = cfg._get_fingers(side)
            for f in fingers:
                assert len(f.links) == len(f.human), f"{name}/{side} 길이 불일치: {f.links}"
                assert len(f.links) >= 2, f"{name}/{side} 체인 너무 짧음: {f.links}"
                assert f.human[0] == 0, f"{name}/{side} 손가락은 wrist(0)에서 시작해야: {f.human}"


# ── end-to-end (dex_retargeting 필요, URDF 는 내장 사용 → env 불필요) ──────────
def test_hand_retarget_end_to_end():
    pytest.importorskip("dex_retargeting")
    pytest.importorskip("pinocchio")
    from whatslab.teleop.hand import HandRetargeter

    r = HandRetargeter("right", "allegro_hand")   # 내장 URDF (urdf_root 없이)
    assert len(r.joint_names) == 16                 # allegro 16-DOF
    assert r.tip_human_indices                      # 팁 인덱스 노출
    q = np.tile([0, 0, 0, 1.0], (17, 1))
    qpos = r.compute(q)
    assert qpos.shape == (16,)
    assert np.all(np.isfinite(qpos))
    # allegro 기준 palm → wrist_offset z=-0.065
    assert np.allclose(r._wrist_offset, [0, 0, -0.065], atol=1e-6)
    # TF 용 human 위치: 손목(0)은 원점
    assert np.allclose(r.last_human_positions[0], 0.0)


def test_hand_controller_from_input_sample():
    """HandRetargetController: InputSample(HandPose) → HandCommand (골격 경로)."""
    pytest.importorskip("dex_retargeting")
    pytest.importorskip("pinocchio")
    from whatslab.core.types import HandPose, InputSample
    from whatslab.teleop.hand import HandRetargetController

    ctrl = HandRetargetController("right", "allegro_hand")   # 내장 URDF
    hand = HandPose.from_sensor_array(np.tile([0, 0, 0, 1.0], (17, 1)), tracked=True)
    cmd = ctrl.compute(InputSample(hand=hand, tracked=True))
    assert cmd.joint_names == ctrl.joint_names
    assert cmd.joint_angles.shape == (16,)
    # 미추적 입력 → 직전 명령 유지 (급변 없음)
    cmd2 = ctrl.compute(InputSample(hand=None, tracked=False))
    assert np.allclose(cmd2.joint_angles, cmd.joint_angles)
