import numpy as np
import pytest

from whatslab.solvers.hand.hand_configs import CONFIG_REGISTRY


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
    from whatslab.solvers.hand import HandRetargeter

    r = HandRetargeter("right", "allegro_hand")
    assert len(r.joint_names) == 16
    assert r.tip_human_indices
    assert len(r.human_joint_names) > 0
    qpos = r.compute({n: 0.0 for n in r.human_joint_names})
    assert qpos.shape == (16,)
    assert np.all(np.isfinite(qpos))
    assert np.allclose(r._wrist_offset, [0, 0, -0.065], atol=1e-6)
    assert np.allclose(r.last_human_positions[0], 0.0)


def test_kp_retargeter_end_to_end():
    pytest.importorskip("dex_retargeting")
    pytest.importorskip("pinocchio")
    from whatslab.solvers.hand import KPHandRetargeter

    r = KPHandRetargeter("right", "orca_hand")
    assert r.scale > 0
    assert set(r.keypoints) == {"thumb", "index", "middle", "ring", "pinky"}
    neutral = {n: 0.0 for n in r.human_joint_names}
    q = r.compute(neutral)
    assert q.shape == (len(r.joint_names),)
    assert np.all(np.isfinite(q))
    for _ in range(10):
        q = r.compute(neutral)
    q2 = r.compute(neutral)
    assert np.allclose(q, q2, atol=1e-3)
    r.reset()
    assert r._cold


def test_kp_retargeter_tracks_tips():
    pytest.importorskip("dex_retargeting")
    pytest.importorskip("pinocchio")
    from whatslab.solvers.hand import KPHandRetargeter

    r = KPHandRetargeter("right", "robotis_hx5_d20")
    curled = {n: (0.6 if "index" in n else 0.15) for n in r.human_joint_names}
    for _ in range(20):
        r.compute(curled)
    T = r._targets(r._human_points(curled))
    r._fk_robot(r._q)
    err = np.mean([np.linalg.norm(r._pos(r._fids[f][-1]) - T[f][-1])
                   for f in T])
    assert err < 0.05


def test_kp_retargeter_snap_rows():
    pytest.importorskip("dex_retargeting")
    pytest.importorskip("pinocchio")
    from whatslab.solvers.hand import KPHandRetargeter
    from whatslab.solvers.hand.kp_retargeter import SNAP_CONTACT

    r = KPHandRetargeter("right", "robotis_hx5_d20")
    T = r._targets(r._human_points({n: 0.0 for n in r.human_joint_names}))
    far, _ = r._pair_rows(T, snap=True)
    for vt, w in far.values():
        assert w == r.w_pair

    T["thumb"] = T["thumb"] + (T["index"][-1] - T["thumb"][-1]) - np.array([0.005, 0, 0])
    near, _ = r._pair_rows(T, snap=True)
    vt, w = near[("thumb", "index")]
    assert w == r.w_snap
    assert np.linalg.norm(vt) < 2 * SNAP_CONTACT


def test_kp_controller_backend():
    pytest.importorskip("dex_retargeting")
    pytest.importorskip("pinocchio")
    from whatslab.core.types import HandPose, InputSample
    from whatslab.solvers.hand import HandRetargetController, KPHandRetargeter

    ctrl = HandRetargetController("right", "orca_hand", backend="kp")
    assert isinstance(ctrl.engine, KPHandRetargeter)
    angles = {n: 0.0 for n in ctrl.engine.human_joint_names}
    hand = HandPose(joint_angles=angles, tracked=True)
    cmd = ctrl.compute(InputSample(hand=hand, tracked=True))
    assert cmd.joint_names == ctrl.joint_names
    assert np.all(np.isfinite(cmd.joint_angles))
    with pytest.raises(ValueError):
        HandRetargetController("right", "orca_hand", backend="nope")


def test_hand_controller_from_input_sample():
    pytest.importorskip("dex_retargeting")
    pytest.importorskip("pinocchio")
    from whatslab.core.types import HandPose, InputSample
    from whatslab.solvers.hand import HandRetargetController

    ctrl = HandRetargetController("right", "allegro_hand")
    angles = {n: 0.0 for n in ctrl.engine.human_joint_names}
    hand = HandPose(joint_angles=angles, tracked=True)
    cmd = ctrl.compute(InputSample(hand=hand, tracked=True))
    assert cmd.joint_names == ctrl.joint_names
    assert cmd.joint_angles.shape == (16,)
    cmd2 = ctrl.compute(InputSample(hand=None, tracked=False))
    assert np.allclose(cmd2.joint_angles, cmd.joint_angles)
