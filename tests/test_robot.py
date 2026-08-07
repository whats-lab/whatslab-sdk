import os
import numpy as np
import pytest

from whatslab.robot import RobotModel, RobotSpec, configs_root, load_rig


def test_configs_root_and_load_rig():
    root = configs_root()
    assert root, "레포 configs/ 미발견 (editable 설치 전제)"
    rig = load_rig("rigs/nero_orca_right.yaml")
    assert rig.arm is not None and rig.arm.kind == "arm"
    assert rig.hand is not None and rig.hand.kind == "hand"
    assert isinstance(rig.lock_joints, list)
    assert rig.resolve_target_ee() == "R-Carpals_8d1f1041"


def test_robot_spec_validation():
    with pytest.raises(ValueError):
        RobotSpec.from_dict({"name": "x", "kind": "arm", "urdf": "u"})
    with pytest.raises(ValueError):
        RobotSpec.from_dict({"name": "x", "kind": "hand", "urdf": "u"})
    with pytest.raises(ValueError):
        RobotSpec.from_dict({"name": "x", "kind": "leg", "urdf": "u"})


def test_target_ee_fallback_chain():
    rig = load_rig("rigs/nero_orca_right.yaml")
    rig.target_ee = "custom_frame"
    assert rig.resolve_target_ee() == "custom_frame"
    rig.target_ee = None
    rig.hand = None
    assert rig.resolve_target_ee() == "ee"


def test_model_canonical_sandwich():
    pytest.importorskip("pinocchio")
    rig = load_rig("rigs/nero_orca_right.yaml")
    rig.calibration.enabled = False
    m = RobotModel(rig)
    assert m.has_arm and m.has_hand
    assert m.arm_joint_names[:7] == [f"joint{i}" for i in range(1, 8)]
    assert len(m.arm_joint_names) == 8

    q0 = np.full(len(m.arm_joint_names), 0.3)
    T_c = m.ee_pose(q0)
    T_b = m.solver.fk(q0)
    assert np.allclose(T_c, m.to_canonical(T_b))
    assert np.allclose(m.to_base(T_c), T_b, atol=1e-12)
    assert np.sign(T_c[0, 3]) == -np.sign(T_b[0, 3]) or abs(T_b[0, 3]) < 1e-9

    m.sync_state(q0 + 0.1)
    for _ in range(80):
        q = m.solve(T_c)
    err = np.linalg.norm(m.ee_pose(q)[:3, 3] - T_c[:3, 3])
    assert err < 5e-3, f"sandwich solve 오차 {err*1e3:.2f}mm"


def test_model_arm_only_and_hand_only():
    pytest.importorskip("pinocchio")
    rig = load_rig("rigs/nero_orca_right.yaml")
    rig_a = load_rig("rigs/nero_orca_right.yaml")
    rig_a.hand = None
    rig_a.target_ee = None
    rig_a.lock_joints = ["joint7"]
    with pytest.raises(ValueError):
        RobotModel(rig_a)
    rig_a.lock_joints = []
    m = RobotModel(rig_a)
    assert m.has_arm and not m.has_hand
    assert m.arm_joint_names[:7] == [f"joint{i}" for i in range(1, 8)]
    assert len(m.arm_joint_names) == 7
    q0 = np.zeros(len(m.arm_joint_names))
    assert m.ee_pose(q0).shape == (4, 4)
    rig_h = load_rig("rigs/nero_orca_right.yaml")
    rig_h.arm = None
    m2 = RobotModel(rig_h)
    assert m2.has_hand and not m2.has_arm and m2.solver is None


def test_model_reach_clamp():
    pytest.importorskip("pinocchio")
    rig = load_rig("rigs/nero_orca_right.yaml")
    rig.calibration.enabled = False
    rig.solver.reach_max = 0.7
    m = RobotModel(rig)
    T = np.eye(4); T[:3, 3] = [3.0, 0.0, 0.0]
    m.sync_state(np.zeros(len(m.arm_joint_names)))
    for _ in range(150):
        q = m.solve(T)
    r = float(np.linalg.norm(m.solver.fk(q)[:3, 3]))
    assert r <= 0.7 + 1e-2, f"reach 클램프 실패: 반경 {r:.3f} > 0.7 (먼 목표가 안 잘림)"


def test_model_uniform_reach_scale():
    pytest.importorskip("pinocchio")
    rig = load_rig("rigs/nero_arm.yaml")
    rig.solver.reach_max = 0.9
    m = RobotModel(rig)
    n = len(m.arm_joint_names)
    T = np.eye(4); T[:3, 3] = [0.2, 0.0, 0.1]

    rig.calibration.enabled = True
    rig.calibration.input_reach = 0.45
    m.sync_state(np.zeros(n)); q_cal = m.solve(T)

    rig.calibration.enabled = False
    T2 = np.eye(4); T2[:3, 3] = [0.4, 0.0, 0.2]
    m.sync_state(np.zeros(n)); q_manual = m.solve(T2)
    assert np.allclose(q_cal, q_manual, atol=1e-6)


def test_robot_model_accepts_path_or_config():
    pytest.importorskip("pinocchio")
    from whatslab.robot import RobotModel, load_rig

    rig = load_rig("rigs/nero_orca_right.yaml")
    a = RobotModel(rig)
    b = RobotModel("rigs/nero_orca_right.yaml")
    c = RobotModel.from_yaml("rigs/nero_orca_right.yaml")

    for m in (b, c):
        assert m.rig.name == a.rig.name
        assert m.arm_joint_names == a.arm_joint_names
        assert np.allclose(m._M, a._M)
    assert a.rig is rig


def test_robot_model_accepts_pathlike():
    pytest.importorskip("pinocchio")
    from pathlib import Path

    from whatslab.paths import configs_root
    from whatslab.robot import RobotModel

    p = Path(configs_root()) / "rigs" / "nero_orca_right.yaml"
    assert RobotModel(p).rig.name == "nero_orca_right"


def test_teleop_model_accepts_rig_path():
    pytest.importorskip("pinocchio")
    from whatslab.teleop import QuestModel

    m = QuestModel("rigs/nero_orca_right.yaml")
    assert m.robot is not None
    assert m.robot.rig.name == "nero_orca_right"
    assert m.robots["left"] is not m.robots["right"], (
        "side 가 RobotModel 을 공유하면 유상태 솔버도 공유된다")
    assert m.robots["left"].solver is not m.robots["right"].solver
    assert m.robots["left"].rig is m.robots["right"].rig


def test_save_calibration_is_atomic_and_keeps_enabled(tmp_path):
    import shutil

    import yaml

    from whatslab.paths import configs_root
    from whatslab.robot import load_rig, save_calibration, save_reach_max

    src = os.path.join(configs_root(), "rigs", "nero_orca_right.yaml")
    dst = tmp_path / "rig.yaml"
    d = yaml.safe_load(open(src))
    d["calibration"]["enabled"] = False
    d["robots"] = {k: os.path.join(configs_root(), v)
                   for k, v in d["robots"].items()}
    dst.write_text(yaml.safe_dump(d, allow_unicode=True, sort_keys=False))
    shutil.copy(src, tmp_path / "orig.yaml")

    rig = load_rig(str(dst))
    assert rig.calibration.enabled is False
    save_calibration(rig, 0.77)

    back = yaml.safe_load(dst.read_text())
    assert back["calibration"]["input_reach"] == 0.77
    assert back["calibration"]["enabled"] is False
    assert back["solver"]["backend"] == d["solver"]["backend"]

    save_reach_max(rig, 0.88)
    assert yaml.safe_load(dst.read_text())["solver"]["reach_max"] == 0.88
    assert not list(tmp_path.glob("*.tmp.*")), "임시 파일이 남았다"
