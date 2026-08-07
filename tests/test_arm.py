import importlib.util

import numpy as np
import pytest


def _nero_solver(backend: str = "dls"):
    from whatslab.robot import RobotModel, load_rig
    rig = load_rig("rigs/nero_arm.yaml")
    rig.calibration.enabled = False
    rig.solver.backend = backend
    rig.arm.reach_max = None
    return RobotModel(rig).solver


def test_backend_cls_selection():
    pytest.importorskip("pinocchio")
    from whatslab.solvers.arm import ArmIK, DiffArmIK, backend_cls
    assert backend_cls("dls") is ArmIK
    assert backend_cls("diff") is DiffArmIK
    with pytest.raises(ValueError):
        backend_cls("nope")


def test_arm_ik_lazy_requires_pinocchio():
    import whatslab.solvers.arm as arm

    if importlib.util.find_spec("pinocchio") is None:
        with pytest.raises(ModuleNotFoundError):
            _ = arm.ArmIK
        pytest.skip("pinocchio 미설치 — solve 검증 생략")
    else:
        assert arm.ArmIK is not None


def test_dls_end_to_end_bundled_urdf():
    pytest.importorskip("pinocchio")
    s = _nero_solver("dls")
    assert s.nq == 7
    lo = np.where(np.isfinite(s.model.lowerPositionLimit), s.model.lowerPositionLimit, -np.pi)
    hi = np.where(np.isfinite(s.model.upperPositionLimit), s.model.upperPositionLimit, np.pi)
    rng = np.random.default_rng(0)
    worst = 0.0
    for _ in range(15):
        q_true = lo + (hi - lo) * (0.15 + 0.70 * rng.random(s.nq))
        T = s.fk(q_true)
        s.sync_state(q_true + 0.1 * rng.standard_normal(s.nq))
        for _ in range(12):
            q = s.solve(T)
        worst = max(worst, s.pose_error(q, T)[0])
    assert worst < 5e-3, f"pos 오차 과다: {worst*1e3:.2f} mm"


def test_arm_ik_no_casadi_dependency():
    pytest.importorskip("pinocchio")
    import whatslab.solvers.arm.arm_ik as m
    src = open(m.__file__, encoding="utf-8").read()
    assert "import casadi" not in src
    assert "from pinocchio import casadi" not in src


def test_diff_backend_tracks_and_is_continuous():
    pytest.importorskip("pinocchio")
    s = _nero_solver("diff")
    lo = np.where(np.isfinite(s.model.lowerPositionLimit), s.model.lowerPositionLimit, -np.pi)
    hi = np.where(np.isfinite(s.model.upperPositionLimit), s.model.upperPositionLimit, np.pi)
    rng = np.random.default_rng(0)
    worst = 0.0
    for _ in range(10):
        q_true = lo + (hi - lo) * (0.2 + 0.6 * rng.random(s.nq))
        T = s.fk(q_true)
        s.sync_state(q_true + 0.3 * rng.standard_normal(s.nq))
        prev = s.history_data.copy()
        for _ in range(120):
            q = s.solve(T)
            assert np.linalg.norm(q - prev) < 0.6, "틱당 관절 점프 과다"
            prev = q
        worst = max(worst, s.pose_error(q, T)[0])
    assert worst < 5e-3, f"diff 추종 수렴 실패: {worst*1e3:.2f}mm"


def test_robot_arm_ik_no_teleport_on_teleop_targets():
    pytest.importorskip("pinocchio")
    from scipy.spatial.transform import Rotation
    from whatslab.teleop.ik import RobotArmIK
    from whatslab.robot import RobotModel, load_rig

    ik = RobotArmIK(RobotModel(load_rig("rigs/nero_orca_right.yaml")))
    prev, worst = None, 0.0
    for i in range(300):
        t = i / 300
        T = np.eye(4)
        T[:3, 3] = [0.55 + 0.15 * np.sin(2 * np.pi * t), 0.25 * np.sin(4 * np.pi * t), 0.15]
        T[:3, :3] = Rotation.from_euler(
            "xyz", [1.2 * np.sin(2 * np.pi * t), 0.8 * np.sin(3 * np.pi * t),
                    1.5 * np.sin(1.5 * np.pi * t)]).as_matrix()
        q = np.asarray(ik.solve(T), dtype=float)
        assert np.all(np.isfinite(q))
        if prev is not None:
            worst = max(worst, float(np.linalg.norm(q - prev)))
        prev = q
    assert worst < 0.6, f"틱당 관절 점프 과다(EE 순간이동): {worst:.2f} rad"


def _orca_rig_ik():
    from whatslab.teleop.ik import RobotArmIK
    from whatslab.robot import RobotModel, load_rig
    robot = RobotModel(load_rig("rigs/nero_orca_right.yaml"))
    return robot, RobotArmIK(robot)


def _fk_targets(robot, n=200):
    s = robot.solver
    mid = 0.5 * (s._lo + s._hi)
    amp = 0.25 * (s._hi - s._lo)
    phase = np.linspace(0.0, 2.0, s.nq)
    return [robot.to_canonical(s.fk(mid + amp * np.sin(2 * np.pi * (i / n + phase))))
            for i in range(n)]


def _wave_targets(n=200):
    from scipy.spatial.transform import Rotation
    out = []
    for i in range(n):
        t = i / n
        T = np.eye(4)
        T[:3, 3] = [0.55 + 0.15 * np.sin(2 * np.pi * t), 0.25 * np.sin(4 * np.pi * t), 0.15]
        T[:3, :3] = Rotation.from_euler(
            "xyz", [1.2 * np.sin(2 * np.pi * t), 0.8 * np.sin(3 * np.pi * t),
                    1.5 * np.sin(1.5 * np.pi * t)]).as_matrix()
        out.append(T)
    return out


def test_robot_arm_ik_accurate_on_reachable_targets():
    pytest.importorskip("pinocchio")
    robot, ik = _orca_rig_ik()
    errs = [robot.solver.pose_error(ik.solve(T), robot.clamp_reach(robot.to_base(T)))[0]
            for T in _fk_targets(robot)]
    errs = np.array(errs[5:])
    assert errs.mean() < 0.010, f"평균 pos 오차 과다: {errs.mean()*1000:.1f} mm"
    assert np.percentile(errs, 95) < 0.020, f"p95 pos 오차 과다: {np.percentile(errs,95)*1000:.1f} mm"


def test_robot_arm_ik_no_teleport_on_unreachable_targets():
    pytest.importorskip("pinocchio")
    _, ik = _orca_rig_ik()
    qs = [np.asarray(ik.solve(T), dtype=float) for T in _wave_targets()]
    dq = np.linalg.norm(np.diff(np.array(qs), axis=0), axis=1)
    assert np.all(np.isfinite(qs))
    assert dq.max() < 0.5, f"틱당 관절 점프 과다(EE 순간이동): {dq.max():.2f} rad"


def test_solve_robust_is_accurate_from_cold_state():
    pytest.importorskip("pinocchio")
    from whatslab.robot import RobotModel, load_rig

    s = RobotModel(load_rig("rigs/nero_orca_right.yaml")).solver
    lo, hi = s._lo, s._hi
    per_smooth = {}
    for smooth in (0.0, 0.9):
        s._smooth = smooth
        rng = np.random.default_rng(0)
        worst = 0.0
        for _ in range(8):
            q_true = lo + (hi - lo) * (0.2 + 0.6 * rng.random(s.nq))
            T = s.fk(q_true)
            s.sync_state(np.zeros(s.nq))
            worst = max(worst, s.pose_error(s.solve_robust(T), T)[0])
        per_smooth[smooth] = worst
        assert worst < 1e-3, f"solve_robust 오차 과다(_smooth={smooth}): {worst*1000:.2f} mm"
    assert per_smooth[0.0] == pytest.approx(per_smooth[0.9], abs=1e-12), \
        f"solve_robust 가 _smooth 에 의존: {per_smooth}"


def test_global_search_is_event_driven_not_per_frame():
    pytest.importorskip("pinocchio")
    robot, ik = _orca_rig_ik()
    calls = {"n": 0}
    orig = robot.solver.solve_robust

    def spy(*a, **k):
        calls["n"] += 1
        return orig(*a, **k)
    robot.solver.solve_robust = spy
    n = 200
    for T in _wave_targets(n):
        ik.solve(T)
    assert calls["n"] <= 1 + n // 20, \
        f"전역 탐색이 너무 자주 발동: {calls['n']}회 / {n} 프레임"


def test_diff_backend_no_divergence_on_unreachable():
    pytest.importorskip("pinocchio")
    s = _nero_solver("diff")
    T = np.eye(4); T[:3, 3] = [2.0, 0.0, 0.5]
    s.sync_state(np.zeros(s.nq))
    prev = s.history_data.copy()
    for _ in range(150):
        q = s.solve(T)
        assert np.all(np.isfinite(q))
        assert np.linalg.norm(q - prev) < 0.6
        prev = q
    q2 = s.solve(T)
    assert np.linalg.norm(q2 - q) < 0.05


def _cal_pose(pos, quat=(0.0, 0.0, 0.0, 1.0)):
    from whatslab.core.types import Pose
    return Pose(pos=np.array(pos, dtype=float), quat=np.array(quat, dtype=float))


def test_calib_disabled_skips_reach_scale_but_keeps_yaw():
    from scipy.spatial.transform import Rotation

    from whatslab.teleop.calibration import ArmCalibration

    cal = ArmCalibration(reach_max=1.0, input_reach=0.5, enabled=False)
    q = Rotation.from_euler("z", 0.9).as_quat()
    assert cal.capture({"arm_pose": _cal_pose([0.4, 0.1, 0.2], q)}) is True
    T = cal.apply({"arm_pose": _cal_pose([0.3, -0.1, 0.2], q)})["arm_target"]
    assert T[:3, 3] == pytest.approx([0.3, -0.1, 0.2])
    T0 = cal.apply({"arm_pose": _cal_pose([0.4, 0.1, 0.2], q)})["arm_target"]
    assert T0[:3, :3] == pytest.approx(np.eye(3), abs=1e-9)


def test_calib_enabled_applies_scale_and_yaw():
    from scipy.spatial.transform import Rotation

    from whatslab.teleop.calibration import ArmCalibration

    cal = ArmCalibration(reach_max=1.0, input_reach=0.5, enabled=True)
    q = Rotation.from_euler("z", 0.9).as_quat()
    cal.capture({"arm_pose": _cal_pose([0.4, 0.1, 0.2], q)})
    T = cal.apply({"arm_pose": _cal_pose([0.3, -0.1, 0.2], q)})["arm_target"]
    assert T[:3, 3] == pytest.approx([0.6, -0.2, 0.4])
    T0 = cal.apply({"arm_pose": _cal_pose([0.4, 0.1, 0.2], q)})["arm_target"]
    assert T0[:3, :3] == pytest.approx(np.eye(3), abs=1e-9)


def test_calib_enabled_flag_reaches_teleop_path():
    pytest.importorskip("pinocchio")
    from whatslab.teleop.quest import QuestModel
    from whatslab.robot import RobotModel, load_rig

    pose = _cal_pose([0.45, -0.10, 0.05])
    out = {}
    for flag in (True, False):
        rig = load_rig("rigs/nero_orca_right.yaml")
        rig.calibration.enabled = flag
        m = QuestModel(RobotModel(rig))
        assert m.calib["right"].enabled is flag
        out[flag] = m.calib["right"].apply({"arm_pose": pose})["arm_target"][:3, 3]

    assert out[False] == pytest.approx(pose.pos)
    scale = rig.solver.reach_max / rig.calibration.input_reach
    assert out[True] == pytest.approx(np.asarray(pose.pos) * scale)
    assert not np.allclose(out[True], out[False])


def test_yaw_calibration_works_regardless_of_enabled():
    from scipy.spatial.transform import Rotation

    from whatslab.teleop.calibration import ArmCalibration

    q = Rotation.from_euler("z", 1.2).as_quat()
    for flag in (True, False):
        cal = ArmCalibration(reach_max=1.0, input_reach=1.0, enabled=flag)
        assert cal.ready is False
        assert cal.capture({"arm_pose": _cal_pose([0.4, 0.1, 0.2], q)}) is True
        assert cal.ready is True
        T = cal.apply({"arm_pose": _cal_pose([0.4, 0.1, 0.2], q)})["arm_target"]
        assert T[:3, :3] == pytest.approx(np.eye(3), abs=1e-9)
