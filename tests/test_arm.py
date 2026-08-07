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
    from whatslab.solvers import ArmIK, DecoupledArmIK, DiffArmIK, backend_cls
    assert backend_cls("dls") is ArmIK
    assert backend_cls("diff") is DiffArmIK
    assert backend_cls("decoupled") is DecoupledArmIK
    with pytest.raises(ValueError):
        backend_cls("nope")


def test_arm_ik_requires_pinocchio():
    if importlib.util.find_spec("pinocchio") is None:
        with pytest.raises(ModuleNotFoundError):
            import whatslab.solvers  # noqa: F401
        pytest.skip("pinocchio 미설치 — solve 검증 생략")
    import whatslab.solvers as solvers
    assert solvers.ArmIK is not None
    assert solvers.backend_cls("dls") is solvers.ArmIK


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
    from whatslab.robot import RobotArmIK
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
    from whatslab.robot import RobotArmIK
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
    from whatslab.teleop.models.quest import QuestModel
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


def test_static_target_settles_without_oscillation():
    pytest.importorskip("pinocchio")
    s = _nero_solver("diff")
    rng = np.random.default_rng(3)
    lo, hi = s._lo, s._hi
    worst = 0.0
    for _ in range(6):
        q_true = lo + (hi - lo) * (0.2 + 0.6 * rng.random(s.nq))
        T = s.fk(q_true)
        s.sync_state(s.q_neutral)
        errs = [s.pose_error(s.solve(T), T)[0] for _ in range(60)]
        worst = max(worst, float(np.mean(errs[-10:])))
    assert worst < 5e-3, f"정지 목표 정착 실패: {worst*1e3:.1f} mm"


def test_reseed_clears_warm_start():
    pytest.importorskip("pinocchio")
    from whatslab.robot import RobotArmIK, RobotModel, load_rig

    robot = RobotModel(load_rig("rigs/nero_arm.yaml"))
    s = robot.solver
    ik = RobotArmIK(robot)
    ik.solve(robot.to_canonical(s.fk(s.q_neutral + 0.4)))
    assert ik._warm is not None
    ik.reseed()
    assert ik._warm is None
    assert ik._seeded is False and ik._cold_tries == 0


def test_cold_start_is_uncapped():
    pytest.importorskip("pinocchio")
    from whatslab.robot import RobotArmIK, RobotModel, load_rig

    rig = load_rig("rigs/nero_arm.yaml")
    robot = RobotModel(rig)
    s = robot.solver
    ik = RobotArmIK(robot)

    lo, hi = s._lo, s._hi
    q_far = lo + (hi - lo) * 0.8
    T_far = robot.to_canonical(s.fk(q_far))
    T_near = robot.to_canonical(s.fk(s.q_neutral + 0.1))

    q0 = np.asarray(ik.solve(T_near), dtype=float)
    ik.reseed()
    q1 = np.asarray(ik.solve(T_far), dtype=float)
    assert float(np.linalg.norm(q1 - q0)) > 0.5
    assert s.pose_error(q1, robot.to_base(T_far))[0] < 5e-3

    ik.reseed()
    ik.cold_pos_tol = 0.0
    ik.cold_max_tries = 3
    prev = np.asarray(ik.solve(T_near), dtype=float)
    assert ik._seeded is False
    q2 = np.asarray(ik.solve(T_far), dtype=float)
    assert float(np.linalg.norm(q2 - prev)) > 0.5


def test_get_data_publishes_raw_target():
    from whatslab.core.types import Pose
    from whatslab.teleop.base import TeleopModel

    pose = Pose(pos=np.array([0.3, 0.0, 0.1]), quat=np.array([0.0, 0.0, 0.0, 1.0]))
    calls = []

    class _M(TeleopModel):
        def _get_raw_target(self):
            calls.append(1)
            return {"left": None, "right": pose}

    m = _M(None)
    assert m.raw_target == {}
    m.get_data()
    assert m.raw_target["right"] is pose
    assert m.raw_target["left"] is None
    assert len(calls) == 1


def test_two_arm_iks_on_one_robot_do_not_interfere():
    """같은 RobotModel 로 만든 두 RobotArmIK 가 서로의 warm-start 를 침범하면 안 된다.

    단일 rig 를 TeleopModel 에 주면 양쪽 side 가 같은 RobotModel(= 같은 solver)을
    공유한다. 솔버가 history_data 를 들고 있으므로, 두 side 가 번갈아 solve 하면
    서로를 밀어낸다(실측: 오른쪽만 2.6mm → 양쪽 347mm).
    """
    pytest.importorskip("pinocchio")
    from whatslab.robot import RobotArmIK, RobotModel, load_rig

    robot = RobotModel(load_rig("rigs/nero_orca_right.yaml"))
    s = robot.solver
    assert robot.solver is robot.solver

    rng = np.random.default_rng(0)
    lo, hi = s._lo, s._hi
    qa = lo + (hi - lo) * 0.40
    qb = lo + (hi - lo) * 0.55
    qc = lo + (hi - lo) * 0.60
    n = 40
    targets = [robot.to_canonical(s.fk(qa + (qb - qa) * (i / (n - 1)))) for i in range(n)]
    other = [robot.to_canonical(s.fk(qc + (qa - qc) * (i / (n - 1)))) for i in range(n)]

    ik_solo = RobotArmIK(robot)
    solo = [np.asarray(ik_solo.solve(T), dtype=float) for T in targets]

    ik_a, ik_b = RobotArmIK(robot), RobotArmIK(robot)
    both = []
    for Ta, Tb in zip(targets, other):
        ik_b.solve(Tb)
        both.append(np.asarray(ik_a.solve(Ta), dtype=float))

    e_solo = np.array([s.pose_error(q, robot.to_base(T))[0] for q, T in zip(solo, targets)])
    e_both = np.array([s.pose_error(q, robot.to_base(T))[0] for q, T in zip(both, targets)])
    assert e_both.mean() < e_solo.mean() + 5e-3, (
        f"다른 side 가 solver 상태를 오염시킨다: 단독 {e_solo.mean()*1e3:.1f}mm "
        f"vs 양쪽 {e_both.mean()*1e3:.1f}mm")


def _orientation_step(robot, dtheta=0.4):
    from scipy.spatial.transform import Rotation
    s = robot.solver
    q0 = 0.5 * (s._lo + s._hi)
    s.sync_state(q0)
    T = s.fk(q0).copy()
    T[:3, :3] = T[:3, :3] @ Rotation.from_euler("y", dtheta).as_matrix()
    q1 = np.asarray(s.solve(robot.to_canonical(T)), dtype=float)
    return np.abs(q1 - q0)


def test_joint_weights_shift_effort_to_cheap_joints():
    pytest.importorskip("pinocchio")
    from whatslab.robot import RobotModel, load_rig

    def build(weights):
        rig = load_rig("rigs/nero_orca_right.yaml")
        rig.solver.joint_weights = weights
        return RobotModel(rig)

    flat = build(None)
    arm = [n for n in flat.arm_joint_names[:4]]
    heavy = build({n: 5.0 for n in arm})
    i_arm = [flat.arm_joint_names.index(n) for n in arm]

    d_flat = _orientation_step(flat)
    d_heavy = _orientation_step(heavy)
    share_flat = d_flat[i_arm].sum() / max(d_flat.sum(), 1e-12)
    share_heavy = d_heavy[i_arm].sum() / max(d_heavy.sum(), 1e-12)
    assert share_heavy < share_flat, (
        f"팔을 무겁게 해도 팔 사용 비중이 안 줄었다: {share_flat:.2f} → {share_heavy:.2f}")


def test_joint_weights_reject_bad_input():
    pytest.importorskip("pinocchio")
    from whatslab.robot import RobotModel, load_rig
    robot = RobotModel(load_rig("rigs/nero_orca_right.yaml"))
    with pytest.raises(ValueError):
        robot.solver.set_joint_weights({"nope": 2.0})
    with pytest.raises(ValueError):
        robot.solver.set_joint_weights({"joint1": 0.0})


def test_decoupled_wrist_center_is_upstream_of_orientation_joints():
    pytest.importorskip("pinocchio")
    from whatslab.robot import RobotModel, load_rig
    rig = load_rig("rigs/nero_orca_right.yaml")
    rig.solver.backend = "decoupled"
    s = RobotModel(rig).solver
    q = 0.5 * (s._lo + s._hi)
    p0 = s.frame_pose(s.wc_frame, q)[:3, 3]
    for i in s._ori_idx:
        q2 = q.copy()
        q2[i] += 0.3
        p = s.frame_pose(s.wc_frame, q2)[:3, 3]
        assert np.linalg.norm(p - p0) < 1e-9, (
            f"방위 관절 {s.model.names[int(i) + 1]} 이 손목중심을 움직인다")
