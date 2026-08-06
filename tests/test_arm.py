"""arm — 수치 IK 백엔드(dls/diff) 검증. 로봇 조립은 rig 경유 (whatslab.robot)."""
import importlib.util

import numpy as np
import pytest


def _nero_solver(backend: str = "dls"):
    """nero 팔 단독 rig 로 백엔드 솔버 생성 (클램프/매핑 없이 순수 수치 검증)."""
    from whatslab.robot import RobotModel, load_rig
    rig = load_rig("rigs/nero_arm.yaml")
    rig.calibration.enabled = False
    rig.solver.backend = backend
    rig.arm.reach_max = None
    return RobotModel(rig).solver


def test_backend_cls_selection():
    pytest.importorskip("pinocchio")
    from whatslab.teleop.arm import ArmIK, DiffArmIK, backend_cls
    assert backend_cls("dls") is ArmIK
    assert backend_cls("diff") is DiffArmIK
    with pytest.raises(ValueError):
        backend_cls("nope")


def test_arm_ik_lazy_requires_pinocchio():
    """pinocchio 없으면 ArmIK 접근이 ModuleNotFoundError; 있으면 심볼 로드."""
    import whatslab.teleop.arm as arm

    if importlib.util.find_spec("pinocchio") is None:
        with pytest.raises(ModuleNotFoundError):
            _ = arm.ArmIK
        pytest.skip("pinocchio 미설치 — solve 검증 생략")
    else:
        assert arm.ArmIK is not None


def test_dls_end_to_end_bundled_urdf():
    """내장 nero URDF 로 dls 왕복: random q → FK → solve → pose 오차 ~0."""
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
    """arm_ik 는 casadi 를 import 하지 않아야 (순수 pip 스택 보장)."""
    pytest.importorskip("pinocchio")
    import whatslab.teleop.arm.arm_ik as m
    src = open(m.__file__, encoding="utf-8").read()
    assert "import casadi" not in src
    assert "from pinocchio import casadi" not in src


def test_diff_backend_tracks_and_is_continuous():
    """diff 백엔드는 '추종기' — warm 시작에서 고정 목표 수렴 + 틱당 연속성."""
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
    """RobotArmIK: 추종 오차가 남는 목표를 따라가도 틱당 관절 점프가 없어야 한다.

    회귀 방지 — 스톨 자동 reseed 가 전역 재탐색(solve_robust) 해를 무조건 채택하면,
    도달 불가 목표(실효 6축으로 임의 6D 자세를 다 맞출 수 없음)에서 주기적으로
    발동해 EE 가 순간이동한다(실측 최대 5.4 rad/틱 = 325 rad/s).
    """
    pytest.importorskip("pinocchio")
    from scipy.spatial.transform import Rotation
    from whatslab.model.ik import RobotArmIK
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
    from whatslab.model.ik import RobotArmIK
    from whatslab.robot import RobotModel, load_rig
    robot = RobotModel(load_rig("rigs/nero_orca_right.yaml"))
    return robot, RobotArmIK(robot)


def _fk_targets(robot, n=200):
    """도달 가능이 보장된 정준 목표 — 유효 q 를 흔들고 FK. 오차 하한 = 0."""
    s = robot.solver
    mid = 0.5 * (s._lo + s._hi)
    amp = 0.25 * (s._hi - s._lo)
    phase = np.linspace(0.0, 2.0, s.nq)
    return [robot.to_canonical(s.fk(mid + amp * np.sin(2 * np.pi * (i / n + phase))))
            for i in range(n)]


def _wave_targets(n=200):
    """도달범위 안에서 움직이되 자세는 임의(컨트롤러 유사) — 추종 오차가 상시 남는다."""
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
    """도달 가능 목표(FK 로 생성)에서는 오차가 0 에 가까워야 한다.

    합성 좌표 궤적은 도달 불가 구간이 섞여 솔버 품질을 가린다 → 정확도 판정은
    반드시 이 궤적으로. (도구: tools/bench_arm_ik.py --traj fk)
    """
    pytest.importorskip("pinocchio")
    robot, ik = _orca_rig_ik()
    errs = [robot.solver.pose_error(ik.solve(T), robot.clamp_reach(robot.to_base(T)))[0]
            for T in _fk_targets(robot)]
    errs = np.array(errs[5:])                  # 첫 몇 틱은 cold-start 시드 구간
    assert errs.mean() < 0.010, f"평균 pos 오차 과다: {errs.mean()*1000:.1f} mm"
    assert np.percentile(errs, 95) < 0.020, f"p95 pos 오차 과다: {np.percentile(errs,95)*1000:.1f} mm"


def test_robot_arm_ik_no_teleport_on_unreachable_targets():
    """추종 오차가 남는 목표에서도 틱당 관절 점프가 없어야 한다.

    회귀 방지 — 전역 재탐색(solve_robust) 해를 무조건 채택하면 도달 불가 구간에서
    주기적으로 발동해 EE 가 순간이동한다(실측 5.4 rad/틱 = 325 rad/s).
    """
    pytest.importorskip("pinocchio")
    _, ik = _orca_rig_ik()
    qs = [np.asarray(ik.solve(T), dtype=float) for T in _wave_targets()]
    dq = np.linalg.norm(np.diff(np.array(qs), axis=0), axis=1)
    assert np.all(np.isfinite(qs))
    assert dq.max() < 0.5, f"틱당 관절 점프 과다(EE 순간이동): {dq.max():.2f} rad"


def test_solve_robust_is_accurate_from_cold_state():
    """전역 탐색은 cold 상태에서 도달 가능 목표를 sub-mm 로 맞춰야 한다.

    회귀 방지 — `solve_robust` 는 후보마다 `history_data = q0`(랜덤 시드)로 놓고
    `ArmIK.solve` 를 부른다. 그 끝의 출력 EMA(`_smooth`)를 켜둔 채 평가하면 수렴한
    해가 **랜덤 시드로 되섞여** 오차가 폭증한다(실측 `_smooth=0.2`: 45mm, 0: 0.03mm).
    연속 solve() 경로만 보는 테스트로는 잡히지 않는다 — 여기서만 잡힌다.
    """
    pytest.importorskip("pinocchio")
    from whatslab.robot import RobotModel, load_rig

    s = RobotModel(load_rig("rigs/nero_orca_right.yaml")).solver
    lo, hi = s._lo, s._hi
    per_smooth = {}
    for smooth in (0.0, 0.9):            # 극단 EMA 에서도 결과가 **동일**해야 한다
        s._smooth = smooth
        rng = np.random.default_rng(0)
        worst = 0.0
        for _ in range(8):
            q_true = lo + (hi - lo) * (0.2 + 0.6 * rng.random(s.nq))
            T = s.fk(q_true)
            s.sync_state(np.zeros(s.nq))     # cold
            worst = max(worst, s.pose_error(s.solve_robust(T), T)[0])
        per_smooth[smooth] = worst
        assert worst < 1e-3, f"solve_robust 오차 과다(_smooth={smooth}): {worst*1000:.2f} mm"
    # 출력 평활은 추종용 후처리다 — 전역 탐색 결과에 영향을 주면 안 된다(구조적 격리).
    assert per_smooth[0.0] == pytest.approx(per_smooth[0.9], abs=1e-12), \
        f"solve_robust 가 _smooth 에 의존: {per_smooth}"


def test_global_search_is_event_driven_not_per_frame():
    """`solve_robust` 는 basin 재선택 시점에만 불려야 한다(매 프레임 금지).

    후보 하나가 full-convergence DLS(수 ms)라 매 프레임 다중 재시작은 60Hz 예산을
    넘는다. 역할 분리(추종=solve / 전역=solve_robust)를 구조로 고정한다.
    """
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
    """diff 백엔드: 도달 불가 목표(2m)에서도 발산/NaN 없이 경계에서 안정."""
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



def test_null_projector_is_idempotent_and_leak_free():
    """N = I − J⁺J 는 투영자여야 한다 — J⁺ 가 태스크 해의 감쇠를 공유하면 아니다.

    공유하면 널스페이스 항(한계회피·선호자세)이 주태스크로 새어 들어가
    "주태스크 불간섭" 이라는 전제가 깨진다.
    """
    pytest.importorskip("pinocchio")
    import pinocchio as pin
    s = _nero_solver("diff")
    rng = np.random.default_rng(0)
    worst_idem = worst_leak = 0.0
    for _ in range(50):
        q = pin.integrate(s.model, s.q_neutral, rng.uniform(-1.5, 1.5, s.model.nv))
        T = s.fk(pin.integrate(s.model, q, rng.uniform(-0.4, 0.4, s.model.nv)))
        e, J = s._error_and_jac(q, T)
        w = s._task_w
        WJ = w[:, None] * J
        N = s._null_projector(WJ, np.eye(s.model.nv))
        worst_idem = max(worst_idem, float(np.linalg.norm(N @ N - N)))
        v = N @ rng.standard_normal(s.model.nv)
        leak = np.linalg.norm(WJ @ v) / max(np.linalg.norm(WJ) * np.linalg.norm(v), 1e-12)
        worst_leak = max(worst_leak, float(leak))
    assert worst_idem < 1e-9, f"N 이 멱등이 아니다: ‖N²−N‖={worst_idem:.2e}"
    assert worst_leak < 1e-9, f"주태스크 누설 {worst_leak*100:.6f}%"


def test_dtheta_max_caps_orientation_demand_not_target_delta():
    """dtheta_max 는 '현재 EE 자세 → 목표' 각도를 자른다(목표의 변화량이 아니다)."""
    pytest.importorskip("pinocchio")
    from scipy.spatial.transform import Rotation
    s = _nero_solver("diff")
    s.dtheta_max = 0.1
    q = s.q_neutral
    T_cur = s.fk(q)
    T_goal = T_cur.copy()
    T_goal[:3, :3] = T_cur[:3, :3] @ Rotation.from_rotvec([0.0, 0.0, 2.0]).as_matrix()

    T_lim = s._rate_limited_target(q, T_goal)
    ang = np.linalg.norm(Rotation.from_matrix(
        T_cur[:3, :3].T @ T_lim[:3, :3]).as_rotvec())
    assert ang == pytest.approx(0.1, abs=1e-6)


def test_rig_exposes_diff_step_and_projector_tuning():
    """코드 기본값 대신 rig solver: 에서 튜닝되는지 — 변경이 diff 에 남아야 한다."""
    pytest.importorskip("pinocchio")
    from whatslab.robot import RobotModel, load_rig
    rig = load_rig("rigs/nero_orca_right.yaml")
    assert rig.solver.dtheta_max == pytest.approx(0.25)
    assert rig.solver.proj_rcond == pytest.approx(1e-6)
    assert rig.solver.k_limit == pytest.approx(0.30)

    rig.solver.dtheta_max = 1.5
    rig.solver.k_limit = 0.7
    rig.solver.proj_rcond = 1e-8
    s = RobotModel(rig).solver
    assert s.dtheta_max == pytest.approx(1.5)
    assert s._k_limit == pytest.approx(0.7)
    assert s.proj_rcond == pytest.approx(1e-8)


# ─────────────────────────────────────────────────────────────────────────
# 캘리브 on/off (calibration.enabled) + 캘리브 원점 p0
#
# on  : target = scale·(p0 + W(p − p0)),  rot = W·G     (p0/W 는 capture 시점)
# off : target = p,  rot = G                            (리시버 좌표를 그대로 신뢰)
#
# enabled 는 지금까지 텔레옵 경로에서 무시됐다 — 껐다 켤 수가 없었다.
# ─────────────────────────────────────────────────────────────────────────
def _cal_pose(pos, quat=(0.0, 0.0, 0.0, 1.0)):
    from whatslab.core.types import Pose
    return Pose(pos=np.array(pos, dtype=float), quat=np.array(quat, dtype=float))


def test_calib_disabled_passes_receiver_pose_through():
    from scipy.spatial.transform import Rotation

    from whatslab.model.calibration import ArmCalibration

    cal = ArmCalibration(reach_max=1.0, input_reach=0.5, enabled=False)
    q = Rotation.from_euler("z", 0.9).as_quat()
    cal.capture({"arm_pose": _cal_pose([0.4, 0.1, 0.2], q)})   # 캡처해도 무시
    T = cal.apply({"arm_pose": _cal_pose([0.3, -0.1, 0.2], q)})["arm_target"]
    assert T[:3, 3] == pytest.approx([0.3, -0.1, 0.2])         # 스케일 없음
    assert T[:3, :3] == pytest.approx(Rotation.from_quat(q).as_matrix())   # W 없음


def test_calib_enabled_uses_captured_origin():
    from whatslab.model.calibration import ArmCalibration

    cal = ArmCalibration(reach_max=0.9, input_reach=0.9)       # scale 1.0
    p0 = [0.40, -0.12, 0.18]
    assert cal.capture({"arm_pose": _cal_pose(p0)}) is True
    assert cal.anchor == pytest.approx(p0)

    T = cal.apply({"arm_pose": _cal_pose(p0)})["arm_target"]
    assert T[:3, 3] == pytest.approx(p0)                       # 캘리브 지점 = 원점

    T = cal.apply({"arm_pose": _cal_pose(np.array(p0) + [0.1, 0.0, 0.0])})["arm_target"]
    assert T[:3, 3] == pytest.approx(np.array(p0) + [0.1, 0.0, 0.0])


def test_calib_origin_rotates_only_the_displacement():
    from scipy.spatial.transform import Rotation

    from whatslab.model.calibration import ArmCalibration

    cal = ArmCalibration(reach_max=1.0, input_reach=1.0)
    yaw = 0.7
    q = Rotation.from_euler("z", yaw).as_quat()
    p0 = np.array([0.4, 0.1, 0.2])
    cal.capture({"arm_pose": _cal_pose(p0, q)})                # W = Rz(-yaw)
    d = np.array([0.10, 0.05, 0.0])
    T = cal.apply({"arm_pose": _cal_pose(p0 + d, q)})["arm_target"]
    assert T[:3, 3] == pytest.approx(p0 + Rotation.from_euler("z", -yaw).as_matrix() @ d)


def test_calib_before_capture_is_scale_only():
    from whatslab.model.calibration import ArmCalibration

    cal = ArmCalibration(reach_max=1.0, input_reach=0.5)       # scale 2.0
    T = cal.apply({"arm_pose": _cal_pose([0.25, 0.0, 0.1])})["arm_target"]
    assert T[:3, 3] == pytest.approx([0.5, 0.0, 0.2])


def test_calib_enabled_flag_reaches_model_from_rig():
    pytest.importorskip("pinocchio")
    from whatslab.model.quest import QuestModel
    from whatslab.robot import RobotModel, load_rig

    rig = load_rig("rigs/nero_orca_right.yaml")
    assert rig.calibration.enabled is True
    assert QuestModel(RobotModel(rig)).calib["right"].enabled is True

    rig.calibration.enabled = False
    assert QuestModel(RobotModel(rig)).calib["right"].enabled is False
