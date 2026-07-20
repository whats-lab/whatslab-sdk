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

