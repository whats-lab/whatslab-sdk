"""robot — rig 로더/검증, RobotModel 정준 샌드위치, 부분 조립, 파이프라인."""
import numpy as np
import pytest

from whatslab.robot import RobotModel, RobotSpec, configs_root, load_rig


def test_configs_root_and_load_rig():
    root = configs_root()
    assert root, "레포 configs/ 미발견 (editable 설치 전제)"
    rig = load_rig("rigs/nero_orca_right.yaml")
    assert rig.arm is not None and rig.arm.kind == "arm"
    assert rig.hand is not None and rig.hand.kind == "hand"
    assert isinstance(rig.lock_joints, list)     # 파싱 확인 (값은 config 편집 대상)
    # target_ee 생략 → hand.base_frame fallback
    assert rig.resolve_target_ee() == "R-Carpals_8d1f1041"


def test_robot_spec_validation():
    with pytest.raises(ValueError):        # arm 인데 ee.parent 없음
        RobotSpec.from_dict({"name": "x", "kind": "arm", "urdf": "u"})
    with pytest.raises(ValueError):        # hand 인데 base_frame 없음
        RobotSpec.from_dict({"name": "x", "kind": "hand", "urdf": "u"})
    with pytest.raises(ValueError):        # 잘못된 kind
        RobotSpec.from_dict({"name": "x", "kind": "leg", "urdf": "u"})


def test_target_ee_fallback_chain():
    rig = load_rig("rigs/nero_orca_right.yaml")
    rig.target_ee = "custom_frame"
    assert rig.resolve_target_ee() == "custom_frame"
    rig.target_ee = None
    rig.hand = None
    assert rig.resolve_target_ee() == "ee"     # arm TCP


def test_model_canonical_sandwich():
    """mount(yaw 180°) 샌드위치: ee_pose = M @ fk, solve(정준)→q 왕복."""
    pytest.importorskip("pinocchio")
    rig = load_rig("rigs/nero_orca_right.yaml")
    rig.calibration.enabled = False        # 매핑 없이 순수 샌드위치 검증
    m = RobotModel(rig)
    assert m.has_arm and m.has_hand
    assert len(m.arm_joint_names) == 7     # joint1..7 (잠금 없음)

    q0 = np.full(len(m.arm_joint_names), 0.3)
    T_c = m.ee_pose(q0)                    # 정준 (lazy out-leg)
    T_b = m.solver.fk(q0)                  # 베이스
    assert np.allclose(T_c, m.to_canonical(T_b))
    assert np.allclose(m.to_base(T_c), T_b, atol=1e-12)
    # mount yaw π → 정준 x 와 베이스 x 는 부호 반대
    assert np.sign(T_c[0, 3]) == -np.sign(T_b[0, 3]) or abs(T_b[0, 3]) < 1e-9

    # 정준 목표로 solve → 같은 pose 로 수렴 (diff 백엔드, warm)
    m.sync_state(q0 + 0.1)
    for _ in range(80):
        q = m.solve(T_c)
    err = np.linalg.norm(m.ee_pose(q)[:3, 3] - T_c[:3, 3])
    assert err < 5e-3, f"sandwich solve 오차 {err*1e3:.2f}mm"


def test_model_arm_only_and_hand_only():
    pytest.importorskip("pinocchio")
    rig = load_rig("rigs/nero_orca_right.yaml")
    # arm 단독
    rig_a = load_rig("rigs/nero_orca_right.yaml")
    rig_a.hand = None
    rig_a.target_ee = None
    # ee.parent(joint7) 를 잠그면 arm 단독은 에러여야 (TCP 부모 잠금 금지)
    rig_a.lock_joints = ["joint7"]
    with pytest.raises(ValueError):
        RobotModel(rig_a)
    rig_a.lock_joints = []
    m = RobotModel(rig_a)
    assert m.has_arm and not m.has_hand
    assert len(m.arm_joint_names) == 7           # joint1..7 (잠금 없음)
    q0 = np.zeros(len(m.arm_joint_names))
    assert m.ee_pose(q0).shape == (4, 4)
    # hand 단독
    rig_h = load_rig("rigs/nero_orca_right.yaml")
    rig_h.arm = None
    m2 = RobotModel(rig_h)
    assert m2.has_hand and not m2.has_arm and m2.solver is None


def test_model_reach_clamp():
    """reach_max 가 먼 목표를 구 반경으로 클램프 (workspace 매핑 폐기 후)."""
    pytest.importorskip("pinocchio")
    rig = load_rig("rigs/nero_orca_right.yaml")
    rig.calibration.enabled = False
    rig.solver.reach_max = 0.7            # 도달 가능 반경 안 (min<0.7<max)
    m = RobotModel(rig)
    # 정준 원점에서 아주 먼 목표(반경 3m) → 베이스 목표가 reach_max 로 잘려야
    T = np.eye(4); T[:3, 3] = [3.0, 0.0, 0.0]
    m.sync_state(np.zeros(len(m.arm_joint_names)))
    for _ in range(150):
        q = m.solve(T)
    r = float(np.linalg.norm(m.solver.fk(q)[:3, 3]))
    assert r <= 0.7 + 1e-2, f"reach 클램프 실패: 반경 {r:.3f} > 0.7 (먼 목표가 안 잘림)"


def test_model_uniform_reach_scale():
    """calibration.input_reach → 정준 위치를 s=reach_max/input_reach 등방 스케일."""
    pytest.importorskip("pinocchio")
    rig = load_rig("rigs/nero_arm.yaml")
    rig.solver.reach_max = 0.9
    m = RobotModel(rig)
    n = len(m.arm_joint_names)
    T = np.eye(4); T[:3, 3] = [0.2, 0.0, 0.1]

    rig.calibration.enabled = True         # s = 0.9/0.45 = 2.0 → 위치 2배
    rig.calibration.input_reach = 0.45
    m.sync_state(np.zeros(n)); q_cal = m.solve(T)

    rig.calibration.enabled = False        # 수동 2배 위치 → 동일 목표 → 동일 q
    T2 = np.eye(4); T2[:3, 3] = [0.4, 0.0, 0.2]
    m.sync_state(np.zeros(n)); q_manual = m.solve(T2)
    assert np.allclose(q_cal, q_manual, atol=1e-6)
