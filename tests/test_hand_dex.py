import os
import subprocess
import sys
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import yaml
from scipy.spatial.transform import Rotation

from whatslab.core.constants import OPENXR26_TO_HAND21, OPENXR_TO_HAND21_AXES
from whatslab.core.types import HandPose, InputSample
from whatslab.kinematics import Kinematics
from whatslab.paths import models_root
from whatslab.retarget import (
    DexEngine,
    DexHandConfig,
    Engine,
    HandRetargeter,
    hand_config_names,
    mirror_xr_joints,
    xr_to_mano,
)
from whatslab.retarget.dex import mano_palm_frame, resolve_hand_config, xr_wrist_positions
from whatslab.retarget.dex.optimizer import huber_vector_loss

CONFIGS = ("orca_right", "orca_left", "dg5f_right", "dg5f_left", "robotis_right", "robotis_left",
           "allegro_right", "allegro_left", "leap_right", "leap_left", "ability_right",
           "ability_left", "schunk_right", "schunk_left")
RIGHT_MCP_X = {"index": -0.025, "middle": -0.005, "ring": 0.015, "pinky": 0.033}
SEG = (0.045, 0.026, 0.021)
FLEX = np.radians((80.0, 95.0, 60.0))


def _finger(mcp, curl):
    pts, p, phi = [np.asarray(mcp, float)], np.asarray(mcp, float), 0.0
    for l_mat, a in zip(SEG, FLEX):
        phi += curl * a
        p = p + l_mat * np.array([0.0, -np.sin(phi), -np.cos(phi)])
        pts.append(p)
    return pts


def synthetic_right_hand(curl: float) -> np.ndarray:
    xr = np.zeros((26, 8))
    xr[:, 3] = 1.0
    xr[:, 7] = 0.01
    cmc = np.array([-0.022, -0.012, -0.03])
    d = (1.0 - curl) * np.array([-0.6, -0.25, -0.75]) + curl * np.array([0.45, -0.6, -0.55])
    d /= np.linalg.norm(d)
    xr[2, 4:7] = cmc
    for k, l_mat in enumerate((0.04, 0.032, 0.026)):
        xr[3 + k, 4:7] = xr[2 + k, 4:7] + l_mat * d
    for base, f in zip((6, 11, 16, 21), ("index", "middle", "ring", "pinky")):
        x = RIGHT_MCP_X[f]
        xr[base, 4:7] = (0.5 * x, 0.0, -0.035)
        for k, p in enumerate(_finger((x, 0.0, -0.09), curl)):
            xr[base + 1 + k, 4:7] = p
    xr[0, 4:7] = 0.5 * (xr[11, 4:7] + xr[12, 4:7])
    return xr


def hand_for(side: str, curl: float) -> np.ndarray:
    xr = synthetic_right_hand(curl)
    return xr if side == "right" else mirror_xr_joints(xr)


def _rigid(xr, rot, t):
    out = xr.copy()
    out[:, 4:7] = xr[:, 4:7] @ rot.T + t
    out[:, :4] = (Rotation.from_matrix(rot) * Rotation.from_quat(xr[:, :4])).as_quat()
    return out


def _require_urdf(name):
    cfg = DexHandConfig.load(name)
    if not os.path.isfile(os.path.join(models_root(), cfg.urdf)):
        pytest.skip(f"{name}: URDF 없음 ({cfg.urdf})")
    return cfg


def test_openxr_to_mano_index_list():
    assert OPENXR26_TO_HAND21 == (1, 2, 3, 4, 5, 7, 8, 9, 10, 12, 13, 14, 15,
                              17, 18, 19, 20, 22, 23, 24, 25)
    assert np.isclose(np.linalg.det(OPENXR_TO_HAND21_AXES), 1.0)


def test_xr_to_mano_axes_on_hand_made_pose():
    xr = synthetic_right_hand(0.0)
    mano = xr_to_mano(xr)
    assert mano.shape == (21, 3)
    assert np.allclose(mano[0], 0.0)
    assert np.allclose(mano[5], [0.0, 0.025, 0.09])
    assert np.allclose(mano[17], [0.0, -0.033, 0.09])
    assert np.allclose(mano[8], OPENXR_TO_HAND21_AXES @ xr[10, 4:7])
    tips = mano[[8, 12, 16, 20]]
    assert np.all(tips[:, 2] > 0.15)
    curled = xr_to_mano(synthetic_right_hand(1.0))
    assert np.all(curled[[8, 12, 16, 20], 0] > 0.02), "굽힌 손끝이 손바닥(+x) 쪽이 아니다"


def test_xr_to_mano_ignores_wrist_pose():
    xr = synthetic_right_hand(0.4)
    a = np.radians(37.0)
    rot = np.array([[np.cos(a), 0.0, np.sin(a)], [0.0, 1.0, 0.0], [-np.sin(a), 0.0, np.cos(a)]])
    moved = _rigid(xr, rot, np.array([0.3, -0.1, 1.2]))
    assert np.allclose(xr_to_mano(moved), xr_to_mano(xr), atol=1e-9)
    assert np.allclose(xr_wrist_positions(moved), xr[:, 4:7], atol=1e-9)


def test_palm_frame_and_mirror():
    mano_r = xr_to_mano(synthetic_right_hand(0.0))
    rot = mano_palm_frame(mano_r, "right")
    assert np.isclose(np.linalg.det(rot), 1.0)
    assert rot[1, 1] > 0.99 and rot[2, 2] > 0.99, "오른손 손바닥 프레임이 MANO 축과 다르다"
    xr_l = mirror_xr_joints(synthetic_right_hand(0.0))
    mano_l = xr_to_mano(xr_l)
    assert np.allclose(mano_l, mano_r * [1.0, -1.0, 1.0])
    sv = np.diag([1.0, -1.0, 1.0])
    assert np.allclose(mano_palm_frame(mano_l, "left"), sv @ rot @ sv, atol=1e-9)
    back = mirror_xr_joints(xr_l)
    assert np.allclose(back[:, 4:7], synthetic_right_hand(0.0)[:, 4:7])


def test_hand_configs_are_shipped():
    names = hand_config_names()
    for n in CONFIGS:
        assert n in names
    assert resolve_hand_config("dex:orca", "left").endswith("orca_left.yaml")
    assert resolve_hand_config("orca_right", "left").endswith("orca_right.yaml")
    with pytest.raises(FileNotFoundError):
        resolve_hand_config("nope", "left")


def test_config_rejects_unknown_dex_key():
    with pytest.raises(ValueError, match="모르는 키"):
        DexHandConfig.from_dict({"side": "right", "urdf": "x.urdf",
                                 "palm_links": {"index": "a", "middle": "b", "pinky": "c"},
                                 "retargeting": {"wrist_link": "w"}})


def _tip_wrist_mean(eng, q):
    kin = Kinematics(eng.urdf_path)
    tf = kin.link_placements(eng.full_qpos(q))
    w = tf[kin.frame_ids([eng.config.retargeting["wrist_link_name"]])[0], :3, 3]
    tips = kin.frame_ids(eng.config.retargeting["finger_tip_link_names"][1:])
    return float(np.mean(np.linalg.norm(tf[tips, :3, 3] - w, axis=1)))


def _settle(eng, xr, n=25):
    q = None
    for _ in range(n):
        q = eng.solve(xr)
    return q


@pytest.mark.parametrize("name", CONFIGS)
def test_retargeter_smoke(name):
    cfg = _require_urdf(name)
    eng = DexEngine(name)
    lim = eng.joint_limits
    assert len(eng.joint_names) == len(lim)
    assert not set(cfg.fixed_joints) & set(eng.joint_names)
    dist = []
    for curl in (0.0, 0.5, 1.0):
        eng.reset()
        q = _settle(eng, hand_for(cfg.side, curl))
        assert q.shape == (len(eng.joint_names),)
        assert np.all(np.isfinite(q))
        assert np.all(q >= lim[:, 0] - 2e-3) and np.all(q <= lim[:, 1] + 2e-3), name
        dist.append(_tip_wrist_mean(eng, q))
    assert dist[0] > dist[1] > dist[2], f"{name}: 펴짐→주먹 굽힘이 단조가 아니다 {dist}"
    assert dist[2] < 0.85 * dist[0], f"{name}: 주먹이 충분히 닫히지 않는다 {dist}"


SEQUENCE_CURLS = np.concatenate([np.linspace(0.0, 1.0, 8), np.linspace(1.0, 0.0, 8)])


def _synthetic_sequence(side):
    rng = np.random.default_rng(7)
    return [_rigid(hand_for(side, c), Rotation.from_rotvec(rng.normal(0.0, 0.6, 3)).as_matrix(),
                   rng.normal(0.0, 0.3, 3))
            for c in SEQUENCE_CURLS]


@pytest.mark.parametrize("name", hand_config_names())
def test_every_hand_config_solves_synthetic_sequence(name):
    cfg = _require_urdf(name)
    eng = DexEngine(name)
    lim = eng.joint_limits
    assert not set(eng.kinematics.mimic_names) & set(eng.joint_names)
    for xr in _synthetic_sequence(cfg.side):
        q = eng.solve(xr)
        assert q is not None, f"{name}: {eng.last_failure}"
        assert np.all(np.isfinite(q))
        assert np.all(q >= lim[:, 0] - 2e-3) and np.all(q <= lim[:, 1] + 2e-3), name


def test_link_offsets_match_fixed_tip_links():
    cfg = _require_urdf("dg5f_s_right")
    with open(cfg.path) as f:
        d = yaml.safe_load(f)
    d["retargeting"]["finger_tip_link_names"][1] = "link_2_4"
    d["link_offsets"] = {"link_2_4": [0.018, 0.0, 0.0], "link_base": [0.0, 0.0, 0.0]}
    shifted = DexEngine(DexHandConfig.from_dict(d))
    assert shifted.tip_frames[1] == "link_2_4_offset"
    assert shifted.wrist_frame == "link_base_offset"
    plain = DexEngine("dg5f_s_right")
    for curl in (0.2, 0.9):
        a = _settle(shifted, hand_for("right", curl))
        b = _settle(plain, hand_for("right", curl))
        assert np.allclose(a, b, atol=1e-5)


def test_palm_links_accept_link_lists():
    cfg = DexHandConfig.from_dict({"side": "right", "urdf": "x.urdf", "retargeting": {},
                                   "palm_links": {"index": "a", "middle": ["a", "b"],
                                                  "pinky": "b"}})
    assert cfg.palm_links == (("a",), ("a", "b"), ("b",))


def test_chained_mimic_joints_follow_urdf():
    cfg = _require_urdf("inspire_rh56dftp_unitree_right")
    eng = DexEngine(cfg)
    assert len(eng.joint_names) == 6
    full = dict(zip(eng.robot_joint_names, eng.full_qpos(_settle(eng, hand_for("right", 0.8)))))
    mimic = [(j.get("name"), m.get("joint"), float(m.get("multiplier")),
              float(m.get("offset", 0.0)))
             for j in ET.parse(eng.urdf_path).getroot().iter("joint") for m in j.findall("mimic")]
    assert any(source in {n for n, *_ in mimic} for _, source, *_ in mimic)
    for name, source, k, offset in mimic:
        assert np.isclose(full[name], k * full[source] + offset)


def test_full_qpos_fills_mimic_joints():
    _require_urdf("ability_right")
    eng = DexEngine("ability_right")
    assert "index_q2" not in eng.joint_names
    q = _settle(eng, hand_for("right", 1.0))
    full = dict(zip(eng.robot_joint_names, eng.full_qpos(q)))
    for f in ("index", "middle", "ring", "pinky"):
        assert np.isclose(full[f"{f}_q2"], 1.05851325 * full[f"{f}_q1"] + 0.72349796)


def test_retargeter_mirrors_opposite_side_input():
    _require_urdf("orca_right")
    eng = DexEngine("orca_right", side="left")
    assert eng.mirror
    ref = DexEngine("orca_right")
    q_l = _settle(eng, hand_for("left", 0.7))
    q_r = _settle(ref, hand_for("right", 0.7))
    assert np.allclose(q_l, q_r, atol=1e-4)


def test_retargeter_uses_xr_joints():
    _require_urdf("orca_right")
    ctrl = HandRetargeter("dex:orca", side="right")
    assert ctrl.engine_name == "dex:"
    assert ctrl.engine.config.name == "orca_right"
    assert not any("Carpals" in n and "TopTower" in n for n in ctrl.joint_names)
    held = ctrl.compute(InputSample(hand=HandPose(tracked=True), tracked=True))
    lim = ctrl.engine.joint_limits
    assert np.allclose(held.joint_angles, np.clip(0.0, lim[:, 0], lim[:, 1]))
    cmd = ctrl.compute(InputSample(hand=HandPose(xr_joints=hand_for("right", 1.0),
                                                 tracked=True), tracked=True))
    assert cmd.joint_names == ctrl.joint_names
    assert np.abs(cmd.joint_angles).max() > 0.1
    lost = ctrl.compute(InputSample(hand=None, tracked=False))
    assert np.allclose(lost.joint_angles, cmd.joint_angles)


def test_retargeter_rejects_onnx_kwargs():
    _require_urdf("orca_right")
    with pytest.raises(TypeError, match="모르는 인자"):
        HandRetargeter("dex:orca", side="right", onnx_path="x.onnx")


def test_degenerate_input_holds_last_command():
    _require_urdf("orca_right")
    ctrl = HandRetargeter("dex:orca", side="right")
    good = synthetic_right_hand(0.3)
    first = ctrl.compute(InputSample(hand=HandPose(xr_joints=good, tracked=True)))
    for bad in (np.zeros((26, 8)), np.full((26, 8), np.nan)):
        bad[:, 3] = 1.0
        cmd = ctrl.compute(InputSample(hand=HandPose(xr_joints=bad, tracked=True)))
        assert np.array_equal(cmd.joint_angles, first.joint_angles)
        assert ctrl.last_failure.startswith("손바닥 프레임")
    tip = good.copy()
    tip[10, 4:7] = np.nan
    cmd = ctrl.compute(InputSample(hand=HandPose(xr_joints=tip, tracked=True)))
    assert np.array_equal(cmd.joint_angles, first.joint_angles)
    assert ctrl.last_failure == "손 키포인트에 유한하지 않은 값이 있다"


def test_solver_failure_returns_none(monkeypatch):
    _require_urdf("orca_right")
    eng = DexEngine("orca_right")
    nan = (float("nan"), np.full(len(eng.joint_names), np.nan))
    monkeypatch.setattr(eng.optimizer, "objective", lambda ref, center: lambda x: nan)
    assert eng.solve(synthetic_right_hand(0.3)) is None
    assert eng.last_failure.startswith("SLSQP")
    monkeypatch.undo()
    assert eng.solve(synthetic_right_hand(0.3)) is not None and eng.last_failure is None


def test_dex_respects_xr_valid():
    _require_urdf("orca_right")
    ctrl = HandRetargeter("dex:orca", side="right")
    good = synthetic_right_hand(0.3)
    first = ctrl.compute(InputSample(hand=HandPose(xr_joints=good, tracked=True))).joint_angles
    valid = np.ones(26, dtype=bool)
    valid[0] = False
    ctrl.reset()
    q = ctrl.compute(InputSample(hand=HandPose(xr_joints=good, xr_valid=valid, tracked=True)))
    assert ctrl.last_failure is None and np.allclose(q.joint_angles, first, atol=1e-6)
    valid[0], valid[12] = True, False
    held = ctrl.compute(
        InputSample(hand=HandPose(xr_joints=synthetic_right_hand(0.8), xr_valid=valid,
                                  tracked=True))).joint_angles
    assert np.array_equal(held, q.joint_angles)
    assert ctrl.last_failure.startswith("손바닥 프레임")
    ctrl.compute(InputSample(hand=None))
    assert ctrl.last_failure is None


def test_mirrored_dex_tolerates_zero_quaternion_on_invalid_joint():
    _require_urdf("orca_right")
    j = synthetic_right_hand(0.3).copy()
    j[0, :4] = 0.0
    valid = np.ones(26, dtype=bool)
    valid[0] = False
    ctrl = HandRetargeter("dex:orca_right", side="left")
    cmd = ctrl.compute(InputSample(hand=HandPose(xr_joints=j, xr_valid=valid, tracked=True)))
    assert np.all(np.isfinite(cmd.joint_angles))


def test_dex_engine_satisfies_engine_protocol():
    assert isinstance(DexEngine("orca_right"), Engine)


UPSTREAM_FIXTURE = os.path.join(os.path.dirname(__file__), "data", "dex_upstream_0_5_0.npz")


@pytest.mark.parametrize("name", ("orca_right", "dg5f_right", "ability_right"))
def test_dexpilot_close_to_dex_retargeting_0_5_0(name):
    _require_urdf(name)
    data = np.load(UPSTREAM_FIXTURE)
    eng = DexEngine(name)
    tip_ids = eng.kinematics.frame_ids(eng.config.retargeting["finger_tip_link_names"])

    def tips(q):
        return eng.kinematics.frames(eng.full_qpos(q), tip_ids)[0]

    q = np.array([eng.solve(x.astype(float)) for x in data["xr"]])
    tip_mm = [np.linalg.norm(tips(a) - tips(b), axis=1) * 1e3 for a, b in zip(q, data[name])]
    assert np.abs(q - data[name]).mean() < 0.03
    assert np.mean(tip_mm) < 1.5


def _fd_check(f, x, grad, eps=1e-6):
    fd = np.array([(f(x + eps * e) - f(x - eps * e)) / (2 * eps) for e in np.eye(len(x))])
    assert np.allclose(grad, fd, rtol=1e-4, atol=1e-6), np.abs(grad - fd).max()


def test_huber_vector_loss_gradient_matches_finite_difference():
    rng = np.random.default_rng(0)
    target = rng.normal(0.0, 0.05, size=(8, 3))
    weight = rng.uniform(0.1, 2.0, size=8)
    delta = np.full(8, 0.03)
    robot = target + rng.normal(0.0, 0.03, size=(8, 3))
    dist = np.linalg.norm(robot - target, axis=1)
    assert (dist < delta).any() and (dist > delta).any()
    _, grad = huber_vector_loss(robot, target, weight, delta)

    def loss(flat):
        return huber_vector_loss(flat.reshape(8, 3), target, weight, delta)[0]

    _fd_check(loss, robot.ravel(), grad.ravel())


OBJECTIVES = ("orca_right", "dg5f_right", "ability_right", "schunk_right")


@pytest.mark.parametrize("name", OBJECTIVES)
def test_objective_gradient_matches_finite_difference(name):
    _require_urdf(name)
    eng = DexEngine(name, normal_delta=0.0)
    ref = eng.reference(eng.human_keypoints(synthetic_right_hand(0.6)))
    lim = eng.joint_limits
    x = lim[:, 0] + np.random.default_rng(1).uniform(0.2, 0.8, len(lim)) * (lim[:, 1] - lim[:, 0])
    f = eng.optimizer.objective(ref, x)
    _fd_check(lambda v: f(v)[0], x, f(x)[1])


@pytest.mark.parametrize("overrides,match", (
    ({"type": "Vector"}, "DexPilot"),
    ({"type": "Position"}, "DexPilot"),
))
def test_only_dexpilot_type_is_accepted(overrides, match):
    _require_urdf("dg5f_right")
    with pytest.raises(ValueError, match=match):
        DexEngine("dg5f_right", **overrides)


@pytest.mark.parametrize("key", ("vector_weight", "position_weight", "huber_delta",
                                 "target_origin_link_names", "target_task_link_names",
                                 "target_link_human_indices"))
def test_vector_objective_keys_fail_at_load(key):
    cfg = _require_urdf("dg5f_right")
    with pytest.raises(TypeError, match=key):
        DexEngine("dg5f_right", **{key: 0.5})
    d = {"name": "x", "side": "right", "urdf": cfg.urdf, "palm_links": dict(
        zip(("index", "middle", "pinky"), cfg.palm_links)),
        "retargeting": {**cfg.retargeting, key: 0.5}}
    with pytest.raises(ValueError, match=key):
        DexHandConfig.from_dict(d)


def test_sdk_import_does_not_import_torch():
    _require_urdf("orca_right")
    code = (
        "import sys\n"
        "import whatslab, whatslab.hand, whatslab.retarget\n"
        "from whatslab.retarget import DexEngine\n"
        "DexEngine('orca_right')\n"
        "heavy = ('torch', 'dex_retargeting', 'yourdfpy', 'trimesh')\n"
        "bad = [m for m in heavy if m in sys.modules]\n"
        "assert not bad, bad\n"
    )
    proc = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True,
                          timeout=120)
    assert proc.returncode == 0, proc.stderr
