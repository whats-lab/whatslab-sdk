import os

import numpy as np
import pytest
import yaml
from test_hand_dex import _fd_check, _require_urdf, hand_for, synthetic_right_hand

from whatslab.retarget.dex import (
    DexEngine,
    DexHandConfig,
    hand_config_names,
    mirror_xr_joints,
    resolve_hand_config,
)
from whatslab.retarget.dex.posture import FEATURES, human_angles, parse_posture, spread_joints

DATA = os.path.join(os.path.dirname(__file__), "data")
RP_CONFIG = os.path.join(DATA, "dg5f_s_robotplatform.yaml")
RP_FIXTURE = os.path.join(DATA, "posture_robotplatform_dg5f_s.npz")
WARMUP = 10


def _rp_engine(**kw):
    with open(RP_CONFIG) as f:
        cfg = DexHandConfig.from_dict(yaml.safe_load(f))
    return DexEngine(cfg, urdf_root=DATA, **kw)


def test_prior_target_matches_robotplatform_dg5f_s():
    data = np.load(RP_FIXTURE)
    eng = _rp_engine()
    for xr, target, weight in zip(data["xr"], data["target"], data["weight"]):
        center, w = eng.posture_target(xr)
        assert np.allclose(center, target, atol=1e-6)
        assert np.array_equal(w, weight)


def test_solve_matches_robotplatform_dg5f_s():
    data = np.load(RP_FIXTURE)
    eng = _rp_engine()
    for _ in range(WARMUP - 1):
        eng.solve(data["xr"][0])
    eng._smoothed = None
    q = np.array([eng.solve(xr) for xr in data["xr"]])
    assert np.degrees(np.abs(q - data["q"])).mean() < 0.5


@pytest.mark.parametrize("name", ("dg5f_s_right", "orca_right", "allegro_right", "shadow_left"))
def test_objective_gradient_with_prior(name):
    _require_urdf(name)
    eng = DexEngine(name, normal_delta=0.0)
    xr = hand_for(eng.config.side, 0.6)
    prior = eng.posture_target(xr)
    assert prior is not None and np.any(prior[1] > 0.0)
    ref = eng.reference(eng.human_keypoints(xr))
    lim = eng.joint_limits
    x = lim[:, 0] + np.random.default_rng(2).uniform(0.2, 0.8, len(lim)) * (lim[:, 1] - lim[:, 0])
    f = eng.optimizer.objective(ref, x, prior)
    _fd_check(lambda v: f(v)[0], x, f(x)[1])
    plain = eng.optimizer.objective(ref, x)
    d = x - prior[0]
    assert np.isclose(f(x)[0] - plain(x)[0], np.sum(prior[1] * d * d))


def test_prior_pulls_toward_target():
    _require_urdf("dg5f_s_right")
    xr = synthetic_right_hand(0.3)
    on, off = DexEngine("dg5f_s_right"), DexEngine("dg5f_s_right", posture_prior=False)
    assert off.posture is None and off.posture_target(xr) is None
    center, _ = on.posture_target(xr)
    for _ in range(20):
        q_on, q_off = on.solve(xr), off.solve(xr)
    assert np.abs(q_on - center).mean() < np.abs(q_off - center).mean()


def test_set_posture_target_overrides_and_clears():
    _require_urdf("dg5f_s_right")
    eng = DexEngine("dg5f_s_right")
    xr = synthetic_right_hand(0.5)
    auto = eng.posture_target(xr)
    center = eng.joint_limits.mean(1)
    eng.set_posture_target(center, 5.0)
    got = eng.posture_target(xr)
    assert np.array_equal(got[0], center) and np.all(got[1] == 5.0)
    for _ in range(20):
        q = eng.solve(xr)
    assert np.abs(q - center).mean() < np.radians(2.0)
    eng.set_posture_target(center)
    assert np.array_equal(eng.posture_target(xr)[1], eng.posture.weight)
    eng.set_posture_target(None)
    again = eng.posture_target(xr)
    assert np.array_equal(again[0], auto[0]) and np.array_equal(again[1], auto[1])


def test_set_posture_target_without_config_prior():
    _require_urdf("dg5f_s_right")
    eng = DexEngine("dg5f_s_right", posture_prior=False)
    n = len(eng.joint_names)
    with pytest.raises(ValueError, match="weight"):
        eng.set_posture_target(np.zeros(n))
    with pytest.raises(ValueError, match="shape"):
        eng.set_posture_target(np.zeros(n + 1), 1.0)
    with pytest.raises(ValueError, match="유한"):
        eng.set_posture_target(np.zeros(n), -1.0)
    eng.set_posture_target(np.zeros(n), np.full(n, 0.1))
    assert eng.solve(synthetic_right_hand(0.5)) is not None


def test_left_hand_spread_is_anatomical():
    xr = synthetic_right_hand(0.4)
    right = human_angles(xr, "right")
    left = human_angles(mirror_xr_joints(xr), "left")
    assert np.allclose(right, left, atol=1e-9)


MIRROR_PAIRS = (("dg5f_s_right", "dg5f_s_left"), ("orca_right", "orca_left"),
                ("shadow_right", "shadow_left"), ("robotis_right", "robotis_left"))


@pytest.mark.parametrize("pair", MIRROR_PAIRS)
def test_left_config_mirrors_right(pair):
    for n in pair:
        _require_urdf(n)
    right, left = (DexEngine(n) for n in pair)
    for xr in np.load(RP_FIXTURE)["xr"][::10]:
        a = right.posture_target(xr)[0]
        b = left.posture_target(mirror_xr_joints(xr))[0]
        assert np.allclose(np.abs(a), np.abs(b), atol=np.radians(4.0)), np.degrees(a - b)


def _spec(**changes):
    base = {"weight": 0.01, "joints": {"j1": {"index.mcp": 1.0}}}
    return {**base, **changes}


@pytest.mark.parametrize("raw,match", (
    (_spec(weight=0.0), "weight"),
    (_spec(joints={}), "joints"),
    (_spec(joints={"j1": {"index.bend": 1.0}}), "모르는 사람 손 각도"),
    (_spec(joints={"j1": {}}), "비었다"),
    (_spec(extra=1), "모르는 키"),
    (_spec(joint_weights={"j2": 0.1}), "joint_weights"),
    (_spec(joint_weights={"j1": 0.0}), "joint_weights"),
    (_spec(thumb_base={"joints": ["j1", "j3"], "bone": ["a", "b"], "flex_joint": "c"}),
     "thumb_base"),
    (_spec(thumb_base={"joints": ["j3"], "bone": ["a", "b"], "flex_joint": "c"}), "두 개"),
    (_spec(open_hand={"open_below_deg": 30, "free_above_deg": 10, "weight": 1}), "free_above"),
    (_spec(open_hand={"open_below_deg": 1, "free_above_deg": 10, "weight": 1,
                      "fingers": ["toe"]}), "손가락"),
    (_spec(abduction={"full": 60}), "모르는 키"),
    (_spec(abduction={"full_deg": 0}), "full_deg"),
    (_spec(abduction={"gain": -1}), "gain"),
    (_spec(abduction={"decay": -1}), "decay"),
))
def test_posture_schema_errors_at_load(raw, match):
    with pytest.raises(ValueError, match=match):
        parse_posture(raw)


def _abduction_engines(name):
    _require_urdf(name)
    with open(resolve_hand_config(name)) as f:
        raw = yaml.safe_load(f)
    raw["posture_prior"].pop("abduction", None)
    off = DexEngine(DexHandConfig.from_dict(raw))
    raw["posture_prior"]["abduction"] = {"full_deg": 60.0, "decay": 1.0, "gain": 10.0}
    return off, DexEngine(DexHandConfig.from_dict(raw))


def test_abduction_rule_shrinks_target_and_raises_weight_with_flexion():
    off, on = _abduction_engines("dg5f_s_15dof_right")
    rows = [on.joint_names.index(j) for j in spread_joints(on.config.posture.joints)]
    assert rows
    xr = hand_for("right", 0.0)
    f0 = on.posture.abduction.flex(human_angles(xr))
    a, b = off.posture_target(xr), on.posture_target(xr)
    rest = 1.0 + 10.0 * f0
    assert np.allclose(b[1][rows], a[1][rows] * rest)
    xr = hand_for("right", 1.0)
    f1 = on.posture.abduction.flex(human_angles(xr))
    assert np.all(f1 > f0)
    a, b = off.posture_target(xr), on.posture_target(xr)
    assert np.allclose(b[1][rows], a[1][rows] * (1.0 + 10.0 * f1))
    assert np.all(np.abs(b[0][rows]) <= np.abs(a[0][rows]) + 1e-12)
    other = [i for i in range(len(on.joint_names)) if i not in rows]
    assert np.allclose(a[0][other], b[0][other]) and np.array_equal(a[1][other], b[1][other])


def test_posture_joint_names_checked_at_engine_build():
    cfg = _require_urdf("dg5f_s_right")
    d = {"name": "x", "side": "right", "urdf": cfg.urdf,
         "palm_links": dict(zip(("index", "middle", "pinky"), cfg.palm_links)),
         "retargeting": cfg.retargeting,
         "posture_prior": {"weight": 0.01, "joints": {"no_such_joint": {"index.mcp": 1.0}}}}
    with pytest.raises(ValueError, match="no_such_joint"):
        DexEngine(DexHandConfig.from_dict(d))


def test_positive_suffix_uses_positive_part():
    spec = parse_posture({"weight": 0.01, "joints": {"j": {"index.mcp+": 2.0}}})
    assert spec.joints == {"j": {"index.mcp+": 2.0}}
    assert len(FEATURES) == 18


@pytest.mark.parametrize("name", hand_config_names())
def test_every_bundled_config_has_a_usable_prior(name):
    _require_urdf(name)
    eng = DexEngine(name)
    assert eng.posture is not None
    xr = hand_for(eng.config.side, 0.5)
    center, weight = eng.posture_target(xr)
    lim = eng.joint_limits
    assert np.all(center >= lim[:, 0]) and np.all(center <= lim[:, 1])
    assert eng.solve(xr) is not None
