import dataclasses
import xml.etree.ElementTree as ET

import numpy as np
import pytest
from test_hand_dex import _fd_check, _require_urdf, synthetic_right_hand

from whatslab.kinematics import Kinematics, coupling_overrides, parse_couplings
from whatslab.paths import models_root
from whatslab.retarget.dex import DexEngine, DexHandConfig
from whatslab.retarget.geort.train.hand import (
    HandKinematics,
    coupled,
    read_hand,
    read_joints,
    urdf_path,
    urdf_tips,
)

CHAIN = """<robot name="c">
  <link name="base"/><link name="a"/><link name="b"/><link name="c"/><link name="tip"/>
  <joint name="j1" type="revolute"><parent link="base"/><child link="a"/>
    <origin xyz="0 0 0.05"/><axis xyz="1 0 0"/><limit lower="0" upper="1.5"/></joint>
  <joint name="j2" type="revolute"><parent link="a"/><child link="b"/>
    <origin xyz="0 0 0.04"/><axis xyz="1 0 0"/><limit lower="0" upper="1.5"/>{mimic2}</joint>
  <joint name="j3" type="revolute"><parent link="b"/><child link="c"/>
    <origin xyz="0 0 0.03"/><axis xyz="0 1 0"/><limit lower="0" upper="1.5"/>{mimic3}</joint>
  <joint name="t" type="fixed"><parent link="c"/><child link="tip"/>
    <origin xyz="0 0 0.02"/></joint>
</robot>"""
MIMIC2 = '<mimic joint="j1" multiplier="0.7" offset="0.1"/>'
MIMIC3 = '<mimic joint="j2" multiplier="-2.0" offset="0.05"/>'


def _chain(mimic2="", mimic3=""):
    return Kinematics.from_string(CHAIN.format(mimic2=mimic2, mimic3=mimic3))


def test_couplings_match_urdf_mimic():
    urdf = _chain(MIMIC2, MIMIC3)
    coupled = _chain()
    coupled.couple(parse_couplings({
        "j2": {"source": "j1", "multiplier": 0.7, "offset": 0.1},
        "j3": {"source": "j2", "multiplier": -2.0, "offset": 0.05},
    }))
    assert coupled.mimic_names == urdf.mimic_names
    assert coupled.actuated_joint_names == urdf.actuated_joint_names == ["j1"]
    assert np.allclose(coupled.mimic_multipliers, urdf.mimic_multipliers)
    assert np.allclose(coupled.mimic_offsets, urdf.mimic_offsets)
    ids = urdf.frame_ids(urdf.link_names)
    for v in (0.0, 0.4, 1.2):
        q = np.array([v, 0.0, 0.0])
        a = urdf.link_placements(urdf.forward_qpos(q.copy()))[ids]
        b = coupled.link_placements(coupled.forward_qpos(q.copy()))[ids]
        assert np.array_equal(a, b)


def test_couplings_extend_urdf_mimic():
    kin = _chain(MIMIC2)
    kin.couple(parse_couplings({"j3": {"source": "j2"}}))
    q = kin.forward_qpos(np.array([0.5, 0.0, 0.0]))
    assert np.isclose(q[1], 0.7 * 0.5 + 0.1) and np.isclose(q[2], q[1])


@pytest.mark.parametrize("raw,match", (
    ([1], "종속 관절"),
    ({"j2": "j1"}, "키-값"),
    ({"j2": {"source": "j1", "gain": 1.0}}, "모르는 키"),
    ({"j2": {"multiplier": 1.0}}, "source"),
    ({"j2": {"source": "j2"}}, "자기 자신"),
    ({"j2": {"source": "j3"}, "j3": {"source": "j2"}}, "순환"),
    ({"j2": {"source": "j1", "override_urdf": "yes"}}, "override_urdf"),
))
def test_coupling_schema_errors(raw, match):
    with pytest.raises(ValueError, match=match):
        parse_couplings(raw)


@pytest.mark.parametrize("mimic2,raw,match", (
    ("", {"j9": {"source": "j1"}}, "없는"),
    ("", {"j2": {"source": "j9"}}, "없는"),
    (MIMIC2, {"j2": {"source": "j3"}}, "URDF mimic"),
    (MIMIC2, {"j1": {"source": "j2"}}, "순환"),
    ("", {"j2": {"source": "j1", "override_urdf": True}}, "URDF mimic 관절이 아니다"),
))
def test_coupling_urdf_errors(mimic2, raw, match):
    with pytest.raises(ValueError, match=match):
        _chain(mimic2).couple(parse_couplings(raw), coupling_overrides(raw))


OVERRIDE = {"j2": {"source": "j1", "multiplier": -1.3, "offset": 0.2, "override_urdf": True}}
PATCHED2 = '<mimic joint="j1" multiplier="-1.3" offset="0.2"/>'


def test_override_urdf_coupling_matches_patched_urdf():
    patched = _chain(PATCHED2, MIMIC3)
    kin = _chain(MIMIC2, MIMIC3)
    kin.couple(parse_couplings(OVERRIDE), coupling_overrides(OVERRIDE))
    assert kin.mimic_names == patched.mimic_names
    assert np.allclose(kin.mimic_multipliers, patched.mimic_multipliers)
    assert np.allclose(kin.mimic_offsets, patched.mimic_offsets)
    ids = kin.frame_ids(kin.link_names)
    for v in (0.0, 0.4, 1.2):
        q = np.array([v, 0.0, 0.0])
        a = patched.link_placements(patched.forward_qpos(q.copy()))[ids]
        b = kin.link_placements(kin.forward_qpos(q.copy()))[ids]
        assert np.array_equal(a, b)


def _chain_joints(mimic2):
    return read_joints(ET.fromstring(CHAIN.format(mimic2=mimic2, mimic3=MIMIC3)))


def test_geort_train_coupled_honours_override():
    joints = _chain_joints(MIMIC2)
    with pytest.raises(ValueError, match="URDF mimic"):
        coupled(joints, parse_couplings(OVERRIDE))
    out = coupled(joints, parse_couplings(OVERRIDE), coupling_overrides(OVERRIDE))
    assert [j.mimic for j in out] == [j.mimic for j in _chain_joints(PATCHED2)]


def test_schunk_left_override_matches_patched_urdf_and_mirrors_right():
    left, right = _require_urdf("schunk_left"), _require_urdf("schunk_right")
    path = f"{models_root()}/{left.urdf}"
    text = open(path).read()
    bug = '<mimic joint="left_hand_Thumb_Opposition" multiplier="1" offset="0"/>'
    assert text.count(bug) == 1
    patched = Kinematics.from_string(text.replace(bug, bug.replace('"1"', '"-1"')))
    eng = DexEngine(left, posture_prior=False)
    kin = eng.kinematics
    ids = kin.frame_ids(patched.link_names)
    pids = patched.frame_ids(patched.link_names)
    lim = eng.joint_limits
    for t in (0.0, 0.3, 0.8):
        q = lim[:, 0] + t * (lim[:, 1] - lim[:, 0])
        full = eng.full_qpos(q)
        mine = kin.link_placements(full)[ids]
        ref = patched.link_placements(patched.forward_qpos(full.copy()))[pids]
        assert np.abs(mine - ref).max() < 1e-12
    reng = DexEngine(right, posture_prior=False)
    flip = np.array([1.0, -1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, -1.0])
    for qd in ([30, 0, 0, 0, 0, 0, 0, 0, 0], [50, 30, 20, 10, 20, 10, 30, 40, 15]):
        qr = np.radians(qd)
        tips = []
        for e, q in ((reng, qr), (eng, qr * flip)):
            ids = e.kinematics.frame_ids([e.wrist_frame, *e.tip_frames])
            pos = e.kinematics.frames(e.full_qpos(q), ids)[0]
            tips.append((pos[1:] - pos[0]) @ e.robot_palm.T)
        assert np.abs(tips[0] * [1.0, -1.0, 1.0] - tips[1]).max() < 1e-4


def _igris_coupled():
    cfg = _require_urdf("igris_c_right")
    return cfg, dataclasses.replace(cfg, posture=None)


def test_engine_drops_coupled_joints_and_fills_them_in_full_qpos():
    cfg, coupled = _igris_coupled()
    eng = DexEngine(coupled)
    assert len(eng.joint_names) == 6
    assert not set(coupled.couplings) & set(eng.joint_names)
    q = eng.solve(synthetic_right_hand(0.7))
    full = dict(zip(eng.robot_joint_names, eng.full_qpos(q)))
    for name, (src, _, _) in coupled.couplings.items():
        assert full[name] == full[src]


def test_objective_gradient_with_couplings():
    _, coupled = _igris_coupled()
    eng = DexEngine(coupled, normal_delta=0.0)
    ref = eng.reference(eng.human_keypoints(synthetic_right_hand(0.6)))
    lim = eng.joint_limits
    x = lim[:, 0] + np.random.default_rng(3).uniform(0.2, 0.8, len(lim)) * (lim[:, 1] - lim[:, 0])
    f = eng.optimizer.objective(ref, x)
    _fd_check(lambda v: f(v)[0], x, f(x)[1])


def test_coupled_joint_cannot_be_fixed():
    _, coupled = _igris_coupled()
    both = dataclasses.replace(coupled, fixed_joints={n: 0.0 for n in coupled.couplings})
    with pytest.raises(ValueError, match="fixed_joints"):
        DexEngine(both)


@pytest.mark.parametrize("side", ("right", "left"))
def test_igris_c_distal_follows_middle(side):
    cfg = _require_urdf(f"igris_c_{side}")
    eng = DexEngine(cfg)
    assert len(eng.joint_names) == 6 and len(cfg.couplings) == 5
    assert all("distal" in n and "middle" in src for n, (src, _, _) in cfg.couplings.items())
    full = dict(zip(eng.robot_joint_names, eng.full_qpos(eng.joint_limits.mean(1))))
    assert all(full[n] == full[src] for n, (src, _, _) in cfg.couplings.items())


def test_geort_train_couplings_follow_source():
    _require_urdf("igris_c_right")
    cfg = read_hand("igris_c_right")
    kin = HandKinematics(cfg, urdf_path(cfg))
    assert len(kin.joint_order) == 6
    q = (kin.lower + kin.upper) / 2
    assert np.abs(kin.tips(q[None])[0] - urdf_tips(kin, q)).max() < 1e-12
    bad = {**cfg, "fingers": [{**f, "joints": [*f["joints"], *(
        ["r_4_joint_index_distal"] if f["name"] == "index" else [])]} for f in cfg["fingers"]]}
    with pytest.raises(ValueError, match="종속 관절"):
        HandKinematics(bad, urdf_path(cfg))


def test_coupling_keys_load_from_dict():
    cfg = _require_urdf("igris_c_right")
    d = {"name": "x", "side": "right", "urdf": cfg.urdf,
         "palm_links": dict(zip(("index", "middle", "pinky"), cfg.palm_links)),
         "retargeting": cfg.retargeting, "couplings": {"a": {"source": "a"}}}
    with pytest.raises(ValueError, match="자기 자신"):
        DexHandConfig.from_dict(d)
