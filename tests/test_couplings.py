import dataclasses

import numpy as np
import pytest
from test_hand_dex import _fd_check, _require_urdf, synthetic_right_hand

from whatslab.kinematics import Kinematics, parse_couplings
from whatslab.retarget.dex import DexEngine, DexHandConfig

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
))
def test_coupling_schema_errors(raw, match):
    with pytest.raises(ValueError, match=match):
        parse_couplings(raw)


@pytest.mark.parametrize("mimic2,raw,match", (
    ("", {"j9": {"source": "j1"}}, "없는"),
    ("", {"j2": {"source": "j9"}}, "없는"),
    (MIMIC2, {"j2": {"source": "j3"}}, "URDF mimic"),
    (MIMIC2, {"j1": {"source": "j2"}}, "순환"),
))
def test_coupling_urdf_errors(mimic2, raw, match):
    with pytest.raises(ValueError, match=match):
        _chain(mimic2).couple(parse_couplings(raw))


def _igris_coupled():
    cfg = _require_urdf("igris_c_right")
    couplings = {name: (name.replace("distal", "middle").replace(f"_{n}_", f"_{n - 1}_"), 1.0, 0.0)
                 for name in cfg.fixed_joints for n in [int(name.split("_")[1])]}
    return cfg, dataclasses.replace(cfg, fixed_joints={}, couplings=couplings, posture=None)


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
    cfg, coupled = _igris_coupled()
    both = dataclasses.replace(coupled, fixed_joints=dict(cfg.fixed_joints))
    with pytest.raises(ValueError, match="fixed_joints"):
        DexEngine(both)


@pytest.mark.parametrize("side", ("right", "left"))
def test_igris_c_distal_joints_are_passive_at_rest(side):
    cfg = _require_urdf(f"igris_c_{side}")
    eng = DexEngine(cfg)
    assert len(eng.joint_names) == 6 and len(cfg.fixed_joints) == 5
    assert all("distal" in n for n in cfg.fixed_joints)
    full = dict(zip(eng.robot_joint_names, eng.full_qpos(eng.joint_limits.mean(1))))
    assert all(full[n] == 0.0 for n in cfg.fixed_joints)


def test_coupling_keys_load_from_dict():
    cfg = _require_urdf("igris_c_right")
    d = {"name": "x", "side": "right", "urdf": cfg.urdf,
         "palm_links": dict(zip(("index", "middle", "pinky"), cfg.palm_links)),
         "retargeting": cfg.retargeting, "couplings": {"a": {"source": "a"}}}
    with pytest.raises(ValueError, match="자기 자신"):
        DexHandConfig.from_dict(d)
