import glob
import os

import numpy as np
import pytest

from whatslab.kinematics import Kinematics
from whatslab.paths import models_root

pin = pytest.importorskip("pinocchio")

TOL = 1e-9


def _urdfs():
    root = models_root()
    return sorted(os.path.relpath(p, root)
                  for p in glob.glob(os.path.join(root, "**", "*.urdf"), recursive=True))


def _random_q(kin, rng):
    lo, hi = kin.joint_limits.T
    lo, hi = np.where(np.isfinite(lo), lo, -np.pi), np.where(np.isfinite(hi), hi, np.pi)
    return rng.uniform(lo, hi)


@pytest.mark.parametrize("rel", _urdfs())
def test_matches_pinocchio(rel):
    path = os.path.join(models_root(), rel)
    model = pin.buildModelFromUrdf(path)
    data = model.createData()
    kin = Kinematics(path)
    assert kin.joint_names == list(model.names)[1:]
    assert np.array_equal(kin.joint_limits[:, 0], model.lowerPositionLimit)
    assert np.array_equal(kin.joint_limits[:, 1], model.upperPositionLimit)
    ids = kin.frame_ids(kin.link_names)
    pin_ids = [model.getFrameId(n, pin.BODY) for n in kin.link_names]
    rng = np.random.default_rng(0)
    for _ in range(5):
        q = _random_q(kin, rng)
        pin.computeJointJacobians(model, data, q)
        pin.updateFramePlacements(model, data)
        want_tf = np.array([data.oMf[f].homogeneous for f in pin_ids])
        want_jac = np.array([pin.getFrameJacobian(model, data, f, pin.LOCAL_WORLD_ALIGNED)
                             for f in pin_ids])
        tf, jac = kin.jacobians(q, ids)
        np.testing.assert_allclose(kin.link_placements(q), want_tf, rtol=0, atol=TOL)
        np.testing.assert_allclose(tf, want_tf, rtol=0, atol=TOL)
        np.testing.assert_allclose(jac, want_jac, rtol=0, atol=TOL)
        pos, jac3 = kin.frames(q, ids)
        np.testing.assert_allclose(pos, want_tf[:, :3, 3], rtol=0, atol=TOL)
        np.testing.assert_allclose(jac3, want_jac[:, :3], rtol=0, atol=TOL)


URDF = """<robot name="t">
  <link name="base"/><link name="a"/><link name="b"/><link name="c"/><link name="d"/>
  <link name="e"/>
  <joint name="z" type="revolute"><parent link="base"/><child link="a"/>
    <origin xyz="0.1 0 0" rpy="0.3 -0.2 1.1"/><axis xyz="0 1 1"/>
    <limit lower="-1" upper="1" effort="1" velocity="1"/></joint>
  <joint name="y" type="fixed"><parent link="base"/><child link="b"/>
    <origin xyz="0 0.2 0" rpy="0 0 0.5"/></joint>
  <joint name="x" type="prismatic"><parent link="b"/><child link="c"/>
    <axis xyz="1 0 0"/><limit lower="0" upper="0.1" effort="1" velocity="1"/>
    <mimic joint="z" multiplier="0.5" offset="0.01"/></joint>
  <joint name="w" type="revolute"><parent link="c"/><child link="d"/>
    <origin xyz="0 0 0.05"/><axis xyz="0 0 -1"/>
    <limit lower="-2" upper="2" effort="1" velocity="1"/>
    <mimic joint="x" multiplier="2" offset="0.3"/></joint>
  <joint name="v" type="continuous"><parent link="d"/><child link="e"/><axis xyz="1 0 0"/></joint>
</robot>"""


def test_tree_order_and_chained_mimic():
    kin = Kinematics.from_string(URDF)
    assert kin.joint_names == ["x", "w", "v", "z"]
    assert kin.actuated_joint_names == ["v", "z"]
    assert kin.mimic_source_names == ["z", "z"]
    np.testing.assert_allclose(kin.mimic_multipliers, [0.5, 1.0])
    np.testing.assert_allclose(kin.mimic_offsets, [0.01, 0.32])
    q = kin.forward_qpos(np.array([0.0, 0.0, 0.4, 0.6]))
    np.testing.assert_allclose(q, [0.31, 0.92, 0.4, 0.6])
    assert np.isinf(kin.joint_limits[2]).all()


def test_continuous_matches_pinocchio():
    model = pin.buildModelFromXML(URDF)
    data = model.createData()
    kin = Kinematics.from_string(URDF)
    q = np.array([0.05, 0.7, 2.5, -0.4])
    pq = np.array([0.05, 0.7, np.cos(2.5), np.sin(2.5), -0.4])
    pin.computeJointJacobians(model, data, pq)
    pin.updateFramePlacements(model, data)
    ids = kin.frame_ids(kin.link_names)
    for f, tf, jac in zip(ids, *kin.jacobians(q, ids)):
        fid = model.getFrameId(kin.link_names[f], pin.BODY)
        np.testing.assert_allclose(tf, data.oMf[fid].homogeneous, atol=TOL)
        want = pin.getFrameJacobian(model, data, fid, pin.LOCAL_WORLD_ALIGNED)
        np.testing.assert_allclose(jac, want, atol=TOL)


def test_unknown_link_rejected():
    with pytest.raises(ValueError, match="없는 링크"):
        Kinematics.from_string(URDF).frame_ids(["nope"])


def test_added_frame_matches_pinocchio():
    model = pin.buildModelFromXML(URDF)
    kin = Kinematics.from_string(URDF)
    parent = model.getFrameId("d", pin.BODY)
    frame = model.frames[parent]
    offset = np.array([0.01, -0.02, 0.03])
    model.addFrame(pin.Frame("tip", frame.parentJoint, parent,
                             frame.placement * pin.SE3(np.eye(3), offset), pin.BODY))
    data = model.createData()
    fid = kin.add_frame("tip", "d", offset)
    q = np.array([0.05, 0.7, 2.5, -0.4])
    pin.computeJointJacobians(model, data, np.array([0.05, 0.7, np.cos(2.5), np.sin(2.5), -0.4]))
    pin.updateFramePlacements(model, data)
    tf, jac = kin.jacobians(q, [fid])
    want = model.getFrameId("tip", pin.BODY)
    np.testing.assert_allclose(tf[0], data.oMf[want].homogeneous, atol=TOL)
    np.testing.assert_allclose(
        jac[0], pin.getFrameJacobian(model, data, want, pin.LOCAL_WORLD_ALIGNED), atol=TOL)
    with pytest.raises(ValueError, match="이미 있는"):
        kin.add_frame("tip", "d", offset)
