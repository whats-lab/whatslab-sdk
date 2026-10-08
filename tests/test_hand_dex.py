import os

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

pytest.importorskip("dex_retargeting")
pin = pytest.importorskip("pinocchio")

from whatslab.core.types import HandPose, InputSample  # noqa: E402
from whatslab.paths import models_root  # noqa: E402
from whatslab.robot import RobotModel, load_rig  # noqa: E402
from whatslab.solvers.hand import (DexHandConfig, HandRetargetController,  # noqa: E402
                                   XRHandRetargeter, hand_config_names, mirror_xr_joints,
                                   xr_to_mano)
from whatslab.solvers.hand.dex_retargeter import resolve_hand_config  # noqa: E402
from whatslab.solvers.hand.xr_input import (OPENXR_TO_MANO, XR_TO_MANO,  # noqa: E402
                                            mano_palm_frame, xr_wrist_positions)

CONFIGS = ("orca_right", "orca_left", "dg5f_right", "dg5f_left", "robotis_right", "robotis_left")
RIGHT_MCP_X = {"index": -0.025, "middle": -0.005, "ring": 0.015, "pinky": 0.033}
SEG = (0.045, 0.026, 0.021)
FLEX = np.radians((80.0, 95.0, 60.0))


def _finger(mcp, curl):
    pts, p, phi = [np.asarray(mcp, float)], np.asarray(mcp, float), 0.0
    for L, a in zip(SEG, FLEX):
        phi += curl * a
        p = p + L * np.array([0.0, -np.sin(phi), -np.cos(phi)])
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
    for k, L in enumerate((0.04, 0.032, 0.026)):
        xr[3 + k, 4:7] = xr[2 + k, 4:7] + L * d
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


def _rigid(xr, R, t):
    out = xr.copy()
    out[:, 4:7] = xr[:, 4:7] @ R.T + t
    out[:, :4] = (Rotation.from_matrix(R) * Rotation.from_quat(xr[:, :4])).as_quat()
    return out


def _require_urdf(name):
    try:
        cfg = DexHandConfig.load(name)
    except FileNotFoundError as e:
        pytest.skip(str(e))
    if not os.path.isfile(os.path.join(models_root(), cfg.urdf)):
        pytest.skip(f"{name}: URDF 없음 ({cfg.urdf})")
    return cfg


def test_openxr_to_mano_index_list():
    assert OPENXR_TO_MANO == (1, 2, 3, 4, 5, 7, 8, 9, 10, 12, 13, 14, 15,
                              17, 18, 19, 20, 22, 23, 24, 25)
    assert np.isclose(np.linalg.det(XR_TO_MANO), 1.0)


def test_xr_to_mano_axes_on_hand_made_pose():
    xr = synthetic_right_hand(0.0)
    mano = xr_to_mano(xr)
    assert mano.shape == (21, 3)
    assert np.allclose(mano[0], 0.0)
    assert np.allclose(mano[5], [0.0, 0.025, 0.09])
    assert np.allclose(mano[17], [0.0, -0.033, 0.09])
    assert np.allclose(mano[8], XR_TO_MANO @ xr[10, 4:7])
    tips = mano[[8, 12, 16, 20]]
    assert np.all(tips[:, 2] > 0.15)
    curled = xr_to_mano(synthetic_right_hand(1.0))
    assert np.all(curled[[8, 12, 16, 20], 0] > 0.02), "굽힌 손끝이 손바닥(+x) 쪽이 아니다"


def test_xr_to_mano_ignores_wrist_pose():
    xr = synthetic_right_hand(0.4)
    a = np.radians(37.0)
    R = np.array([[np.cos(a), 0.0, np.sin(a)], [0.0, 1.0, 0.0], [-np.sin(a), 0.0, np.cos(a)]])
    moved = _rigid(xr, R, np.array([0.3, -0.1, 1.2]))
    assert np.allclose(xr_to_mano(moved), xr_to_mano(xr), atol=1e-9)
    assert np.allclose(xr_wrist_positions(moved), xr[:, 4:7], atol=1e-9)


def test_palm_frame_and_mirror():
    mano_r = xr_to_mano(synthetic_right_hand(0.0))
    R = mano_palm_frame(mano_r, "right")
    assert np.isclose(np.linalg.det(R), 1.0)
    assert R[1, 1] > 0.99 and R[2, 2] > 0.99, "오른손 손바닥 프레임이 MANO 축과 다르다"
    xr_l = mirror_xr_joints(synthetic_right_hand(0.0))
    mano_l = xr_to_mano(xr_l)
    assert np.allclose(mano_l, mano_r * [1.0, -1.0, 1.0])
    S = np.diag([1.0, -1.0, 1.0])
    assert np.allclose(mano_palm_frame(mano_l, "left"), S @ R @ S, atol=1e-9)
    back = mirror_xr_joints(xr_l)
    assert np.allclose(back[:, 4:7], synthetic_right_hand(0.0)[:, 4:7])


def test_hand_configs_are_shipped():
    names = hand_config_names()
    for n in ("orca_right", "orca_left", "dg5f_right", "dg5f_left"):
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
    m = pin.buildModelFromUrdf(eng.urdf_path)
    d = m.createData()
    qq = pin.neutral(m)
    for n, v in zip(eng.robot_joint_names, eng.full_qpos(q)):
        qq[m.idx_qs[m.getJointId(n)]] = v
    pin.framesForwardKinematics(m, d, qq)
    w = d.oMf[m.getFrameId(eng.config.retargeting["wrist_link_name"])].translation
    tips = eng.config.retargeting["finger_tip_link_names"][1:]
    return float(np.mean([np.linalg.norm(d.oMf[m.getFrameId(t)].translation - w)
                          for t in tips]))


def _settle(eng, xr, n=25):
    q = None
    for _ in range(n):
        q = eng.compute(xr)
    return q


@pytest.mark.parametrize("name", CONFIGS)
def test_retargeter_smoke(name):
    cfg = _require_urdf(name)
    eng = XRHandRetargeter(name)
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


def test_retargeter_mirrors_opposite_side_input():
    _require_urdf("orca_right")
    eng = XRHandRetargeter("orca_right", side="left")
    assert eng.mirror
    ref = XRHandRetargeter("orca_right")
    q_l = _settle(eng, hand_for("left", 0.7))
    q_r = _settle(ref, hand_for("right", 0.7))
    assert np.allclose(q_l, q_r, atol=1e-4)


def test_controller_uses_xr_joints():
    _require_urdf("orca_right")
    ctrl = HandRetargetController("right", "dex:orca")
    assert ctrl.xr_input
    assert ctrl.engine.config.name == "orca_right"
    assert not any("Carpals" in n and "TopTower" in n for n in ctrl.joint_names)
    held = ctrl.compute(InputSample(hand=HandPose(tracked=True), tracked=True))
    assert np.allclose(held.joint_angles, ctrl._last)
    cmd = ctrl.compute(InputSample(hand=HandPose(xr_joints=hand_for("right", 1.0),
                                                 tracked=True), tracked=True))
    assert cmd.joint_names == ctrl.joint_names
    assert np.abs(cmd.joint_angles).max() > 0.1
    lost = ctrl.compute(InputSample(hand=None, tracked=False))
    assert np.allclose(lost.joint_angles, cmd.joint_angles)


def test_controller_rejects_onnx_kwargs():
    _require_urdf("orca_right")
    with pytest.raises(TypeError, match="모르는 인자"):
        HandRetargetController("right", "dex:orca", onnx_path="x.onnx")


def test_rig_selects_dex_and_keeps_wrist_for_arm():
    _require_urdf("orca_right")
    rig = load_rig("rigs/nero_orca_right.yaml")
    assert rig.hand.retarget == "dex:orca"
    robot = RobotModel(rig)
    for side in ("left", "right"):
        ctrl = robot.make_hand_controller(rig.hand.retarget, side)
        assert ctrl.xr_input and ctrl.engine.config.side == side
        assert not set(ctrl.joint_names) & set(robot.arm_joint_names)
