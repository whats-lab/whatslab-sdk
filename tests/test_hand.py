import numpy as np
import pytest


def test_uni_retargeter_end_to_end():
    pytest.importorskip("onnxruntime")
    from whatslab.solvers.hand import UniRetargeter

    for robot in ("orca", "allegro", "tesollo", "robotis", "human"):
        for side in ("left", "right"):
            r = UniRetargeter(side, robot)
            assert r.joint_names, f"{robot}/{side}"
            neutral = {n: 0.0 for n in r.human_joint_names}
            q = r.compute(neutral)
            assert q.shape == (len(r.joint_names),)
            assert np.all(np.isfinite(q))
            lo, hi = r._feed["lo"], r._feed["hi"]
            slack = 0.02 * (hi - lo)
            assert np.all(q >= lo - slack) and np.all(q <= hi + slack)


def test_uni_retargeter_responds_to_input():
    pytest.importorskip("onnxruntime")
    from whatslab.solvers.hand import UniRetargeter

    r = UniRetargeter("left", "orca")
    q0 = r.compute({n: 0.0 for n in r.human_joint_names})
    q1 = r.compute({n: 0.4 for n in r.human_joint_names})
    assert np.abs(q1 - q0).max() > 0.05, "입력에 무반응 — 그래프가 상수화됐다"


def test_uni_retargeter_config_alias():
    pytest.importorskip("onnxruntime")
    from whatslab.solvers.hand import UniRetargeter

    a = UniRetargeter("left", "orca_hand")
    b = UniRetargeter("left", "orca")
    assert a.joint_names == b.joint_names
    with pytest.raises(ValueError, match="표에 없는"):
        UniRetargeter("left", "nope_hand")


def test_uni_retargeter_accepts_unprefixed_names():
    pytest.importorskip("onnxruntime")
    from whatslab.solvers.hand import UniRetargeter

    r = UniRetargeter("left", "tesollo")
    full = {n: 0.3 for n in r.human_joint_names}
    bare = {n.split("_", 1)[1]: 0.3 for n in r.human_joint_names}
    assert np.allclose(r.compute(full), r.compute(bare))


def test_hand_controller_from_input_sample():
    pytest.importorskip("onnxruntime")
    from whatslab.core.types import HandPose, InputSample
    from whatslab.solvers.hand import HandRetargetController

    ctrl = HandRetargetController("right", "orca_hand")
    angles = {n: 0.0 for n in ctrl.engine.human_joint_names}
    hand = HandPose(joint_angles=angles, tracked=True)
    cmd = ctrl.compute(InputSample(hand=hand, tracked=True))
    assert cmd.joint_names == ctrl.joint_names
    assert cmd.joint_angles.shape == (len(ctrl.joint_names),)
    cmd2 = ctrl.compute(InputSample(hand=None, tracked=False))
    assert np.allclose(cmd2.joint_angles, cmd.joint_angles)


def test_uni_retargeter_reads_the_side_it_was_asked_for():
    pytest.importorskip("onnxruntime")
    from whatslab.solvers.hand.uni_retargeter import UniRetargeter

    for side in ("left", "right"):
        r = UniRetargeter(side, "orca_hand")
        assert all(n.startswith(side + "_") for n in r.human_joint_names)


def test_uni_retargeter_responds_to_both_sides():
    pytest.importorskip("onnxruntime")
    import numpy as np

    from whatslab.solvers.hand.uni_retargeter import UniRetargeter

    for side in ("left", "right"):
        r = UniRetargeter(side, "orca_hand")
        flat = {n: 0.0 for n in r.human_joint_names}
        curl = {n: (0.8 if n.endswith(("_mcp_flex", "_pip")) else 0.0)
                for n in r.human_joint_names}
        assert np.abs(r.compute(curl) - r.compute(flat)).max() > 0.1, side
