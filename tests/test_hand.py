import numpy as np
import pytest

from whatslab.core.types import HandPose, InputSample
from whatslab.retarget.controller import HandRetargetController, engine_of


@pytest.mark.parametrize("name,engine", [("dex:orca", "dex:"), ("geort:orca_right", "geort:")])
def test_engine_prefix(name, engine):
    assert engine_of(name) == engine


@pytest.mark.parametrize("name", ["orca_hand", "dex:", "geort:", "", None, "uni:orca"])
def test_unknown_engine_rejected(name):
    with pytest.raises(ValueError):
        engine_of(name)


def test_retarget_package_import_is_light():
    import importlib
    import sys

    def ours():
        return [m for m in sys.modules if m.startswith("whatslab.retarget")]

    saved = {m: sys.modules.pop(m) for m in ours()}
    try:
        importlib.import_module("whatslab.retarget")
        assert "whatslab.retarget.dex_retargeter" not in sys.modules
    finally:
        for m in ours():
            del sys.modules[m]
        sys.modules.update(saved)


def _geort_model_or_skip():
    import os

    from whatslab.retarget.geort.solver import DEFAULT_MODEL, MODEL_ENV
    path = os.path.expanduser(os.environ.get(MODEL_ENV) or DEFAULT_MODEL)
    if not os.path.isfile(path):
        pytest.skip("GeoRT 모델 없음")


def _hand():
    j = np.zeros((26, 8))
    j[:, 3] = 1.0
    for root, x in {2: -0.03, 6: -0.02, 11: 0.0, 16: 0.02, 21: 0.04}.items():
        for k in range(4 if root == 2 else 5):
            j[root + k, 4:7] = [x, 0.0, -0.03 - 0.025 * k]
    j[1, 4:7] = 0.0
    return j


def test_geort_controller_through_dispatcher():
    _geort_model_or_skip()
    ctrl = HandRetargetController("right", "geort:orca_right")
    assert len(ctrl.joint_names) == 16
    cmd = ctrl.compute(InputSample(hand=HandPose(xr_joints=_hand(), tracked=True), tracked=True))
    assert cmd.joint_angles.shape == (16,) and np.all(np.isfinite(cmd.joint_angles))
    held = ctrl.compute(InputSample(hand=HandPose(tracked=False)))
    assert np.array_equal(held.joint_angles, cmd.joint_angles)


def test_geort_ignores_urdf_root():
    _geort_model_or_skip()
    ctrl = HandRetargetController("right", "geort:orca_right", urdf_root="/tmp")
    assert len(ctrl.joint_names) == 16
