import numpy as np
import pytest

from whatslab.core.types import HandPose, InputSample
from whatslab.retarget.retargeter import HandRetargeter, engine_of


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
        assert "whatslab.retarget.dex" not in sys.modules
    finally:
        for m in ours():
            del sys.modules[m]
        sys.modules.update(saved)


def _geort_model_or_skip():
    import os

    from whatslab.retarget.geort.engine import DEFAULT_MODEL, MODEL_ENV
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


def test_geort_through_retargeter():
    _geort_model_or_skip()
    ctrl = HandRetargeter("geort:orca_right", side="right")
    assert len(ctrl.joint_names) == 16
    cmd = ctrl.compute(InputSample(hand=HandPose(xr_joints=_hand(), tracked=True), tracked=True))
    assert cmd.joint_angles.shape == (16,) and np.all(np.isfinite(cmd.joint_angles))
    held = ctrl.compute(InputSample(hand=HandPose(tracked=False)))
    assert np.array_equal(held.joint_angles, cmd.joint_angles)


def test_geort_ignores_urdf_root():
    _geort_model_or_skip()
    ctrl = HandRetargeter("geort:orca_right", side="right", urdf_root="/tmp")
    assert len(ctrl.joint_names) == 16


def test_dex_preprocess_import_is_light():
    import importlib
    import sys

    def ours():
        return [m for m in sys.modules if m.startswith("whatslab.retarget")]

    saved = {m: sys.modules.pop(m) for m in ours()}
    try:
        retarget = importlib.import_module("whatslab.retarget")
        assert callable(retarget.xr_to_mano) and callable(retarget.mirror_xr_joints)
        assert "whatslab.retarget.dex.preprocess" in sys.modules
        assert "whatslab.retarget.dex.engine" not in sys.modules
    finally:
        for m in ours():
            del sys.modules[m]
        sys.modules.update(saved)


class _Source:
    def __init__(self):
        self.tracked = True

    def start(self):
        pass

    def stop(self):
        pass

    def get(self, side):
        if not self.tracked:
            return InputSample(hand=HandPose(tracked=False))
        return InputSample(hand=HandPose(xr_joints=_hand(), tracked=True), tracked=True)


def test_safety_does_not_hide_tracking_loss():
    _geort_model_or_skip()
    from whatslab.hand import HandModel
    from whatslab.safety import JointLimit, SafetyFilter

    src = _Source()
    m = HandModel("geort:orca_right", side="right", hand_source=src)
    lim = m.retarget.engine.joint_limits
    m.safety = SafetyFilter({n: JointLimit(lo, hi, float("inf"))
                             for n, (lo, hi) in zip(m.retarget.joint_names, lim)}, dt=0.01)
    assert len(m.get_q()["right"]) == 16
    src.tracked = False
    assert m.get_q() == {"right": {}}


def test_geort_engine_satisfies_engine_protocol():
    _geort_model_or_skip()
    from whatslab.retarget import Engine, GeortEngine

    assert isinstance(GeortEngine.from_config("orca_right"), Engine)
    assert not isinstance(object(), Engine)
