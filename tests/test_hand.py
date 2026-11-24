import importlib
import sys

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


def _retarget_modules():
    return [m for m in sys.modules if m.startswith("whatslab.retarget")]


@pytest.fixture
def fresh_retarget():
    saved = {m: sys.modules.pop(m) for m in _retarget_modules()}
    yield importlib.import_module("whatslab.retarget")
    for m in _retarget_modules():
        del sys.modules[m]
    sys.modules.update(saved)


def test_retarget_package_import_is_light(fresh_retarget):
    assert "whatslab.retarget.dex" not in sys.modules


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


def test_dex_preprocess_import_is_light(fresh_retarget):
    assert callable(fresh_retarget.xr_to_mano) and callable(fresh_retarget.mirror_xr_joints)
    assert "whatslab.retarget.dex.preprocess" in sys.modules
    assert "whatslab.retarget.dex.engine" not in sys.modules


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


class _FlakyEngine:
    joint_names = ["j"]
    joint_limits = np.array([[-1.0, 1.0]])

    def __init__(self):
        self.fail = False
        self.last_failure = None
        self.resets = 0

    def solve(self, xr_joints):
        self.last_failure = "못 풂" if self.fail else None
        return None if self.fail else np.array([0.5])

    def reset(self):
        self.resets += 1
        self.last_failure = None


def test_engine_none_holds_last_command(caplog):
    from whatslab.retarget import Engine

    engine = _FlakyEngine()
    assert isinstance(engine, Engine)
    ctrl = HandRetargeter.__new__(HandRetargeter)
    ctrl._engine = engine
    ctrl.engine_name = "dex:"
    ctrl._last = np.zeros(1)
    ctrl._tracking = False
    ctrl.last_failure = None
    sample = InputSample(hand=HandPose(xr_joints=_hand(), tracked=True), tracked=True)
    assert ctrl.compute(sample).joint_angles[0] == 0.5
    engine.fail = True
    with caplog.at_level("WARNING"):
        for _ in range(3):
            assert ctrl.compute(sample).joint_angles[0] == 0.5
            assert ctrl.last_failure == "못 풂"
    assert sum("직전 명령을 유지" in r.message for r in caplog.records) == 1
    assert engine.resets == 3
    engine.fail = False
    ctrl.compute(sample)
    assert ctrl.last_failure is None
