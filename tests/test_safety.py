import pytest

from whatslab.safety import JointLimit, SafetyFilter, load_limits_from_urdf, tighten

URDF = """<robot name="t">
  <joint name="j1" type="revolute"><limit lower="-1.0" upper="1.0" velocity="2.0"/></joint>
  <joint name="j2" type="prismatic"><limit lower="0.0" upper="0.5" velocity="0.1"/></joint>
  <joint name="fixed1" type="fixed"/>
</robot>"""


def test_load_limits_excludes_fixed():
    lim = load_limits_from_urdf(URDF)
    assert set(lim) == {"j1", "j2"}
    assert lim["j1"] == JointLimit(-1.0, 1.0, 2.0)


def test_position_clamp():
    lim = load_limits_from_urdf(URDF)
    f = SafetyFilter(lim, dt=1.0, initial={"j1": 0.0})
    assert f.step({"j1": 5.0})["j1"] == 1.0
    f2 = SafetyFilter(lim, dt=1.0, initial={"j1": 0.0})
    assert f2.step({"j1": -5.0})["j1"] == -1.0


def test_rate_limit_caps_jump():
    lim = load_limits_from_urdf(URDF)
    f = SafetyFilter(lim, dt=0.1, initial={"j2": 0.0})
    assert abs(f.step({"j2": 0.5})["j2"] - 0.01) < 1e-9


def test_estop_latches_then_reset():
    lim = load_limits_from_urdf(URDF)
    f = SafetyFilter(lim, dt=1.0, initial={"j1": 0.3})
    f.trip()
    assert f.estopped
    assert f.step({"j1": 1.0})["j1"] == 0.3
    assert f.step({"j1": -1.0})["j1"] == 0.3
    f.reset()
    assert not f.estopped
    assert f.step({"j1": 0.5})["j1"] == 0.5


def test_deadman_disabled_holds():
    lim = load_limits_from_urdf(URDF)
    f = SafetyFilter(lim, dt=1.0, initial={"j1": 0.2})
    f.set_enabled(False)
    assert f.step({"j1": 1.0})["j1"] == 0.2


def test_watchdog_none_holds():
    lim = load_limits_from_urdf(URDF)
    f = SafetyFilter(lim, dt=1.0, initial={"j1": 0.1})
    assert f.step(None)["j1"] == 0.1


def test_unknown_joint_holds_fail_safe():
    lim = load_limits_from_urdf(URDF)
    f = SafetyFilter(lim, dt=1.0)
    assert f.step({"jX": 5.0})["jX"] == 0.0


def test_tighten_intersects():
    lim = load_limits_from_urdf(URDF)
    t = tighten(lim, {"j1": {"upper": 0.5, "velocity": 1.0}})
    assert t["j1"].upper == 0.5
    assert t["j1"].lower == -1.0
    assert t["j1"].velocity == 1.0


def test_step_uses_measured_dt():
    lim = {"j1": JointLimit(-10.0, 10.0, 1.0)}
    f = SafetyFilter(lim, dt=1.0 / 60.0, initial={"j1": 0.0})
    assert f.step({"j1": 5.0})["j1"] == pytest.approx(1.0 / 60.0)
    f.seed({"j1": 0.0})
    assert f.step({"j1": 5.0}, 1.0 / 20.0)["j1"] == pytest.approx(1.0 / 20.0)


def test_step_caps_dt_so_a_loop_hitch_cannot_authorize_a_jump():
    lim = {"j1": JointLimit(-10.0, 10.0, 1.0)}
    f = SafetyFilter(lim, dt=0.01, initial={"j1": 0.0})
    assert f._dt_max == pytest.approx(0.04)
    assert f.step({"j1": 5.0}, 2.0)["j1"] == pytest.approx(0.04)
    f2 = SafetyFilter(lim, dt=0.01, initial={"j1": 0.0}, dt_max=0.5)
    assert f2.step({"j1": 5.0}, 2.0)["j1"] == pytest.approx(0.5)


def test_step_negative_dt_is_clamped_to_zero():
    lim = {"j1": JointLimit(-10.0, 10.0, 1.0)}
    f = SafetyFilter(lim, dt=1.0, initial={"j1": 0.3})
    assert f.step({"j1": 5.0}, -1.0)["j1"] == pytest.approx(0.3)


def test_teleop_model_passes_measured_dt_to_filter():
    import time as _time

    from whatslab.teleop.base import TeleopModel

    class _Model(TeleopModel):
        def _get_raw_target(self):
            return {s: None for s in self.SIDES}

    m = _Model(None)
    seen = []

    class _Spy:
        def step(self, desired, dt=None):
            seen.append(dt)
            return {}

    m.safety = _Spy()
    m.solve = lambda data: {"right": {}}
    m.get_data = lambda: {s: {} for s in m.SIDES}
    m._apply_calib = lambda d: d

    m.get_q()
    assert seen == [None]
    _time.sleep(0.02)
    m.get_q()
    assert seen[-1] is not None and seen[-1] >= 0.015
