"""whatslab.safety 단위테스트 — clamp / rate-limit / watchdog(hold) / 래칭 e-stop / deadman.

순수 로직 (rclpy·하드웨어 불필요). 안전 필터의 핵심 동작을 고정.
"""
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
    f = SafetyFilter(lim, dt=1.0, initial={"j1": 0.0})   # dmax=2.0 → rate 무관
    assert f.step({"j1": 5.0})["j1"] == 1.0              # upper 로 clamp
    f2 = SafetyFilter(lim, dt=1.0, initial={"j1": 0.0})
    assert f2.step({"j1": -5.0})["j1"] == -1.0           # lower 로 clamp


def test_rate_limit_caps_jump():
    lim = load_limits_from_urdf(URDF)
    f = SafetyFilter(lim, dt=0.1, initial={"j2": 0.0})   # vel 0.1 → dmax 0.01
    assert abs(f.step({"j2": 0.5})["j2"] - 0.01) < 1e-9  # 급점프 → 0.01 로 제한


def test_estop_latches_then_reset():
    lim = load_limits_from_urdf(URDF)
    f = SafetyFilter(lim, dt=1.0, initial={"j1": 0.3})
    f.trip()
    assert f.estopped
    assert f.step({"j1": 1.0})["j1"] == 0.3              # hold (latched)
    assert f.step({"j1": -1.0})["j1"] == 0.3             # 여전히 hold
    f.reset()
    assert not f.estopped
    assert f.step({"j1": 0.5})["j1"] == 0.5              # 재개


def test_deadman_disabled_holds():
    lim = load_limits_from_urdf(URDF)
    f = SafetyFilter(lim, dt=1.0, initial={"j1": 0.2})
    f.set_enabled(False)
    assert f.step({"j1": 1.0})["j1"] == 0.2


def test_watchdog_none_holds():
    lim = load_limits_from_urdf(URDF)
    f = SafetyFilter(lim, dt=1.0, initial={"j1": 0.1})
    assert f.step(None)["j1"] == 0.1                     # 무입력/stale → hold


def test_unknown_joint_holds_fail_safe():
    lim = load_limits_from_urdf(URDF)
    f = SafetyFilter(lim, dt=1.0)
    assert f.step({"jX": 5.0})["jX"] == 0.0              # 미상 관절 → hold


def test_tighten_intersects():
    lim = load_limits_from_urdf(URDF)
    t = tighten(lim, {"j1": {"upper": 0.5, "velocity": 1.0}})
    assert t["j1"].upper == 0.5      # 더 빡빡
    assert t["j1"].lower == -1.0     # base 유지
    assert t["j1"].velocity == 1.0
