"""텔레옵/명령 안전 필터 — dep-light 순수 로직 (numpy·stdlib 만; pinocchio·dex-retargeting 무관).

defense-in-depth 의 한 층: non-ros2_control 드라이버용 폴백 + 텔레옵 품질층.
위치 clamp · 속도 rate-limit · watchdog(입력 끊김 → hold) · 래칭 e-stop · deadman.

소비자(ROS 게이트 노드·sim·직접 파이썬)가 이 로직을 감싸 쓴다. enforcement 프로세스와
HW e-stop 은 이 코드와 독립 — 여기는 "안전한 명령을 계산"할 뿐 최종 권위가 아니다.
plain ROS env(numpy2)에서도 `pip install whatslab-sdk`(core) 후 `import whatslab.safety` 가능.
"""
from __future__ import annotations

import xml.etree.ElementTree as ET
from dataclasses import dataclass
from typing import Dict, Optional

_INF = float("inf")


@dataclass(frozen=True)
class JointLimit:
    """관절 한계. velocity 는 |Δq|/s 최대(rate-limit 용). continuous 는 lower/upper=±inf."""
    lower: float
    upper: float
    velocity: float


def load_limits_from_urdf(urdf_xml: str) -> Dict[str, JointLimit]:
    """URDF 문자열 → {joint_name: JointLimit}. <limit> 있는 관절만(revolute/prismatic/
    continuous-with-limit). fixed/미한계 관절은 제외 → 필터가 hold(fail-safe)."""
    root = ET.fromstring(urdf_xml)
    out: Dict[str, JointLimit] = {}
    for j in root.findall("joint"):
        name = j.get("name")
        lim = j.find("limit")
        if name is None or lim is None:
            continue
        out[name] = JointLimit(
            lower=float(lim.get("lower", "-inf")),
            upper=float(lim.get("upper", "inf")),
            velocity=float(lim.get("velocity", "inf")),
        )
    return out


def tighten(base: Dict[str, JointLimit],
            override: Optional[dict]) -> Dict[str, JointLimit]:
    """override 로 **더 빡빡하게만** (교집합: max lower, min upper, min velocity).
    override[name] = {'lower':.., 'upper':.., 'velocity':..} (부분 지정 가능)."""
    out = dict(base)
    for name, o in (override or {}).items():
        b = base.get(name)
        lo, up, ve = o.get("lower", -_INF), o.get("upper", _INF), o.get("velocity", _INF)
        if b is not None:
            out[name] = JointLimit(max(lo, b.lower), min(up, b.upper), min(ve, b.velocity))
        else:
            out[name] = JointLimit(lo, up, ve)
    return out


class SafetyFilter:
    """틱마다 desired joint 위치를 안전한 명령으로 변환.

    hold 조건(래칭 e-stop / deadman off / 무입력=stale)이면 마지막 안전값 유지.
    아니면 위치 clamp → 속도 rate-limit(직전 대비). 미상 관절은 hold(fail-safe).
    """

    def __init__(self, limits: Dict[str, JointLimit], dt: float,
                 initial: Optional[Dict[str, float]] = None):
        self._lim = dict(limits)
        self._dt = float(dt)
        self._last: Dict[str, float] = dict(initial or {})
        self._estopped = False
        self._enabled = True

    # ---- 상태 ----
    def trip(self) -> None:
        self._estopped = True          # 래칭 — reset() 전까지 유지

    def reset(self) -> bool:
        self._estopped = False         # 명시적 해제만
        return True

    @property
    def estopped(self) -> bool:
        return self._estopped

    def set_enabled(self, enabled: bool) -> None:
        self._enabled = bool(enabled)

    def seed(self, positions: Dict[str, float]) -> None:
        """현재 로봇 상태로 last 초기화 — first-command ramp 기준."""
        self._last = dict(positions)

    @property
    def holding(self) -> bool:
        return self._estopped or not self._enabled

    # ---- 핵심 ----
    def step(self, desired: Optional[Dict[str, float]]) -> Dict[str, float]:
        """desired={name:pos} 또는 None(무입력/stale) → {name: safe pos}.
        hold 시 마지막 안전값을 그대로 반환(재발행용)."""
        if self._estopped or not self._enabled or desired is None:
            return dict(self._last)                         # hold
        out: Dict[str, float] = {}
        for name, val in desired.items():
            lim = self._lim.get(name)
            prev = self._last.get(name)
            if lim is None:                                 # 미상 관절 → hold
                out[name] = prev if prev is not None else 0.0
                continue
            v = min(max(float(val), lim.lower), lim.upper)  # 위치 clamp
            if prev is not None and lim.velocity != _INF:   # 속도 rate-limit
                dmax = lim.velocity * self._dt
                v = min(max(v, prev - dmax), prev + dmax)
            out[name] = v
        self._last = {**self._last, **out}
        return dict(self._last)
