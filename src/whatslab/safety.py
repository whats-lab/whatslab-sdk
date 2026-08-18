from __future__ import annotations

import xml.etree.ElementTree as ET
from dataclasses import dataclass
from typing import Dict, Optional

_INF = float("inf")
_DT_MAX = 0.05


@dataclass(frozen=True)
class JointLimit:
    lower: float
    upper: float
    velocity: float


def load_limits_from_urdf(urdf_xml: str) -> Dict[str, JointLimit]:
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

    def __init__(self, limits: Dict[str, JointLimit], dt: float,
                 initial: Optional[Dict[str, float]] = None,
                 dt_max: Optional[float] = None):
        self._lim = dict(limits)
        self._dt = float(dt)
        self._dt_max = (float(dt_max) if dt_max is not None
                        else min(4.0 * float(dt), _DT_MAX))
        self._last: Dict[str, float] = dict(initial or {})
        self._estopped = False
        self._enabled = True

    def trip(self) -> None:
        self._estopped = True

    def reset(self) -> bool:
        self._estopped = False
        return True

    @property
    def estopped(self) -> bool:
        return self._estopped

    def set_enabled(self, enabled: bool) -> None:
        self._enabled = bool(enabled)

    @property
    def enabled(self) -> bool:
        return self._enabled

    def clone(self) -> "SafetyFilter":
        out = SafetyFilter(self._lim, self._dt, self._last, self._dt_max)
        out._estopped = self._estopped
        out._enabled = self._enabled
        return out

    def seed(self, positions: Dict[str, float]) -> None:
        self._last = dict(positions)

    @property
    def holding(self) -> bool:
        return self._estopped or not self._enabled

    def step(self, desired: Optional[Dict[str, float]],
             dt: Optional[float] = None) -> Dict[str, float]:
        if self._estopped or not self._enabled or desired is None:
            return dict(self._last)
        step_dt = self._dt if dt is None else min(max(float(dt), 0.0), self._dt_max)
        out: Dict[str, float] = {}
        for name, val in desired.items():
            lim = self._lim.get(name)
            prev = self._last.get(name)
            if lim is None:
                out[name] = prev if prev is not None else 0.0
                continue
            v = min(max(float(val), lim.lower), lim.upper)
            if prev is not None and lim.velocity != _INF:
                dmax = lim.velocity * step_dt
                v = min(max(v, prev - dmax), prev + dmax)
            out[name] = v
        self._last = {**self._last, **out}
        return dict(self._last)
