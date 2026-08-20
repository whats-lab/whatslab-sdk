from __future__ import annotations

import logging
import os
import time
from abc import ABC, abstractmethod
from typing import Dict, Optional

import numpy as np

from whatslab.core.types import Pose
from whatslab.robot import RobotModel, save_calibration
from whatslab.robot.config import RigConfig, load_rig

from .side import SideModel, _has_fingers

logger = logging.getLogger(__name__)


class TeleopModel(ABC):

    arm_source = None
    hand_source = None
    SIDES = ("left", "right")

    def __init__(self, robot):
        robots = self._as_side_map(robot)
        self.sides: Dict[str, SideModel] = {
            s: SideModel.build(s, robots.get(s))
            for s in (*self.SIDES, *(k for k in robots if k not in self.SIDES))}

        have = [v.robot for v in self.sides.values() if v.robot is not None]
        rigs = {id(getattr(r, "rig", r)) for r in have}
        self.robot = have[0] if len(rigs) == 1 else None

        self._safety = None
        self._t_prev = None

    @staticmethod
    def _as_rig(r):
        if isinstance(r, RigConfig):
            return r
        if isinstance(r, (str, os.PathLike)):
            return load_rig(os.fspath(r))
        return None

    def _as_side_map(self, robot) -> Dict[str, RobotModel]:
        def _load(r):
            rig = self._as_rig(r)
            return RobotModel(rig) if rig is not None else r
        if robot is None:
            return {}
        if isinstance(robot, dict):
            return {s: _load(r) for s, r in robot.items()}
        if isinstance(robot, (list, tuple)):
            return {self.SIDES[i]: _load(r) for i, r in enumerate(robot) if r is not None}
        rig = self._as_rig(robot)
        if rig is not None:
            return {s: RobotModel(rig) for s in self.SIDES}
        logger.warning(
            "%s 인스턴스를 주면 양쪽 side 가 그 유상태 솔버를 공유한다 — side 가"
            " 서로를 밀어낸다(실측 2.6 → 347mm). rig 경로/RigConfig 를 주거나"
            " {side: model} 로 따로 주라.", type(robot).__name__)
        return {s: robot for s in self.SIDES}

    @property
    def _receivers(self) -> list:
        out, seen = [], set()
        for r in (self.arm_source, self.hand_source):
            if r is not None and id(r) not in seen:
                seen.add(id(r))
                out.append(r)
        return out

    def start(self) -> None:
        for r in self._receivers:
            r.start()

    def stop(self) -> None:
        for r in self._receivers:
            r.stop()

    @abstractmethod
    def _get_raw_target(self) -> Dict[str, Optional[Pose]]:
        ...

    def get_data(self) -> Dict[str, dict]:
        poses = self._get_raw_target()
        out: Dict[str, dict] = {}
        for s in self.SIDES:
            arm_s = self.arm_source.get(s) if self.arm_source else None
            hand_s = self.hand_source.get(s) if self.hand_source else None
            arm_pose = poses.get(s)
            self.sides[s].raw_target = arm_pose
            out[s] = {
                "arm_pose": arm_pose,
                "fingers": hand_s,
                "q": self._joint_q(arm_s, hand_s),
                "tracked": arm_pose is not None or _has_fingers(hand_s),
            }
        return out

    @staticmethod
    def _joint_q(arm_s, hand_s):
        if hand_s is not None and hand_s.joint_q is not None:
            return hand_s.joint_q
        if arm_s is not None and arm_s.joint_q is not None:
            return arm_s.joint_q
        return None

    def solve(self, data: Dict[str, dict]) -> Dict[str, Dict[str, float]]:
        return {s: self.sides[s].solve(data[s]) for s in self.SIDES}

    def _apply_calib(self, data: Dict[str, dict]) -> Dict[str, dict]:
        for s in self.SIDES:
            data[s] = self.sides[s].apply_calib(data[s])
        return data

    @property
    def safety(self):
        return self._safety

    @safety.setter
    def safety(self, f) -> None:
        self._safety = f
        for m in self.sides.values():
            m.safety = None

    def get_q(self) -> Dict[str, Dict[str, float]]:
        data = self._apply_calib(self.get_data())
        q = self.solve(data)
        if self._safety is not None:
            now = time.monotonic()
            dt = None if self._t_prev is None else now - self._t_prev
            self._t_prev = now
            for s, v in q.items():
                m = self.sides[s]
                q[s] = m.filter(v, dt, self._safety)
                m.sync_ik(q[s])
        for s, v in q.items():
            self.sides[s].q = v
        return q

    @property
    def q(self) -> Dict[str, Dict[str, float]]:
        return {s: m.q for s, m in self.sides.items()}

    def set_reach(self, input_reach: float) -> Dict[str, bool]:
        out: Dict[str, bool] = {}
        for s in self.SIDES:
            calib = self.sides[s].calib
            if calib is not None:
                calib.set_reach(float(input_reach))
            out[s] = calib is not None
        return out

    def calibrate_yaw(self) -> Dict[str, bool]:
        data = self.get_data()
        out: Dict[str, bool] = {}
        for s in self.SIDES:
            m = self.sides[s]
            ok = bool(m.calib.capture(data[s])) if m.calib is not None else False
            out[s] = ok
            if ok:
                m.reseed()
        return out

    def calibrate_reach(self, duration: float = 8.0, rate_hz: float = 60.0,
                        persist: bool = False) -> Dict[str, float]:
        r_max: Dict[str, float] = {s: 0.0 for s in self.SIDES}
        period, t_end = 1.0 / rate_hz, time.monotonic() + duration
        while time.monotonic() < t_end:
            data = self.get_data()
            for s in self.SIDES:
                pose = data[s].get("arm_pose")
                if pose is not None:
                    r_max[s] = max(r_max[s], float(np.linalg.norm(
                        np.asarray(pose.pos, dtype=float))))
            time.sleep(period)
        for s in self.SIDES:
            m = self.sides[s]
            if m.calib is None or r_max[s] <= 0.0:
                continue
            m.calib.set_reach(r_max[s])
            m.reseed()
            if persist and getattr(m.robot, "rig", None) is not None:
                save_calibration(m.robot.rig, r_max[s])
        return r_max

    def send_feedback(self, data) -> None:
        pass
