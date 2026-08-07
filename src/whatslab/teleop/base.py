from __future__ import annotations

import os
import time
from abc import ABC, abstractmethod
from typing import Dict, Optional

import numpy as np

from whatslab.core.interfaces import HandController
from whatslab.core.types import Pose
from whatslab.robot import RobotArmIK, RobotModel, save_calibration
from .calibration import ArmCalibration


class TeleopModel(ABC):

    arm_source = None
    hand_source = None
    SIDES = ("left", "right")

    def __init__(self, robot):
        self.robots: Dict[str, RobotModel] = self._as_side_map(robot)

        uniq = {id(r): r for r in self.robots.values()}
        self.robot = next(iter(uniq.values())) if len(uniq) == 1 else None

        self.safety = None

        self.target: Dict[str, Optional[np.ndarray]] = {}
        self.q: Dict[str, Dict[str, float]] = {}

        self.ik: Dict[str, RobotArmIK] = {}
        self.retarget: Dict[str, HandController] = {}
        self.calib: Dict[str, ArmCalibration] = {}

        for s, r in self.robots.items():
            cfg = r.rig.hand.retarget if r.rig.hand is not None else None
            if r.has_arm:
                self.ik[s] = RobotArmIK(r)
                self.calib[s] = ArmCalibration(
                    reach_max=r.rig.solver.reach_max,
                    input_reach=r.rig.calibration.input_reach,
                    enabled=r.rig.calibration.enabled)
            if r.has_hand and cfg:
                self.retarget[s] = r.make_hand_controller(cfg, s)

    def _as_side_map(self, robot) -> Dict[str, RobotModel]:
        def _load(r):
            return RobotModel(r) if isinstance(r, (str, os.PathLike)) else r
        if robot is None:
            return {}
        if isinstance(robot, dict):
            return {s: _load(r) for s, r in robot.items()}
        if isinstance(robot, (list, tuple)):
            return {self.SIDES[i]: _load(r) for i, r in enumerate(robot) if r is not None}
        r = _load(robot)
        return {s: r for s in self.SIDES}

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
            out[s] = {
                "arm_pose": arm_pose,
                "fingers": hand_s,
                "q": self._joint_q(arm_s, hand_s),
                "tracked": arm_pose is not None or self._has_fingers(hand_s),
            }
        return out

    @staticmethod
    def _joint_q(arm_s, hand_s):
        if hand_s is not None and hand_s.joint_q is not None:
            return hand_s.joint_q
        if arm_s is not None and arm_s.joint_q is not None:
            return arm_s.joint_q
        return None

    def _solve_side(self, side: str, data: dict) -> Dict[str, float]:
        if data.get("q") is not None:
            return dict(data["q"])
        q: Dict[str, float] = {}
        ik = self.ik.get(side)
        T = data.get("arm_target")
        if ik is not None and T is not None:
            q_arm = np.asarray(ik.solve(T), dtype=float)
            q.update(zip(ik.joint_names, (float(v) for v in q_arm)))
        retarget = self.retarget.get(side)
        fingers = data.get("fingers")
        if retarget is not None and self._has_fingers(fingers):
            cmd = retarget.compute(fingers)
            q.update(zip(cmd.joint_names, (float(v) for v in cmd.joint_angles)))
        return q

    @staticmethod
    def _has_fingers(fingers) -> bool:
        return (fingers is not None and fingers.hand is not None
                and fingers.hand.tracked)

    def solve(self, data: Dict[str, dict]) -> Dict[str, Dict[str, float]]:
        return {s: self._solve_side(s, data[s]) for s in self.SIDES}

    def _apply_calib(self, data: Dict[str, dict]) -> Dict[str, dict]:
        for s in self.SIDES:
            calib = self.calib.get(s)
            if calib is not None:
                data[s] = calib.apply(data[s])
            self.target[s] = data[s].get("arm_target")
        return data

    def get_q(self) -> Dict[str, Dict[str, float]]:
        data = self._apply_calib(self.get_data())
        q = self.solve(data)
        if self.safety is not None:
            q = {s: self.safety.step(v) for s, v in q.items()}
        self.q = q
        return q

    def set_reach(self, input_reach: float) -> Dict[str, bool]:
        out: Dict[str, bool] = {}
        for s in self.SIDES:
            calib = self.calib.get(s)
            if calib is not None:
                calib.set_reach(float(input_reach))
            out[s] = calib is not None
        return out

    def calibrate_yaw(self) -> Dict[str, bool]:
        data = self.get_data()
        out: Dict[str, bool] = {}
        for s in self.SIDES:
            calib = self.calib.get(s)
            ok = bool(calib.capture(data[s])) if calib is not None else False
            out[s] = ok
            ik = self.ik.get(s)
            if ok and ik is not None and hasattr(ik, "reseed"):
                ik.reseed()
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
                    r_max[s] = max(r_max[s], float(np.linalg.norm(np.asarray(pose.pos, dtype=float))))
            time.sleep(period)
        for s in self.SIDES:
            calib = self.calib.get(s)
            if calib is not None and r_max[s] > 0.0:
                calib.set_reach(r_max[s])
                ik = self.ik.get(s)
                if ik is not None and hasattr(ik, "reseed"):
                    ik.reseed()
                if persist:
                    robot = self.robots.get(s)
                    if robot is not None:
                        save_calibration(robot.rig, r_max[s])
        return r_max

    def send_feedback(self, data) -> None:
        pass
