from __future__ import annotations

import logging
from dataclasses import dataclass, field
from typing import Dict, Optional

import numpy as np

from whatslab.core.interfaces import HandController
from whatslab.core.types import Pose
from whatslab.robot import RobotArmIK, RobotModel

from .calibration import ArmCalibration

logger = logging.getLogger(__name__)


@dataclass
class SideModel:

    side: str
    robot: Optional[RobotModel] = None
    ik: Optional[RobotArmIK] = None
    retarget: Optional[HandController] = None
    calib: Optional[ArmCalibration] = None
    safety: Optional[object] = None

    raw_target: Optional[Pose] = None
    target: Optional[np.ndarray] = None
    q: Dict[str, float] = field(default_factory=dict)

    @classmethod
    def build(cls, side: str, robot: Optional[RobotModel] = None) -> "SideModel":
        out = cls(side=side, robot=robot)
        if robot is None:
            return out
        rig = getattr(robot, "rig", None)
        if getattr(robot, "has_arm", False):
            out.ik = RobotArmIK(robot)
            out.calib = ArmCalibration(
                reach_max=rig.solver.reach_max,
                input_reach=rig.calibration.input_reach,
                enabled=rig.calibration.enabled)
        cfg = rig.hand.retarget if rig is not None and rig.hand is not None else None
        if getattr(robot, "has_hand", False) and cfg:
            out.retarget = robot.make_hand_controller(cfg, side)
        return out

    def attach_safety(self, prototype) -> None:
        if hasattr(prototype, "clone"):
            self.safety = prototype.clone()
            return
        logger.warning(
            "%s: 안전필터에 clone() 이 없어 side 간 공유한다 — 두 side 가 같은 관절"
            " 이름을 쓰면 서로를 속도제한한다(실측 8.9 → 72.2mm). clone() 을 구현하라.",
            type(prototype).__name__)
        self.safety = prototype

    def filter(self, q: Dict[str, float], dt: Optional[float],
               prototype) -> Dict[str, float]:
        if self.safety is None:
            self.attach_safety(prototype)
        f = self.safety
        if f is not prototype:
            f.set_enabled(prototype.enabled)
            if prototype.estopped:
                f.trip()
            else:
                f.reset()
        return f.step(q, dt)

    def apply_calib(self, data: dict) -> dict:
        if self.calib is not None:
            data = self.calib.apply(data)
        self.target = data.get("arm_target")
        return data

    def solve(self, data: dict) -> Dict[str, float]:
        if data.get("q") is not None:
            return dict(data["q"])
        q: Dict[str, float] = {}
        T = data.get("arm_target")
        if self.ik is not None and T is not None:
            q_arm = np.asarray(self.ik.solve(T), dtype=float)
            q.update(zip(self.ik.joint_names, (float(v) for v in q_arm)))
        if self.retarget is not None and _has_fingers(data.get("fingers")):
            cmd = self.retarget.compute(data["fingers"])
            q.update(zip(cmd.joint_names, (float(v) for v in cmd.joint_angles)))
        return q

    def sync_ik(self, q: Dict[str, float]) -> None:
        if self.ik is None:
            return
        if all(n in q for n in self.ik.joint_names):
            self.ik.sync_state([q[n] for n in self.ik.joint_names])

    def reseed(self) -> None:
        if self.ik is not None and hasattr(self.ik, "reseed"):
            self.ik.reseed()


def _has_fingers(fingers) -> bool:
    return (fingers is not None and fingers.hand is not None
            and fingers.hand.tracked)
