from __future__ import annotations

from typing import Optional

import numpy as np
from scipy.spatial.transform import Rotation


def _yaw(R: np.ndarray) -> float:
    return float(np.arctan2(R[1, 0], R[0, 0]))


def _Rz(a: float) -> np.ndarray:
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


class ArmCalibration:

    def __init__(self, reach_max: Optional[float] = None,
                 input_reach: Optional[float] = None,
                 enabled: bool = True):
        self.reach_max = reach_max
        self.enabled = bool(enabled)
        self._W: Optional[np.ndarray] = None
        self._p0: Optional[np.ndarray] = None
        self._input_reach: Optional[float] = input_reach

    @property
    def scale(self) -> float:
        if self._input_reach and self.reach_max:
            return float(self.reach_max / self._input_reach)
        return 1.0

    @property
    def anchor(self) -> Optional[np.ndarray]:
        if self._p0 is None:
            return None
        return self.scale * self._p0

    def apply(self, data: dict) -> dict:
        pose = data.get("arm_pose")
        if pose is None:
            data["arm_target"] = None
            return data
        p = np.asarray(pose.pos, dtype=float)
        G = Rotation.from_quat(np.asarray(pose.quat, dtype=float)).as_matrix()
        T = np.eye(4)
        if not self.enabled:
            T[:3, 3] = p
            T[:3, :3] = G
            data["arm_target"] = T
            return data
        W = self._W
        if self._p0 is not None:
            d = p - self._p0
            T[:3, 3] = self.scale * (self._p0 + (W @ d if W is not None else d))
        else:
            T[:3, 3] = self.scale * p
        T[:3, :3] = (W @ G) if W is not None else G
        data["arm_target"] = T
        return data

    def capture(self, data: dict) -> bool:
        pose = data.get("arm_pose")
        if pose is None:
            return False
        G = Rotation.from_quat(np.asarray(pose.quat, dtype=float)).as_matrix()
        self._W = _Rz(-_yaw(G))
        self._p0 = np.asarray(pose.pos, dtype=float).copy()
        return True

    def set_reach(self, input_reach: float) -> None:
        self._input_reach = float(input_reach)

    @property
    def ready(self) -> bool:
        return self._W is not None
