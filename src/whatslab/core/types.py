from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional

import numpy as np
from scipy.spatial.transform import Rotation

_IDENTITY_QUAT = np.array([0.0, 0.0, 0.0, 1.0])


@dataclass
class Pose:

    pos: np.ndarray = field(default_factory=lambda: np.zeros(3))
    quat: np.ndarray = field(default_factory=lambda: _IDENTITY_QUAT.copy())

    def to_matrix(self) -> np.ndarray:
        tf = np.eye(4)
        tf[:3, :3] = Rotation.from_quat(self.quat).as_matrix()
        tf[:3, 3] = self.pos
        return tf

    @staticmethod
    def from_matrix(tf: np.ndarray) -> "Pose":
        mat = np.asarray(tf)
        return Pose(pos=mat[:3, 3].copy(),
                    quat=Rotation.from_matrix(mat[:3, :3]).as_quat())


@dataclass
class HandPose:
    """사람 손 한 프레임.

    ``xr_joints``: OpenXR ``XrHandJointEXT`` 26관절 × ``[qx qy qz qw px py pz radius]``,
    손목 기준(m), OpenXR 손 관절 축. 손 리타게팅(dex·geort)의 입력이다. ``xr_valid`` 는 관절별
    유효 여부(없으면 전부 유효로 본다). ``wrist`` 는 소스가 주는 손목 자세(없을 수 있다).
    """

    wrist: Optional[Pose] = None
    xr_joints: Optional[np.ndarray] = None
    xr_valid: Optional[np.ndarray] = None
    tracked: bool = False
    timestamp: float = 0.0


@dataclass
class InputSample:
    controller: Optional[Pose] = None
    hand: Optional[HandPose] = None
    hmd: Optional[Pose] = None
    joint_q: Optional[Dict[str, float]] = None
    tracked: bool = False
    timestamp: float = 0.0


@dataclass
class HandCommand:
    joint_names: List[str] = field(default_factory=list)
    joint_angles: np.ndarray = field(default_factory=lambda: np.zeros(0))
    gripper: Optional[float] = None
    wrist: Optional[np.ndarray] = None
