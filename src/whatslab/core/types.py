from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np
from scipy.spatial.transform import Rotation

_IDENTITY_QUAT = np.array([0.0, 0.0, 0.0, 1.0])


@dataclass
class Pose:

    pos: np.ndarray = field(default_factory=lambda: np.zeros(3))
    quat: np.ndarray = field(default_factory=lambda: _IDENTITY_QUAT.copy())

    def to_matrix(self) -> np.ndarray:
        T = np.eye(4)
        T[:3, :3] = Rotation.from_quat(self.quat).as_matrix()
        T[:3, 3] = self.pos
        return T

    @staticmethod
    def from_matrix(T: np.ndarray) -> "Pose":
        A = np.asarray(T)
        return Pose(pos=A[:3, 3].copy(),
                    quat=Rotation.from_matrix(A[:3, :3]).as_quat())


@dataclass(frozen=True)
class JointSpec:
    name: str
    parent: Optional[str]
    sensed: bool


HUMAN_HAND: Tuple[JointSpec, ...] = (
    JointSpec("wrist",       None,          False),
    JointSpec("thumb_cmc0",  "wrist",       True),
    JointSpec("thumb_cmc1",  "thumb_cmc0",  True),
    JointSpec("thumb_mcp",   "thumb_cmc1",  True),
    JointSpec("thumb_ip",    "thumb_mcp",   True),
    JointSpec("thumb_tip",   "thumb_ip",    False),
    JointSpec("index_mcp",   "wrist",       True),
    JointSpec("index_pip",   "index_mcp",   True),
    JointSpec("index_dip",   "index_pip",   True),
    JointSpec("index_tip",   "index_dip",   False),
    JointSpec("middle_mcp",  "wrist",       True),
    JointSpec("middle_pip",  "middle_mcp",  True),
    JointSpec("middle_dip",  "middle_pip",  True),
    JointSpec("middle_tip",  "middle_dip",  False),
    JointSpec("ring_mcp",    "wrist",       True),
    JointSpec("ring_pip",    "ring_mcp",    True),
    JointSpec("ring_dip",    "ring_pip",    True),
    JointSpec("ring_tip",    "ring_dip",    False),
    JointSpec("pinky0",      "wrist",       False),
    JointSpec("pinky_mcp",   "pinky0",      True),
    JointSpec("pinky_pip",   "pinky_mcp",   True),
    JointSpec("pinky_dip",   "pinky_pip",   True),
    JointSpec("pinky_tip",   "pinky_dip",   False),
)

SENSED_JOINTS: List[str] = [j.name for j in HUMAN_HAND if j.sensed]
JOINT_INDEX: Dict[str, int] = {j.name: i for i, j in enumerate(HUMAN_HAND)}


@dataclass
class HandPose:

    wrist: Optional[Pose] = None
    joint_rot: Dict[str, np.ndarray] = field(default_factory=dict)
    joint_angles: Dict[str, float] = field(default_factory=dict)
    tracked: bool = False
    timestamp: float = 0.0

    def to_sensor_array(self) -> np.ndarray:
        arr = np.tile(_IDENTITY_QUAT, (1 + len(SENSED_JOINTS), 1)).astype(float)
        if self.wrist is not None:
            arr[0] = self.wrist.quat
        for i, name in enumerate(SENSED_JOINTS):
            q = self.joint_rot.get(name)
            if q is not None:
                arr[1 + i] = q
        return arr

    @staticmethod
    def from_sensor_array(arr17: np.ndarray, wrist_pos: Optional[np.ndarray] = None,
                          tracked: bool = True, timestamp: float = 0.0) -> "HandPose":
        a = np.asarray(arr17, dtype=float)
        wrist = Pose(pos=(np.zeros(3) if wrist_pos is None else np.asarray(wrist_pos, float)),
                     quat=a[0].copy())
        joint_rot = {name: a[1 + i].copy() for i, name in enumerate(SENSED_JOINTS)}
        return HandPose(wrist=wrist, joint_rot=joint_rot, tracked=tracked, timestamp=timestamp)


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
