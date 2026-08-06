from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np
from scipy.spatial.transform import Rotation

_IDENTITY_QUAT = np.array([0.0, 0.0, 0.0, 1.0])


@dataclass
class Pose:

    pos: np.ndarray = field(default_factory=lambda: np.zeros(3))
    quat: np.ndarray = field(default_factory=lambda: _IDENTITY_QUAT.copy())  # xyzw

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


# ─────────────────────────────────────────────────────────────────────────
# 사람 손 골격 (joint 트리) — 단일 진실 출처
#
# 트리 순서(선언 순서) = 정규 23-포인트 레이아웃 인덱스. 이 스펙 하나가
#   · sensed=True 관절의 순서  = 손 FK/리타게터의 센서 입력 순서(JOINT_ORDER)
#   · 전체 노드 순서           = 리타게팅 config 의 human 인덱스(0~22)
# 를 모두 파생한다.
#
#   sensed : 센서가 회전을 직접 주는 관절(True) / FK 로 계산되는 팁·비센싱(False)
#   parent : 트리 부모 관절명 (None = root = wrist)
# ─────────────────────────────────────────────────────────────────────────
@dataclass(frozen=True)
class JointSpec:
    name: str
    parent: Optional[str]
    sensed: bool


HUMAN_HAND: Tuple[JointSpec, ...] = (
    JointSpec("wrist",       None,          False),   # 0  (root, 회전은 HandPose.wrist)
    JointSpec("thumb_cmc0",  "wrist",       True),    # 1
    JointSpec("thumb_cmc1",  "thumb_cmc0",  True),    # 2
    JointSpec("thumb_mcp",   "thumb_cmc1",  True),    # 3
    JointSpec("thumb_ip",    "thumb_mcp",   True),    # 4
    JointSpec("thumb_tip",   "thumb_ip",    False),   # 5
    JointSpec("index_mcp",   "wrist",       True),    # 6
    JointSpec("index_pip",   "index_mcp",   True),    # 7
    JointSpec("index_dip",   "index_pip",   True),    # 8
    JointSpec("index_tip",   "index_dip",   False),   # 9
    JointSpec("middle_mcp",  "wrist",       True),    # 10
    JointSpec("middle_pip",  "middle_mcp",  True),    # 11
    JointSpec("middle_dip",  "middle_pip",  True),    # 12
    JointSpec("middle_tip",  "middle_dip",  False),   # 13
    JointSpec("ring_mcp",    "wrist",       True),    # 14
    JointSpec("ring_pip",    "ring_mcp",    True),    # 15
    JointSpec("ring_dip",    "ring_pip",    True),    # 16
    JointSpec("ring_tip",    "ring_dip",    False),   # 17
    JointSpec("pinky0",      "wrist",       False),   # 18  (CMC, FK 에서 pinky_mcp 로 병합)
    JointSpec("pinky_mcp",   "pinky0",      True),    # 19
    JointSpec("pinky_pip",   "pinky_mcp",   True),    # 20
    JointSpec("pinky_dip",   "pinky_pip",   True),    # 21
    JointSpec("pinky_tip",   "pinky_dip",   False),   # 22
)

# 센서가 회전을 주는 관절 이름(트리 순서) = 손 FK 센서 입력 순서 (16개)
SENSED_JOINTS: List[str] = [j.name for j in HUMAN_HAND if j.sensed]
# 전체 노드 이름 → 정규 인덱스(0~22)
JOINT_INDEX: Dict[str, int] = {j.name: i for i, j in enumerate(HUMAN_HAND)}


@dataclass
class HandPose:

    wrist: Optional[Pose] = None
    joint_rot: Dict[str, np.ndarray] = field(default_factory=dict)
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
    hmd: Optional[Pose] = None            # HMD 6D (머리연동 상대 자세 기준). 없으면 None.
    joint_q: Optional[Dict[str, float]] = None
    tracked: bool = False
    timestamp: float = 0.0


@dataclass
class HandCommand:

    joint_names: List[str] = field(default_factory=list)
    joint_angles: np.ndarray = field(default_factory=lambda: np.zeros(0))  # rad
    gripper: Optional[float] = None       # 1-DOF 그리퍼(있으면)
    wrist: Optional[np.ndarray] = None    # 손목 직접명령(flex/roll 등, 있으면)
