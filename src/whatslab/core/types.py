"""공통 데이터 타입 + 사람 손 골격 스펙 (단일 진실 출처).

레이어 간 표현을 한 곳에 정의해, receiver·손·팔이 서로의 구체 클래스를 모르고도
협력하게 한다. 특히 사람 손은 매직 인덱스 배열 대신 **이름 기반 관절 트리**로 표현해,
receiver → hand/arm 텔레옵이 이름으로 조회하도록 한다.

이 모듈은 whatslab 의 최하위 계약 — 무거운 의존 없음(numpy/scipy 만).
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np
from scipy.spatial.transform import Rotation

_IDENTITY_QUAT = np.array([0.0, 0.0, 0.0, 1.0])


@dataclass
class Pose:
    """위치 + 회전(quaternion xyzw). 좌표계는 호출 측 약속(예: HMD 로컬)."""

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
    """사람 손 한 프레임 (이름 기반).

    wrist      : 손목 6D pose (pos+quat). arm 텔레옵의 EE 입력. 소스에 따라 pos 없음
                 (예: AirGlove 는 회전만) → 그 경우 pos 는 신뢰하지 말 것.
    joint_rot  : 관절명 → local quaternion(xyzw). sensed 관절만 채운다.
    """

    wrist: Optional[Pose] = None
    joint_rot: Dict[str, np.ndarray] = field(default_factory=dict)
    tracked: bool = False
    timestamp: float = 0.0

    def to_sensor_array(self) -> np.ndarray:
        """손 FK/리타게터 입력용 (17,4) 배열: [0]=wrist 회전, [1:17]=SENSED_JOINTS 순서."""
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
        """(17,4) 센서 배열 → HandPose. [0]=wrist 회전, [1:17]=SENSED_JOINTS."""
        a = np.asarray(arr17, dtype=float)
        wrist = Pose(pos=(np.zeros(3) if wrist_pos is None else np.asarray(wrist_pos, float)),
                     quat=a[0].copy())
        joint_rot = {name: a[1 + i].copy() for i, name in enumerate(SENSED_JOINTS)}
        return HandPose(wrist=wrist, joint_rot=joint_rot, tracked=tracked, timestamp=timestamp)


@dataclass
class InputSample:
    """한 손(side)의 한 프레임 입력.

    controller : 컨트롤러 6D (글러브 모드의 팔 IK 입력). 없으면 None.
    hand       : 사람 손 포즈(손목+손가락). 손 텔레옵 입력이자 arm 의 손목 입력원.
    joint_q    : 로봇 관절각 직접 지정(이름→rad). 있으면 IK/리타게팅을 건너뛰고
                 그대로 반환한다(TeleopModel.get_q 의 bypass 경로). 없으면 None.
    소스마다 채우는 필드가 다르다(Quest=hand(+wrist pos), 글러브=hand(회전만)).
    """

    controller: Optional[Pose] = None
    hand: Optional[HandPose] = None
    hmd: Optional[Pose] = None            # HMD 6D (머리연동 상대 자세 기준). 없으면 None.
    joint_q: Optional[Dict[str, float]] = None
    tracked: bool = False
    timestamp: float = 0.0


@dataclass
class HandCommand:
    """손 리타게팅/그리퍼 공통 출력."""

    joint_names: List[str] = field(default_factory=list)
    joint_angles: np.ndarray = field(default_factory=lambda: np.zeros(0))  # rad
    gripper: Optional[float] = None       # 1-DOF 그리퍼(있으면)
    wrist: Optional[np.ndarray] = None    # 손목 직접명령(flex/roll 등, 있으면)
