"""여러 모듈이 함께 쓰는 상수의 단일 출처.

규칙: 한 모듈에서만 쓰는 상수는 그 모듈에 두고, 두 곳 이상이 쓰면 여기로 옮긴다. 다른
모듈에서 다시 export 하거나 별칭(`X = Y`)을 두지 않는다 — 쓰는 곳은 여기서 바로 import 한다.
바꿔 가며 쓰는 값(포트·타임아웃·게인)은 상수가 아니라 생성자 인자나 configs/*.yaml 이다.
"""

from __future__ import annotations

from typing import Final, Tuple

import numpy as np

SIDES: Final[Tuple[str, str]] = ("left", "right")
FINGER_COUNT: Final = 5  # 엄지 · 검지 · 중지 · 약지 · 소지
# 손가락 이름 — dexhand-description URDF 센서 링크 이름 규칙
# ({side}_sensor_<finger>_distal)에 쓴다.
FINGER_NAMES: Final[Tuple[str, ...]] = ("thumb", "index", "middle", "ring", "pinky")

# OpenXR XrHandJointEXT 26관절 — 모든 손 입력·리타게팅의 기준 순서.
XR_JOINT_NAMES: Final[Tuple[str, ...]] = (
    "palm",
    "wrist",
    "thumb_metacarpal",
    "thumb_proximal",
    "thumb_distal",
    "thumb_tip",
    "index_metacarpal",
    "index_proximal",
    "index_intermediate",
    "index_distal",
    "index_tip",
    "middle_metacarpal",
    "middle_proximal",
    "middle_intermediate",
    "middle_distal",
    "middle_tip",
    "ring_metacarpal",
    "ring_proximal",
    "ring_intermediate",
    "ring_distal",
    "ring_tip",
    "little_metacarpal",
    "little_proximal",
    "little_intermediate",
    "little_distal",
    "little_tip",
)
XR_JOINT_COUNT: Final = len(XR_JOINT_NAMES)
XR_JOINT_FIELDS: Final = 8  # [qx qy qz qw px py pz radius]
XR_WRIST: Final = 1

# 사람 손 21점 (MANO · MediaPipe 순서: 손목, 엄지 4, 검지~소지 각 4 — 손끝 포함).
# OpenXR 26 에서 palm 과 검지~소지 중수골 시작점(손바닥 안, 손가락 자세 정보 없음)만 뺀다.
OPENXR26_TO_HAND21: Final[Tuple[int, ...]] = (
    1,
    2,
    3,
    4,
    5,
    7,
    8,
    9,
    10,
    12,
    13,
    14,
    15,
    17,
    18,
    19,
    20,
    22,
    23,
    24,
    25,
)
HAND21_COUNT: Final = len(OPENXR26_TO_HAND21)
HAND21_WRIST: Final = 0
HAND21_INDEX_MCP: Final = 5
HAND21_MIDDLE_MCP: Final = 9
HAND21_RING_MCP: Final = 13
HAND21_LITTLE_MCP: Final = 17
HAND21_TIPS: Final[Tuple[int, ...]] = (4, 8, 12, 16, 20)


def readonly_array(a) -> np.ndarray:
    """상수 배열용 — 복사해서 쓰기 금지로 만든다(실수로 값을 바꾸면 ValueError)."""
    a = np.array(a, dtype=float)
    a.setflags(write=False)
    return a


# OpenXR 손 관절 축(-Z 손끝 방향, +Y 손등) → MANO/GeoRT 손 축.
# dex(xr_to_mano)·geort(to_geort) 공용.
OPENXR_TO_HAND21_AXES: Final = readonly_array(
    [[0.0, -1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, -1.0]]
)
