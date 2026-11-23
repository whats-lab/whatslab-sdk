from __future__ import annotations

from typing import Final, Tuple

import numpy as np

SIDES: Final[Tuple[str, str]] = ("left", "right")
FINGER_COUNT: Final = 5

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
XR_JOINT_FIELDS: Final = 8
XR_WRIST: Final = 1

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


def readonly_array(a) -> np.ndarray:
    a = np.array(a, dtype=float)
    a.setflags(write=False)
    return a


OPENXR_TO_HAND21_AXES: Final = readonly_array(
    [[0.0, -1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, -1.0]]
)
