from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional

import numpy as np


@dataclass
class HandPose:
    """사람 손 한 프레임.

    ``xr_joints``: OpenXR ``XrHandJointEXT`` 26관절 × ``[qx qy qz qw px py pz radius]``,
    손목 기준(m), OpenXR 손 관절 축. 손 리타게팅(dex·geort)의 입력이다. ``xr_valid`` 는 관절별
    유효 여부(없으면 전부 유효로 본다). 손목 자세는 담지 않는다 — 팔·손목은 사용하는 쪽 몫이다.
    """

    xr_joints: Optional[np.ndarray] = None
    xr_valid: Optional[np.ndarray] = None
    tracked: bool = False
    timestamp: float = 0.0


@dataclass
class InputSample:
    hand: Optional[HandPose] = None
    tracked: bool = False
    timestamp: float = 0.0


@dataclass
class HandCommand:
    joint_names: List[str] = field(default_factory=list)
    joint_angles: np.ndarray = field(default_factory=lambda: np.zeros(0))
