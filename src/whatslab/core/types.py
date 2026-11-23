from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional

import numpy as np


@dataclass
class HandPose:

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
