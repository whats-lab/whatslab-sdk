"""receiver 공용 유틸/상수.

모든 receiver 는 core.Receiver 프로토콜(start/stop/get(side)->InputSample)을 따른다.
python-osc 등 무거운 의존은 각 구현의 start() 안에서 lazy import 한다
(모듈 import 만으로 `whatslab-sdk[receiver]` 를 강제하지 않기 위함).
"""
from __future__ import annotations

import numpy as np

# 손가락 16관절 순서 (Unity/AGA 송신 순서 = 손 리타게터 JOINT_ORDER 와 동일)
FINGER_JOINT_ORDER = [
    "thumb_cmc0", "thumb_cmc1", "thumb_mcp", "thumb_ip",
    "index_mcp", "index_pip", "index_dip",
    "middle_mcp", "middle_pip", "middle_dip",
    "ring_mcp", "ring_pip", "ring_dip",
    "pinky_mcp", "pinky_pip", "pinky_dip",
]
NUM_FINGER_JOINTS = 16


def neutral_finger_quats() -> np.ndarray:
    """17×4 항등 쿼터니언. [0]=손목(root), [1:17]=16관절 (리타게터 입력 레이아웃)."""
    q = np.zeros((17, 4))
    q[:, 3] = 1.0
    return q


def norm_quat(xyzw) -> np.ndarray:
    q = np.array(xyzw[:4], dtype=float)
    n = np.linalg.norm(q)
    return q / n if n > 1e-6 else np.array([0.0, 0.0, 0.0, 1.0])
