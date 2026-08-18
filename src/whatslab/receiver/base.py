from __future__ import annotations

import numpy as np

NUM_FINGER_JOINTS = 16


def neutral_finger_quats() -> np.ndarray:
    q = np.zeros((17, 4))
    q[:, 3] = 1.0
    return q


def norm_quat(xyzw) -> np.ndarray:
    q = np.array(xyzw[:4], dtype=float)
    n = np.linalg.norm(q)
    return q / n if n > 1e-6 else np.array([0.0, 0.0, 0.0, 1.0])
