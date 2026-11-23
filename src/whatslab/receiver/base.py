"""수신기 공용: 기기 좌표계 → 정준 프레임 변환.

정준 프레임 = x 앞, y 왼쪽, z 위, 오른손계(ROS REP-103).
"""
from __future__ import annotations

from typing import Tuple

import numpy as np
from numpy.typing import ArrayLike
from scipy.spatial.transform import Rotation

from whatslab.core.constants import readonly_array

# 오른손계 y-up, -z 앞: WebXR · OpenVR(Vive) · OpenXR.
Y_UP_RH_TO_CANONICAL = readonly_array([[0.0, 0.0, -1.0],
                                       [-1.0, 0.0, 0.0],
                                       [0.0, 1.0, 0.0]])


def norm_quat(xyzw) -> np.ndarray:
    q = np.array(xyzw[:4], dtype=float)
    n = np.linalg.norm(q)
    return q / n if n > 1e-6 else np.array([0.0, 0.0, 0.0, 1.0])


def conjugate(basis: np.ndarray, rot: np.ndarray) -> np.ndarray:
    """회전 행렬 R 을 좌표계 변환 M 으로 옮긴다(M R Mᵀ)."""
    return basis @ rot @ basis.T


def to_canonical_quat(basis: np.ndarray, quat: ArrayLike) -> np.ndarray:
    """기기 좌표계 쿼터니언(xyzw)을 좌표계 변환 basis 로 정준 프레임에 옮긴다."""
    rot = Rotation.from_quat(np.asarray(quat, dtype=float)).as_matrix()
    return Rotation.from_matrix(conjugate(basis, rot)).as_quat()


def to_canonical_pose(
    basis: np.ndarray, pos: ArrayLike, quat: ArrayLike
) -> Tuple[np.ndarray, np.ndarray]:
    """기기 좌표계 (위치, 쿼터니언)을 좌표계 변환 basis 로 정준 프레임에 옮긴다."""
    return basis @ np.asarray(pos, dtype=float), to_canonical_quat(basis, quat)
