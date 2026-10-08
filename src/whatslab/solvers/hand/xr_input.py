from __future__ import annotations

from typing import Tuple

import numpy as np
from scipy.spatial.transform import Rotation

XR_JOINT_COUNT = 26
XR_WRIST = 1
OPENXR_TO_MANO: Tuple[int, ...] = (1, 2, 3, 4, 5, 7, 8, 9, 10, 12, 13, 14, 15,
                                   17, 18, 19, 20, 22, 23, 24, 25)
XR_TO_MANO = np.array([[0.0, -1.0, 0.0],
                       [-1.0, 0.0, 0.0],
                       [0.0, 0.0, -1.0]])
MANO_WRIST = 0
MANO_TIPS: Tuple[int, ...] = (4, 8, 12, 16, 20)
MANO_PALM: Tuple[int, ...] = (0, 5, 9, 17)
_MIRROR = np.diag([-1.0, 1.0, 1.0])


def _as_xr(xr_joints) -> np.ndarray:
    a = np.asarray(xr_joints, dtype=float)
    if a.ndim != 2 or a.shape[0] != XR_JOINT_COUNT or a.shape[1] < 7:
        raise ValueError(f"xr_joints 는 ({XR_JOINT_COUNT}, 7+) [qx qy qz qw px py pz ...]"
                         f" 이어야 한다 — 받은 shape {a.shape}")
    return a


def _wrist_rotation(a: np.ndarray) -> np.ndarray:
    q = a[XR_WRIST, :4]
    n = float(np.linalg.norm(q))
    if not np.isfinite(n) or n < 1e-9:
        return np.eye(3)
    return Rotation.from_quat(q / n).as_matrix()


def xr_wrist_positions(xr_joints) -> np.ndarray:
    a = _as_xr(xr_joints)
    Rw = _wrist_rotation(a)
    return (a[:, 4:7] - a[XR_WRIST, 4:7]) @ Rw


def xr_to_mano(xr_joints) -> np.ndarray:
    return xr_wrist_positions(xr_joints)[list(OPENXR_TO_MANO)] @ XR_TO_MANO.T


def palm_frame(wrist, index, middle, pinky, side: str) -> np.ndarray:
    w, i, m, p = (np.asarray(v, dtype=float) for v in (wrist, index, middle, pinky))
    z = m - w
    nz = float(np.linalg.norm(z))
    if nz < 1e-6:
        raise ValueError("손바닥 프레임: 손목과 중지 MCP 가 겹친다")
    z = z / nz
    y = i - p if side == "right" else p - i
    y = y - z * float(y @ z)
    ny = float(np.linalg.norm(y))
    if ny < 1e-6:
        raise ValueError("손바닥 프레임: 검지·소지 MCP 가 손가락 방향과 나란하다")
    y = y / ny
    return np.column_stack([np.cross(y, z), y, z])


def mano_palm_frame(mano: np.ndarray, side: str) -> np.ndarray:
    k = np.asarray(mano, dtype=float)
    return palm_frame(*(k[j] for j in MANO_PALM), side=side)


def mirror_xr_joints(xr_joints) -> np.ndarray:
    a = _as_xr(xr_joints)
    Rw = _wrist_rotation(a)
    out = a.copy()
    rot = Rotation.from_quat(a[:, :4] / np.linalg.norm(a[:, :4], axis=1, keepdims=True))
    local = Rw.T @ rot.as_matrix()
    out[:, :4] = Rotation.from_matrix(_MIRROR @ local @ _MIRROR).as_quat()
    out[:, 4:7] = ((a[:, 4:7] - a[XR_WRIST, 4:7]) @ Rw) @ _MIRROR
    return out
