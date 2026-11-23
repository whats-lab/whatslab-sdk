from __future__ import annotations

from typing import Optional, Tuple

import numpy as np
from scipy.spatial.transform import Rotation

from whatslab.core.constants import (
    HAND21_INDEX_MCP,
    HAND21_LITTLE_MCP,
    HAND21_MIDDLE_MCP,
    HAND21_WRIST,
    OPENXR26_TO_HAND21,
    OPENXR_TO_HAND21_AXES,
    XR_JOINT_COUNT,
    XR_WRIST,
)

MANO_PALM: Tuple[int, ...] = (HAND21_WRIST, HAND21_INDEX_MCP, HAND21_MIDDLE_MCP, HAND21_LITTLE_MCP)
_MIRROR = np.diag([-1.0, 1.0, 1.0])


class HandInputError(ValueError):
    pass


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
    rot_w = _wrist_rotation(a)
    return (a[:, 4:7] - a[XR_WRIST, 4:7]) @ rot_w


def xr_to_mano(xr_joints) -> np.ndarray:
    return xr_wrist_positions(xr_joints)[list(OPENXR26_TO_HAND21)] @ OPENXR_TO_HAND21_AXES.T


def palm_frame(wrist, index, middle, pinky, side: str) -> np.ndarray:
    w, i, m, p = (np.asarray(v, dtype=float) for v in (wrist, index, middle, pinky))
    z = m - w
    nz = float(np.linalg.norm(z))
    if not nz >= 1e-6:
        raise HandInputError("손바닥 프레임: 손목과 중지 MCP 가 겹치거나 유한하지 않다")
    z = z / nz
    y = i - p if side == "right" else p - i
    y = y - z * float(y @ z)
    ny = float(np.linalg.norm(y))
    if not ny >= 1e-6:
        raise HandInputError("손바닥 프레임: 검지·소지 MCP 가 손가락 방향과 나란하다")
    y = y / ny
    return np.column_stack([np.cross(y, z), y, z])


def mano_palm_frame(mano: np.ndarray, side: str) -> np.ndarray:
    k = np.asarray(mano, dtype=float)
    return palm_frame(*(k[j] for j in MANO_PALM), side=side)


def mirror_xr_joints(xr_joints) -> np.ndarray:
    a = _as_xr(xr_joints)
    rot_w = _wrist_rotation(a)
    out = a.copy()
    quat = a[:, :4].copy()
    norm = np.linalg.norm(quat, axis=1)
    bad = ~(norm > 1e-9)
    quat[bad], norm[bad] = (0.0, 0.0, 0.0, 1.0), 1.0
    rot = Rotation.from_quat(quat / norm[:, None])
    local = rot_w.T @ rot.as_matrix()
    out[:, :4] = Rotation.from_matrix(_MIRROR @ local @ _MIRROR).as_quat()
    out[:, 4:7] = ((a[:, 4:7] - a[XR_WRIST, 4:7]) @ rot_w) @ _MIRROR
    return out


def masked_keypoints(keypoints: np.ndarray, valid: Optional[np.ndarray]) -> np.ndarray:
    kp = np.array(keypoints, dtype=np.float64, copy=True)
    if valid is not None:
        kp[~np.asarray(valid, dtype=bool), 4:7] = np.nan
    return kp
