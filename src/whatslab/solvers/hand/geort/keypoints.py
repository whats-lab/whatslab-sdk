"""GeoRT 입력 좌표계: OpenXR 26관절 → 손목 원점·템플릿 정렬·손 크기로 맞춘 21점."""
from __future__ import annotations

import dataclasses

import numpy as np

from whatslab.core.constants import (
    HAND21_INDEX_MCP,
    HAND21_LITTLE_MCP,
    HAND21_MIDDLE_MCP,
    HAND21_RING_MCP,
    HAND21_WRIST,
    OPENXR26_TO_HAND21,
    OPENXR_TO_HAND21_AXES,
)

PALM_MCPS: tuple[int, ...] = (
    HAND21_INDEX_MCP,
    HAND21_MIDDLE_MCP,
    HAND21_RING_MCP,
    HAND21_LITTLE_MCP,
)
POSITION = slice(4, 7)
MIN_AXIS_NORM = 1e-6
MIRROR_X = np.array([-1.0, 1.0, 1.0])
FRAME_V2 = "openxr-wrist-kabsch-v2"

_INDEX = np.array(OPENXR26_TO_HAND21, dtype=np.intp)
_MCPS = np.array(PALM_MCPS, dtype=np.intp)


def mediapipe21(joints: np.ndarray) -> np.ndarray:
    """OpenXR 26관절(…×26×8)에서 MediaPipe 21점 위치(…×21×3)를 고른다."""
    xr = np.asarray(joints, dtype=np.float64)
    return xr[..., _INDEX, POSITION]


def palm_axes(points: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """21점의 손바닥 좌표축(열: x·y·z)과 원점(손목). 축을 만들 수 없으면 NaN."""
    z = points[..., HAND21_MIDDLE_MCP, :] - points[..., HAND21_WRIST, :]
    across = points[..., HAND21_INDEX_MCP, :] - points[..., HAND21_RING_MCP, :]
    z_norm = np.linalg.norm(z, axis=-1, keepdims=True)
    z = z / np.where(z_norm > MIN_AXIS_NORM, z_norm, np.nan)
    across_norm = np.linalg.norm(across, axis=-1, keepdims=True)
    across = across / np.where(across_norm > MIN_AXIS_NORM, across_norm, np.nan)
    x = np.cross(across, z)
    x_norm = np.linalg.norm(x, axis=-1, keepdims=True)
    x = x / np.where(x_norm > MIN_AXIS_NORM, x_norm, np.nan)
    y = np.cross(z, x)
    return np.stack([x, y, z], axis=-1), points[..., HAND21_WRIST, :]


def human_keypoints(joints: np.ndarray, mirror: bool = False) -> np.ndarray:
    """OpenXR 26관절 → 손바닥 좌표계 21점(손목 원점)."""
    points = mediapipe21(joints)
    rotation, origin = palm_axes(points)
    local = np.einsum("...pi,...ij->...pj", points - origin[..., None, :], rotation)
    if mirror:
        local = local * MIRROR_X
    return local


def hand_size(points: np.ndarray) -> np.ndarray:
    """손목–MCP 네 점 거리의 평균(손 크기)."""
    rel = points[..., _MCPS, :] - points[..., HAND21_WRIST, None, :]
    return np.linalg.norm(rel, axis=-1).mean(axis=-1)


def kabsch(source: np.ndarray, target: np.ndarray) -> np.ndarray:
    """``source @ R ≈ target`` 이 되는 회전 R (반사 없는 Kabsch)."""
    h = np.einsum("...pi,pj->...ij", source, target)
    u, _, vt = np.linalg.svd(h)
    d = np.sign(np.linalg.det(np.einsum("...ij,...jk->...ik", u, vt)))
    fix = np.ones(source.shape[:-2] + (3,))
    fix[..., 2] = np.where(d == 0.0, 1.0, d)
    return np.einsum("...ij,...j,...jk->...ik", u, fix, vt)


def canonical_points(
    points: np.ndarray, template: np.ndarray, size: float, mirror: bool = False
) -> np.ndarray:
    """21점을 손목 원점·템플릿 MCP 정렬·손 크기 size 로 맞춘다. 비유한 입력 프레임은 NaN."""
    rel = np.asarray(points, dtype=np.float64) - points[..., HAND21_WRIST, None, :]
    if mirror:
        rel = rel * MIRROR_X
    measured = hand_size(rel)
    scale = size / np.where(measured > MIN_AXIS_NORM, measured, np.nan)
    rotation = kabsch(np.nan_to_num(rel[..., _MCPS, :]), template)
    out = np.einsum("...pi,...ij->...pj", rel, rotation) * scale[..., None, None]
    return np.where(np.all(np.isfinite(rel), axis=(-2, -1), keepdims=True), out, np.nan)


def to_geort(canonical: np.ndarray) -> np.ndarray:
    """정준 21점을 GeoRT 입력 축으로 옮긴다."""
    return canonical @ OPENXR_TO_HAND21_AXES.T


@dataclasses.dataclass(frozen=True)
class InputFrame:
    """모델 입력 좌표계(템플릿 MCP 와 손 크기)."""
    name: str
    template: np.ndarray
    size: float

    def keypoints(self, joints: np.ndarray, mirror: bool = False) -> np.ndarray:
        """OpenXR 26관절 → 모델 입력 21점."""
        return to_geort(
            canonical_points(mediapipe21(joints), self.template, self.size, mirror=mirror)
        )
