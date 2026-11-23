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
    xr = np.asarray(joints, dtype=np.float64)
    return xr[..., _INDEX, POSITION]


def hand_size(points: np.ndarray) -> np.ndarray:
    rel = points[..., _MCPS, :] - points[..., HAND21_WRIST, None, :]
    return np.linalg.norm(rel, axis=-1).mean(axis=-1)


def kabsch(source: np.ndarray, target: np.ndarray) -> np.ndarray:
    h = np.einsum("...pi,pj->...ij", source, target)
    u, _, vt = np.linalg.svd(h)
    d = np.sign(np.linalg.det(np.einsum("...ij,...jk->...ik", u, vt)))
    fix = np.ones(source.shape[:-2] + (3,))
    fix[..., 2] = np.where(d == 0.0, 1.0, d)
    return np.einsum("...ij,...j,...jk->...ik", u, fix, vt)


def canonical_points(
    points: np.ndarray, template: np.ndarray, size: float, mirror: bool = False
) -> np.ndarray:
    rel = np.asarray(points, dtype=np.float64) - points[..., HAND21_WRIST, None, :]
    if mirror:
        rel = rel * MIRROR_X
    measured = hand_size(rel)
    scale = size / np.where(measured > MIN_AXIS_NORM, measured, np.nan)
    rotation = kabsch(np.nan_to_num(rel[..., _MCPS, :]), template)
    out = np.einsum("...pi,...ij->...pj", rel, rotation) * scale[..., None, None]
    return np.where(np.all(np.isfinite(rel), axis=(-2, -1), keepdims=True), out, np.nan)


def to_geort(canonical: np.ndarray) -> np.ndarray:
    return canonical @ OPENXR_TO_HAND21_AXES.T


@dataclasses.dataclass(frozen=True)
class InputFrame:
    name: str
    template: np.ndarray
    size: float

    def keypoints(self, joints: np.ndarray, mirror: bool = False) -> np.ndarray:
        return to_geort(
            canonical_points(mediapipe21(joints), self.template, self.size, mirror=mirror)
        )
