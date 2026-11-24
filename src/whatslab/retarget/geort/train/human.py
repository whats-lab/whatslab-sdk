from __future__ import annotations

import dataclasses
import os
from pathlib import Path

import numpy as np

from whatslab.core.constants import HAND21_WRIST

from ..preprocess import canonical_points

HUMAN_ENV = "WHATSLAB_GEORT_HUMAN"
FINGERS = ("thumb", "index", "middle", "ring", "little")
CHAINS = ((1, 2, 3, 4), (5, 6, 7, 8), (9, 10, 11, 12), (13, 14, 15, 16), (17, 18, 19, 20))
TIPS = tuple(chain[-1] for chain in CHAINS)
ANGLE_NAMES = (
    "thumb.mcp",
    "thumb.ip",
    *(f"{f}.{j}" for f in FINGERS[1:] for j in ("mcp", "pip", "dip")),
)
SOURCE_EGO = 2
BALANCE_VOXEL = 0.005
HINGE = np.array([-1.0, 0.0, 0.0])


@dataclasses.dataclass(frozen=True)
class HumanData:
    points: np.ndarray
    source: np.ndarray
    template: np.ndarray
    size: float


@dataclasses.dataclass(frozen=True)
class Augment:
    finger_scale: float = 0.08
    palm_width: float = 0.05
    noise_m: float = 0.0015
    dip_straight: float = 0.3
    thumb_ip_straight: float = 0.3
    jump_rate: float = 0.02
    jump_m: tuple[float, float] = (0.008, 0.02)


def human_path(path: Path | None) -> Path:
    raw = path or os.environ.get(HUMAN_ENV)
    if not raw:
        raise SystemExit(f"사람 손 데이터 경로가 없습니다: --human 또는 ${HUMAN_ENV}")
    return Path(raw).expanduser()


def load_human(path: Path) -> HumanData:
    with np.load(path, allow_pickle=False) as data:
        return HumanData(
            data["points"].astype(np.float64),
            np.asarray(data["source"]),
            np.asarray(data["template"], dtype=np.float64),
            float(data["size"]),
        )


def balance_weights(points: np.ndarray) -> np.ndarray:
    weights = np.zeros(len(points))
    for tip in TIPS:
        cells = np.floor(points[:, tip] / BALANCE_VOXEL).astype(np.int64)
        _, inverse, counts = np.unique(cells, axis=0, return_inverse=True, return_counts=True)
        weights += 1.0 / counts[inverse.ravel()]
    return weights / weights.sum()


def sample_frames(
    points: np.ndarray,
    source: np.ndarray,
    count: int,
    quest_share: float,
    rng: np.random.Generator,
) -> np.ndarray:
    quest = np.where(source != SOURCE_EGO)[0]
    ego = np.where(source == SOURCE_EGO)[0]
    n_quest = int(round(count * quest_share)) if len(ego) else count
    picks = [rng.choice(quest, size=n_quest, p=balance_weights(points[quest]))]
    if len(ego):
        picks.append(rng.choice(ego, size=count - n_quest, p=balance_weights(points[ego])))
    return np.concatenate(picks)


def signed_angle(parent: np.ndarray, child: np.ndarray, axis: np.ndarray) -> np.ndarray:
    p = parent - (parent @ axis)[..., None] * axis
    c = child - (child @ axis)[..., None] * axis
    return np.arctan2(np.cross(p, c) @ axis, np.sum(p * c, axis=-1))


def unsigned_angle(parent: np.ndarray, child: np.ndarray) -> np.ndarray:
    return np.arctan2(
        np.linalg.norm(np.cross(parent, child), axis=-1), np.sum(parent * child, axis=-1)
    )


def human_angles(canonical: np.ndarray) -> np.ndarray:
    c = CHAINS[0]
    bones = [canonical[..., c[k + 1], :] - canonical[..., c[k], :] for k in range(3)]
    out = [unsigned_angle(bones[0], bones[1]), unsigned_angle(bones[1], bones[2])]
    for chain in CHAINS[1:]:
        points = [canonical[..., HAND21_WRIST, :], *(canonical[..., i, :] for i in chain)]
        bones = [points[k + 1] - points[k] for k in range(4)]
        out += [signed_angle(bones[k], bones[k + 1], HINGE) for k in range(3)]
    return np.stack(out, axis=-1)


def straighten(points: np.ndarray, chain: tuple[int, ...], mask: np.ndarray) -> None:
    base, joint, tip = chain[1], chain[2], chain[3]
    direction = points[:, joint] - points[:, base]
    direction /= np.linalg.norm(direction, axis=-1, keepdims=True)
    length = np.linalg.norm(points[:, tip] - points[:, joint], axis=-1, keepdims=True)
    points[mask, tip] = (points[:, joint] + direction * length)[mask]


def augment(
    points: np.ndarray, rng: np.random.Generator, cfg: Augment, template: np.ndarray, size: float
) -> np.ndarray:
    out = points.copy()
    n = len(out)
    for chain in CHAINS:
        base = out[:, chain[0], None]
        scale = rng.uniform(1.0 - cfg.finger_scale, 1.0 + cfg.finger_scale, size=(n, 1, 1))
        out[:, chain[1:]] = base + (out[:, chain[1:]] - base) * scale
    out[:, :, 0] *= rng.uniform(1.0 - cfg.palm_width, 1.0 + cfg.palm_width, size=(n, 1))
    for chain in CHAINS[1:]:
        straighten(out, chain, rng.uniform(size=n) < cfg.dip_straight)
    straighten(out, CHAINS[0], rng.uniform(size=n) < cfg.thumb_ip_straight)
    out[:, 1:] += rng.normal(0.0, cfg.noise_m, size=out[:, 1:].shape)
    for i in np.where(rng.uniform(size=n) < cfg.jump_rate)[0]:
        chain = CHAINS[rng.integers(len(CHAINS))]
        offset = rng.normal(size=3)
        offset *= rng.uniform(*cfg.jump_m) / np.linalg.norm(offset)
        out[i, list(chain[1:])] += offset
    return canonical_points(out, template, size)
