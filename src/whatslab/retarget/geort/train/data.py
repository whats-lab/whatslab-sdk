from __future__ import annotations

import json
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import trimesh

from whatslab.paths import mesh_urdf, models_root

from ..preprocess import to_geort
from .hand import HandKinematics, read_hand, rpy_matrix, urdf_path
from .human import ANGLE_NAMES, TIPS, Augment, augment, human_angles, load_human, sample_frames

POSITIVE_SUFFIX = "+"
SURFACE_SAMPLES = 6000
COLLISION_SPACING = 0.005
COLLISION_MAX_POINTS = 160
PACKAGE_PREFIX = "package://dexhand_description/"
FILE_PREFIX = "file://"
MIDDLE = 2


def posture_targets(
    canonical: np.ndarray, spec: dict, kin: HandKinematics
) -> tuple[np.ndarray, np.ndarray]:
    angles = human_angles(canonical)
    prior = np.zeros((len(canonical), len(kin.joint_order)))
    mask = np.zeros(len(kin.joint_order), dtype=bool)
    for joint, terms in spec.items():
        i = kin.joint_order.index(joint)
        mask[i] = True
        for name, gain in terms.items():
            value = angles[:, ANGLE_NAMES.index(name.removesuffix(POSITIVE_SUFFIX))]
            if name.endswith(POSITIVE_SUFFIX):
                value = np.maximum(value, 0.0)
            prior[:, i] += float(gain) * value
        prior[:, i] = np.clip(prior[:, i], kin.lower[i], kin.upper[i])
    return prior, mask


def mesh_file(uri: str, base: Path) -> Path:
    if uri.startswith(PACKAGE_PREFIX):
        return Path(models_root()) / uri[len(PACKAGE_PREFIX) :]
    return base / uri.removeprefix(FILE_PREFIX)


def surface_points(triangles: np.ndarray, rng: np.random.Generator) -> np.ndarray:
    a, b, c = triangles[:, 0], triangles[:, 1], triangles[:, 2]
    area = 0.5 * np.linalg.norm(np.cross(b - a, c - a), axis=-1)
    pick = rng.choice(len(triangles), size=SURFACE_SAMPLES, p=area / area.sum())
    u, v = rng.uniform(size=(2, SURFACE_SAMPLES, 1))
    flip = (u + v) > 1.0
    u, v = np.where(flip, 1.0 - u, u), np.where(flip, 1.0 - v, v)
    points = a[pick] + u * (b[pick] - a[pick]) + v * (c[pick] - a[pick])
    cells = np.floor(points / COLLISION_SPACING).astype(np.int64)
    _, first = np.unique(cells, axis=0, return_index=True)
    points = points[np.sort(first)]
    if len(points) > COLLISION_MAX_POINTS:
        points = points[rng.choice(len(points), size=COLLISION_MAX_POINTS, replace=False)]
    return points


def link_points(root: ET.Element, link: str, base: Path, rng: np.random.Generator) -> np.ndarray:
    element = root.find(f"link[@name='{link}']")
    parts = [p for p in element.findall("collision") if p.find("geometry/mesh") is not None]
    parts = parts or [p for p in element.findall("visual") if p.find("geometry/mesh") is not None]
    if not parts:
        raise ValueError(f"충돌 링크 '{link}' 에 메시가 없습니다")
    out = []
    for part in parts:
        mesh = part.find("geometry/mesh")
        loaded = trimesh.load(mesh_file(mesh.get("filename", ""), base), force="mesh")
        triangles = np.asarray(loaded.vertices, dtype=np.float64)[np.asarray(loaded.faces)]
        scale = np.array([float(v) for v in (mesh.get("scale") or "1 1 1").split()])
        triangles = triangles * scale
        origin = part.find("origin")
        attrs = {} if origin is None else origin.attrib
        xyz = np.array([float(v) for v in (attrs.get("xyz") or "0 0 0").split()])
        rpy = np.array([float(v) for v in (attrs.get("rpy") or "0 0 0").split()])
        out.append(surface_points(triangles @ rpy_matrix(rpy).T + xyz, rng))
    return np.concatenate(out)


def collision_points(
    kin: HandKinematics, spec: dict, base: Path, seed: int
) -> tuple[dict, dict[str, np.ndarray]]:
    rng = np.random.default_rng(seed)
    pairs = [[list(a), list(b)] for a, b in spec["pairs"]]
    links = sorted({name for pair in pairs for group in pair for name in group})
    reachable = set(kin.fixed) | {s.child for f in kin.fingers for s in f.steps}
    missing = [n for n in links if n not in reachable]
    if missing:
        raise ValueError(f"충돌 링크가 손바닥·손가락 사슬에 없습니다: {missing}")
    arrays = {name: link_points(kin.root, name, base, rng) for name in links}
    meta = {"links": links, "pairs": pairs, "margin_m": float(spec.get("margin_mm", 4.0)) * 1e-3}
    return meta, arrays


def mcp_links(kin: HandKinematics, posture: dict) -> list[str]:
    out = []
    for finger in kin.fingers:
        moving = [s for s in finger.steps if s.axis is not None]
        mcp = [s for s in moving if f"{finger.name}.mcp" in (posture.get(s.joint) or {})]
        out.append((mcp or moving[1:] or moving)[0].child)
    return out


def robot_scale(kin: HandKinematics, canonical: np.ndarray) -> float:
    middle = [f.name for f in kin.fingers].index("middle")
    robot = float(np.linalg.norm(kin.tips(kin.zero())[middle]))
    human = float(np.percentile(np.linalg.norm(canonical[:, TIPS[MIDDLE]], axis=-1), 95))
    return robot / human


def hand_entry(
    hand: str, clean: np.ndarray, arrays: dict[str, np.ndarray], seed: int
) -> dict[str, object]:
    cfg = read_hand(hand)
    urdf = urdf_path(cfg)
    kin = HandKinematics(cfg, urdf)
    name = cfg["name"]
    posture = cfg.get("posture") or {}
    prior, mask = posture_targets(clean, posture, kin)
    arrays[f"prior_{name}"] = prior.astype(np.float32)
    arrays[f"mask_{name}"] = mask
    collision = None
    if cfg.get("collision"):
        base = Path(mesh_urdf(str(urdf))).resolve().parent
        collision, points = collision_points(kin, cfg["collision"], base, seed)
        arrays.update({f"coll_{name}_{k}": v for k, v in points.items()})
    return {
        "name": name,
        "urdf": cfg["urdf"],
        "kinematics": kin.to_dict(),
        "collision": collision,
        "scale": float(cfg.get("scale") or robot_scale(kin, clean)),
        "mcp_links": mcp_links(kin, posture),
        "tip_scale": float(cfg.get("tip_scale", 1.0)),
        "abd_scale": float(cfg.get("abd_scale", 1.0)),
    }


def build_data(
    hands: list[str], human: Path, out: Path, frames: int, quest_share: float, seed: int
) -> dict:
    rng = np.random.default_rng(seed)
    data = load_human(human)
    clean = data.points[sample_frames(data.points, data.source, frames, quest_share, rng)]
    noisy = augment(clean, rng, Augment(), data.template, data.size)
    ok = np.all(np.isfinite(noisy), axis=(1, 2)) & np.all(np.isfinite(clean), axis=(1, 2))
    clean, noisy = clean[ok], noisy[ok]
    arrays: dict[str, np.ndarray] = {
        "clean": to_geort(clean).astype(np.float32),
        "noisy": to_geort(noisy).astype(np.float32),
        "template": data.template,
        "size": np.array(data.size),
    }
    spec = {
        "human": {"frames": int(len(clean)), "quest_share": quest_share, "seed": seed},
        "hands": [hand_entry(h, clean, arrays, seed) for h in hands],
    }
    arrays["spec_json"] = np.array(json.dumps(spec))
    np.savez(out, **arrays)
    return spec
