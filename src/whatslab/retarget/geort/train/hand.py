from __future__ import annotations

import dataclasses
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import yaml

from whatslab.paths import configs_root, models_root

from .human import FINGERS

CONFIG_DIR = "geort_train"
MOVING = ("revolute", "continuous")
PALM_POINTS = ("index", "middle", "ring")
REQUIRED_FINGERS = ("thumb", "middle")


@dataclasses.dataclass(frozen=True)
class Joint:
    name: str
    kind: str
    parent: str
    child: str
    origin: np.ndarray
    axis: np.ndarray
    mimic: tuple[str, float, float] | None
    lower: float
    upper: float


@dataclasses.dataclass(frozen=True)
class Step:
    joint: str
    child: str
    origin: np.ndarray
    axis: np.ndarray | None
    index: int
    scale: float
    bias: float


@dataclasses.dataclass(frozen=True)
class Finger:
    name: str
    steps: tuple[Step, ...]
    offset: np.ndarray


def config_file(hand: str) -> Path:
    if hand.endswith((".yaml", ".yml")):
        return Path(hand).expanduser()
    return Path(configs_root()) / CONFIG_DIR / f"{hand}.yaml"


def config_names() -> list[str]:
    return sorted(p.stem for p in (Path(configs_root()) / CONFIG_DIR).glob("*.yaml"))


def read_hand(hand: str) -> dict:
    path = config_file(hand)
    cfg = yaml.safe_load(path.read_text(encoding="utf-8"))
    names = [f["name"] for f in cfg["fingers"]]
    unknown = sorted(set(names) - set(FINGERS))
    missing = [n for n in REQUIRED_FINGERS if n not in names]
    palm = [k for k in PALM_POINTS if k in cfg["palm"]]
    if unknown or missing or len(set(names)) != len(names) or len(palm) < 2:
        raise ValueError(
            f"{path}: fingers 이름은 {FINGERS} 중 서로 다르게, {REQUIRED_FINGERS} 는 반드시, "
            f"palm 에는 {PALM_POINTS} 중 둘 이상이 있어야 합니다"
        )
    return cfg


def urdf_path(cfg: dict) -> Path:
    path = Path(cfg["urdf"]).expanduser()
    return path if path.is_absolute() else Path(models_root()) / path


def floats(text: str | None, default: str) -> np.ndarray:
    return np.array([float(v) for v in (text or default).split()])


def rpy_matrix(rpy: np.ndarray) -> np.ndarray:
    r, p, y = rpy
    rx = np.array([[1, 0, 0], [0, math.cos(r), -math.sin(r)], [0, math.sin(r), math.cos(r)]])
    ry = np.array([[math.cos(p), 0, math.sin(p)], [0, 1, 0], [-math.sin(p), 0, math.cos(p)]])
    rz = np.array([[math.cos(y), -math.sin(y), 0], [math.sin(y), math.cos(y), 0], [0, 0, 1]])
    return rz @ ry @ rx


def transform(xyz: np.ndarray, rpy: np.ndarray) -> np.ndarray:
    t = np.eye(4)
    t[:3, :3] = rpy_matrix(rpy)
    t[:3, 3] = xyz
    return t


def axis_rotation(axis: np.ndarray, angle: np.ndarray) -> np.ndarray:
    a = np.asarray(axis, dtype=np.float64)
    a = a / np.linalg.norm(a)
    k = np.array([[0.0, -a[2], a[1]], [a[2], 0.0, -a[0]], [-a[1], a[0], 0.0]])
    t = np.asarray(angle, dtype=np.float64)[..., None, None]
    return np.eye(3) + np.sin(t) * k + (1.0 - np.cos(t)) * (k @ k)


def read_joints(root: ET.Element) -> list[Joint]:
    out = []
    for j in root.findall("joint"):
        origin = j.find("origin")
        attrs = {} if origin is None else origin.attrib
        axis = j.find("axis")
        m = j.find("mimic")
        limit = j.find("limit")
        mimic = None
        if m is not None:
            mimic = (
                m.get("joint", ""),
                float(m.get("multiplier", "1")),
                float(m.get("offset", "0")),
            )
        lower = -math.inf if limit is None else float(limit.get("lower", "-inf"))
        upper = math.inf if limit is None else float(limit.get("upper", "inf"))
        out.append(
            Joint(
                j.get("name", ""),
                j.get("type", ""),
                j.find("parent").get("link", ""),
                j.find("child").get("link", ""),
                transform(floats(attrs.get("xyz"), "0 0 0"), floats(attrs.get("rpy"), "0 0 0")),
                floats(None if axis is None else axis.get("xyz"), "1 0 0"),
                mimic,
                lower,
                upper,
            )
        )
    return out


def palm_axes(wrist: np.ndarray, index: np.ndarray, middle: np.ndarray, ring: np.ndarray):
    z = middle - wrist
    z = z / np.linalg.norm(z)
    across = index - ring
    across = across / np.linalg.norm(across)
    x = np.cross(across, z)
    x = x / np.linalg.norm(x)
    return np.column_stack([x, np.cross(z, x), z])


class HandKinematics:
    def __init__(self, cfg: dict, urdf: Path) -> None:
        self.root = ET.parse(urdf).getroot()
        joints = read_joints(self.root)
        self.wrist = cfg["wrist"]["link"]
        self.joint_order = tuple(j for f in cfg["fingers"] for j in f["joints"])
        by_name = {j.name: j for j in joints}
        unknown = [n for n in self.joint_order if n not in by_name]
        if unknown:
            raise ValueError(f"URDF 에 없는 관절: {unknown}")
        self.sources = {j.name: self._source(j, by_name) for j in joints}
        self.lower, self.upper = self._limits(by_name, cfg.get("limits_deg") or {})
        frames = self._zero_frames(joints)
        self.palm = self._palm(cfg, frames)
        by_child = {j.child: j for j in joints}
        self.fingers = tuple(self._chain(by_child, f) for f in cfg["fingers"])
        self.fixed = self._fixed(joints)

    def _source(self, joint: Joint, by_name: dict[str, Joint]) -> tuple[int, float, float] | None:
        if joint.kind not in MOVING:
            return None
        current, scale, bias = joint, 1.0, 0.0
        for _ in by_name:
            if current.name in self.joint_order:
                return self.joint_order.index(current.name), scale, bias
            if current.mimic is None or current.mimic[0] not in by_name:
                return None
            name, multiplier, offset = current.mimic
            current, scale, bias = by_name[name], scale * multiplier, scale * offset + bias
        return None

    def _limits(self, by_name: dict[str, Joint], wanted: dict) -> tuple[np.ndarray, np.ndarray]:
        lower, upper = [], []
        for name in self.joint_order:
            lo, hi = by_name[name].lower, by_name[name].upper
            if name in wanted:
                lo = max(lo, math.radians(wanted[name][0]))
                hi = min(hi, math.radians(wanted[name][1]))
            if not (math.isfinite(lo) and math.isfinite(hi) and lo < hi):
                raise ValueError(f"관절 '{name}' 의 한계가 비었거나 없습니다 (limits_deg)")
            lower.append(lo)
            upper.append(hi)
        return np.array(lower), np.array(upper)

    def _zero_frames(self, joints: list[Joint]) -> dict[str, np.ndarray]:
        frames = {self.wrist: np.eye(4)}
        grew = True
        while grew:
            grew = False
            for j in joints:
                if j.parent in frames and j.child not in frames:
                    frames[j.child] = frames[j.parent] @ j.origin
                    grew = True
        return frames

    def _palm(self, cfg: dict, frames: dict[str, np.ndarray]) -> np.ndarray:
        offset = np.array(cfg["wrist"].get("offset", [0.0, 0.0, 0.0]), dtype=float)
        shift = np.array(cfg["palm"].get("shift", [0.0, 0.0, 0.0]), dtype=float)
        points = {}
        for key in PALM_POINTS:
            entry = cfg["palm"].get(key)
            if entry is None:
                continue
            entry = {"link": entry} if isinstance(entry, str) else entry
            t = frames[entry["link"]]
            local = np.array(entry.get("offset", [0.0, 0.0, 0.0]), dtype=float)
            points[key] = t[:3, :3] @ local + t[:3, 3] + shift
        if "middle" not in points:
            points["middle"] = 0.5 * (points["index"] + points["ring"])
        if "ring" not in points:
            points["ring"] = 2.0 * points["middle"] - points["index"]
        if "index" not in points:
            points["index"] = 2.0 * points["middle"] - points["ring"]
        palm = np.eye(4)
        palm[:3, :3] = palm_axes(offset, points["index"], points["middle"], points["ring"])
        palm[:3, 3] = offset
        return palm

    def _chain(self, by_child: dict[str, Joint], finger: dict) -> Finger:
        link = finger["link"]
        path = []
        while link != self.wrist:
            joint = by_child.get(link)
            if joint is None:
                raise ValueError(f"{finger['link']} 가 {self.wrist} 아래에 없습니다")
            path.append(joint)
            link = joint.parent
        steps = []
        for joint in reversed(path):
            source = self.sources[joint.name]
            index, scale, bias = source or (-1, 1.0, 0.0)
            axis = None if source is None else joint.axis / np.linalg.norm(joint.axis)
            steps.append(Step(joint.name, joint.child, joint.origin, axis, index, scale, bias))
        offset = np.array(finger.get("offset", [0.0, 0.0, 0.0]), dtype=float)
        return Finger(finger["name"], tuple(steps), offset)

    def _fixed(self, joints: list[Joint]) -> dict[str, np.ndarray]:
        out = {self.wrist: np.eye(4)}
        grew = True
        while grew:
            grew = False
            for j in joints:
                if self.sources[j.name] is None and j.parent in out and j.child not in out:
                    out[j.child] = out[j.parent] @ j.origin
                    grew = True
        return out

    def link_frames(self, q: np.ndarray) -> dict[str, np.ndarray]:
        q = np.asarray(q, dtype=np.float64)
        base = np.broadcast_to(np.linalg.inv(self.palm), q.shape[:-1] + (4, 4))
        out = {name: base @ t for name, t in self.fixed.items()}
        for finger in self.fingers:
            t = base
            for step in finger.steps:
                t = t @ step.origin
                if step.axis is not None:
                    r = np.zeros(q.shape[:-1] + (4, 4))
                    angle = q[..., step.index] * step.scale + step.bias
                    r[..., :3, :3] = axis_rotation(step.axis, angle)
                    r[..., 3, 3] = 1.0
                    t = t @ r
                out[step.child] = t
        return out

    def tips(self, q: np.ndarray) -> np.ndarray:
        frames = self.link_frames(q)
        out = []
        for finger in self.fingers:
            t = frames[finger.steps[-1].child]
            out.append(t[..., :3, :3] @ finger.offset + t[..., :3, 3])
        return np.stack(out, axis=-2)

    def zero(self) -> np.ndarray:
        return np.clip(np.zeros(len(self.joint_order)), self.lower, self.upper)

    def to_dict(self) -> dict:
        return {
            "joint_order": list(self.joint_order),
            "lower": self.lower.tolist(),
            "upper": self.upper.tolist(),
            "palm": self.palm.tolist(),
            "fixed": {name: t.tolist() for name, t in self.fixed.items()},
            "fingers": [
                {
                    "name": f.name,
                    "offset": f.offset.tolist(),
                    "steps": [
                        {
                            "joint": s.joint,
                            "child": s.child,
                            "origin": s.origin.tolist(),
                            "axis": None if s.axis is None else s.axis.tolist(),
                            "index": s.index,
                            "scale": s.scale,
                            "bias": s.bias,
                        }
                        for s in f.steps
                    ],
                }
                for f in self.fingers
            ],
        }


def urdf_tips(kin: HandKinematics, q: np.ndarray) -> np.ndarray:
    pending = read_joints(kin.root)
    by_name = {j.name: j for j in pending}
    driven = dict(zip(kin.joint_order, q))

    def value(name: str) -> float:
        mimic = by_name[name].mimic
        if name in driven or mimic is None:
            return driven.get(name, 0.0)
        return value(mimic[0]) * mimic[1] + mimic[2]

    frames = {kin.wrist: np.linalg.inv(kin.palm)}
    grew = True
    while grew:
        grew = False
        for j in pending:
            if j.parent in frames and j.child not in frames:
                t = frames[j.parent] @ j.origin
                if j.kind in MOVING:
                    r = np.eye(4)
                    r[:3, :3] = axis_rotation(j.axis, value(j.name))
                    t = t @ r
                frames[j.child] = t
                grew = True
    tips = [frames[f.steps[-1].child] for f in kin.fingers]
    return np.stack([t[:3, :3] @ f.offset + t[:3, 3] for t, f in zip(tips, kin.fingers)])
