from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, Optional, Sequence, Tuple

import numpy as np
from scipy.spatial.transform import Rotation

from whatslab.kinematics import Kinematics

from .preprocess import palm_frame

POSTURE_KEY = "posture_prior"
POSITIVE_SUFFIX = "+"
FORWARD = np.array([0.0, 0.0, -1.0])
FINGER_CHAINS = {"index": (6, 7, 8, 9), "middle": (11, 12, 13, 14), "ring": (16, 17, 18, 19),
                 "little": (21, 22, 23, 24)}
THUMB_CHAIN = (2, 3, 4)
PALM_POINTS = (1, 7, 12, 22)
BEND_PAIRS = ((THUMB_CHAIN[0], THUMB_CHAIN[1]), (THUMB_CHAIN[1], THUMB_CHAIN[2]),
              *((c[k], c[k + 1]) for c in FINGER_CHAINS.values() for k in range(3)))
BEND_NAMES = ("thumb.mcp", "thumb.ip",
              *(f"{f}.{j}" for f in FINGER_CHAINS for j in ("mcp", "pip", "dip")))
SPREAD_PAIRS = tuple((c[0], c[1]) for c in FINGER_CHAINS.values())
SPREAD_NAMES = tuple(f"{f}.spread" for f in FINGER_CHAINS)
FEATURES = (*BEND_NAMES, *SPREAD_NAMES)
PARENTS = np.array([p for p, _ in BEND_PAIRS + SPREAD_PAIRS])
CHILDREN = np.array([c for _, c in BEND_PAIRS + SPREAD_PAIRS])
SPREAD_START = len(BEND_NAMES)
OPEN_FINGERS = ("index", "middle", "ring", "little")
FINGER_BENDS = {"thumb": ("thumb.mcp", "thumb.ip"),
                **{f: tuple(f"{f}.{j}" for j in ("mcp", "pip", "dip")) for f in FINGER_CHAINS}}
_SPEC_KEYS = frozenset({"weight", "joints", "thumb_base", "joint_weights", "open_hand",
                        "abduction"})
_THUMB_KEYS = frozenset({"joints", "bone", "flex_joint", "step_deg", "axis_weight"})
_OPEN_KEYS = frozenset({"open_below_deg", "free_above_deg", "weight", "fingers"})
_ABDUCTION_KEYS = frozenset({"full_deg", "decay", "gain"})


@dataclass(frozen=True)
class ThumbBaseSpec:
    joints: Tuple[str, str]
    bone: Tuple[str, str]
    flex_joint: str
    step_deg: float = 2.0
    axis_weight: float = 0.5


@dataclass(frozen=True)
class OpenHandSpec:
    open_below_deg: float
    free_above_deg: float
    weight: float
    fingers: Tuple[str, ...] = OPEN_FINGERS


@dataclass(frozen=True)
class AbductionSpec:
    full_deg: float = 60.0
    decay: float = 1.0
    gain: float = 10.0


@dataclass(frozen=True)
class PostureSpec:
    weight: float
    joints: Dict[str, Dict[str, float]]
    thumb_base: Optional[ThumbBaseSpec] = None
    joint_weights: Dict[str, float] = field(default_factory=dict)
    open_hand: Optional[OpenHandSpec] = None
    abduction: Optional[AbductionSpec] = None

    @property
    def guided_joints(self) -> Tuple[str, ...]:
        thumb = self.thumb_base.joints if self.thumb_base else ()
        return (*self.joints, *thumb)


def _mapping(raw, where: str) -> dict:
    if not isinstance(raw, dict):
        raise ValueError(f"{where}: 키-값 목록이어야 한다 — 받은 값 {raw!r}")
    return raw


def _keys(raw: dict, allowed: frozenset, required: Sequence[str], where: str) -> None:
    unknown = set(raw) - allowed
    if unknown:
        raise ValueError(f"{where}: 모르는 키 {sorted(unknown)}")
    missing = [k for k in required if k not in raw]
    if missing:
        raise ValueError(f"{where}: 필요한 키가 없다 {missing}")


def _positive(value, where: str) -> float:
    v = float(value)
    if not v > 0.0:
        raise ValueError(f"{where}: 0 보다 커야 한다 — 받은 값 {value!r}")
    return v


def _pair(raw, where: str) -> Tuple[str, str]:
    if not isinstance(raw, (list, tuple)) or len(raw) != 2:
        raise ValueError(f"{where}: 이름 두 개여야 한다 — 받은 값 {raw!r}")
    return str(raw[0]), str(raw[1])


def _thumb_spec(raw) -> ThumbBaseSpec:
    where = f"{POSTURE_KEY}.thumb_base"
    raw = _mapping(raw, where)
    _keys(raw, _THUMB_KEYS, ("joints", "bone", "flex_joint"), where)
    step = _positive(raw.get("step_deg", 2.0), f"{where}.step_deg")
    if step > 10.0:
        raise ValueError(f"{where}.step_deg: 10 이하여야 한다 — 받은 값 {step}")
    axis_weight = float(raw.get("axis_weight", 0.5))
    if axis_weight < 0.0:
        raise ValueError(f"{where}.axis_weight: 0 이상이어야 한다 — 받은 값 {axis_weight}")
    return ThumbBaseSpec(_pair(raw["joints"], f"{where}.joints"),
                         _pair(raw["bone"], f"{where}.bone"),
                         str(raw["flex_joint"]), step, axis_weight)


def _open_spec(raw) -> OpenHandSpec:
    where = f"{POSTURE_KEY}.open_hand"
    raw = _mapping(raw, where)
    _keys(raw, _OPEN_KEYS, ("open_below_deg", "free_above_deg", "weight"), where)
    below = _positive(raw["open_below_deg"], f"{where}.open_below_deg")
    above = _positive(raw["free_above_deg"], f"{where}.free_above_deg")
    if above <= below:
        raise ValueError(f"{where}: free_above_deg 는 open_below_deg 보다 커야 한다")
    fingers = tuple(str(f) for f in raw.get("fingers", OPEN_FINGERS))
    unknown = [f for f in fingers if f not in FINGER_BENDS]
    if unknown:
        raise ValueError(f"{where}.fingers: 모르는 손가락 {unknown} (가능: {list(FINGER_BENDS)})")
    return OpenHandSpec(below, above, _positive(raw["weight"], f"{where}.weight"), fingers)


def _non_negative(value, where: str) -> float:
    v = float(value)
    if not v >= 0.0:
        raise ValueError(f"{where}: 0 이상이어야 한다 — 받은 값 {value!r}")
    return v


def _abduction_spec(raw) -> AbductionSpec:
    where = f"{POSTURE_KEY}.abduction"
    raw = _mapping(raw, where)
    _keys(raw, _ABDUCTION_KEYS, (), where)
    base = AbductionSpec()
    return AbductionSpec(_positive(raw.get("full_deg", base.full_deg), f"{where}.full_deg"),
                         _non_negative(raw.get("decay", base.decay), f"{where}.decay"),
                         _non_negative(raw.get("gain", base.gain), f"{where}.gain"))


def parse_posture(raw) -> Optional[PostureSpec]:
    if raw is None:
        return None
    raw = _mapping(raw, POSTURE_KEY)
    _keys(raw, _SPEC_KEYS, ("weight", "joints"), POSTURE_KEY)
    joints_raw = _mapping(raw["joints"], f"{POSTURE_KEY}.joints")
    if not joints_raw:
        raise ValueError(f"{POSTURE_KEY}.joints: 비었다")
    joints = {}
    for joint, terms in joints_raw.items():
        terms = _mapping(terms, f"{POSTURE_KEY}.joints.{joint}")
        if not terms:
            raise ValueError(f"{POSTURE_KEY}.joints.{joint}: 사람 손 각도 항목이 비었다")
        bad = [n for n in terms if str(n).removesuffix(POSITIVE_SUFFIX) not in FEATURES]
        if bad:
            raise ValueError(f"{POSTURE_KEY}.joints.{joint}: 모르는 사람 손 각도 {bad} "
                             f"(가능: {', '.join(FEATURES)}, 끝에 + 를 붙이면 양수 부분만)")
        joints[str(joint)] = {str(n): float(g) for n, g in terms.items()}
    thumb = None if raw.get("thumb_base") is None else _thumb_spec(raw["thumb_base"])
    if thumb is not None:
        shared = set(thumb.joints) & set(joints)
        if shared:
            raise ValueError(f"{POSTURE_KEY}: thumb_base 관절이 joints 에도 있다 {sorted(shared)}")
    guided = set(joints) | set(thumb.joints if thumb else ())
    weights_raw = _mapping(raw.get("joint_weights") or {}, f"{POSTURE_KEY}.joint_weights")
    joint_weights = {}
    for joint, value in weights_raw.items():
        if joint not in guided:
            raise ValueError(f"{POSTURE_KEY}.joint_weights: '{joint}' 는 joints 나 thumb_base 에 "
                             "없는 관절이다")
        joint_weights[str(joint)] = _positive(value, f"{POSTURE_KEY}.joint_weights.{joint}")
    open_hand = None if raw.get("open_hand") is None else _open_spec(raw["open_hand"])
    abduction = None if raw.get("abduction") is None else _abduction_spec(raw["abduction"])
    return PostureSpec(_positive(raw["weight"], f"{POSTURE_KEY}.weight"), joints, thumb,
                       joint_weights, open_hand, abduction)


def human_angles(xr: np.ndarray, side: str = "right") -> np.ndarray:
    rot = Rotation.from_quat(xr[:, :4]).as_matrix()
    d = np.einsum("nji,njk,k->ni", rot[PARENTS], rot[CHILDREN], FORWARD)
    bend = np.arctan2(-d[:SPREAD_START, 1], -d[:SPREAD_START, 2])
    spread = np.arctan2(d[SPREAD_START:, 0], np.hypot(d[SPREAD_START:, 1], d[SPREAD_START:, 2]))
    return np.concatenate([bend, spread if side == "right" else -spread])


def human_thumb_base(xr: np.ndarray, side: str) -> Optional[Tuple[np.ndarray, np.ndarray]]:
    pos = xr[:, 4:7]
    palm = palm_frame(*(pos[i] for i in PALM_POINTS), side=side)
    bone = pos[THUMB_CHAIN[1]] - pos[THUMB_CHAIN[0]]
    norm = float(np.linalg.norm(bone))
    if palm is None or not norm > 1e-9:
        return None
    axis = -Rotation.from_quat(xr[THUMB_CHAIN[0], :4]).as_matrix()[:, 0]
    return palm.T @ (bone / norm), palm.T @ axis


@dataclass(frozen=True)
class ThumbBaseTable:
    index: Tuple[int, int]
    grid: np.ndarray
    directions: np.ndarray
    axes: np.ndarray
    axis_weight: float

    def solve(self, direction: np.ndarray, axis: np.ndarray) -> np.ndarray:
        score = self.directions @ direction + self.axis_weight * (self.axes @ axis)
        return self.grid[int(np.argmax(score))]


def thumb_base_table(spec: ThumbBaseSpec, kin: Kinematics, full_qpos, names: Sequence[str],
                     limits: np.ndarray, q0: np.ndarray, palm: np.ndarray,
                     flex_sign: float = 1.0) -> ThumbBaseTable:
    where = f"{POSTURE_KEY}.thumb_base.flex_joint"
    bone_ids = kin.frame_ids(list(spec.bone))
    if spec.flex_joint not in kin.joint_names:
        raise ValueError(f"{where}: URDF 에 없는 관절 {spec.flex_joint}")
    flex = kin.joint_names.index(spec.flex_joint)
    if kin.joint_prismatic[flex]:
        raise ValueError(f"{where}: 회전 관절이 아니다 {spec.flex_joint}")
    local_axis = kin.joint_axes[flex] * flex_sign
    index = tuple(list(names).index(j) for j in spec.joints)
    deg = np.degrees(limits)
    ranges = [np.radians(np.arange(deg[i, 0], deg[i, 1] + 1e-9, spec.step_deg)) for i in index]
    grid = np.array([(a, b) for a in ranges[0] for b in ranges[1]])
    directions, axes = [], []
    for a, b in grid:
        x = q0.copy()
        x[index[0]], x[index[1]] = a, b
        qpos = full_qpos(x)
        world = kin.joint_frames(qpos)
        tf = kin.link_placements(qpos)
        bone = tf[bone_ids[1], :3, 3] - tf[bone_ids[0], :3, 3]
        directions.append(palm.T @ (bone / np.linalg.norm(bone)))
        axes.append(palm.T @ (world[flex, :3, :3] @ local_axis))
    return ThumbBaseTable((index[0], index[1]), grid, np.array(directions), np.array(axes),
                          spec.axis_weight)


@dataclass(frozen=True)
class OpenHandTable:
    rows: np.ndarray
    fingers: np.ndarray
    open_below: float
    free_above: float
    weight: float

    def share(self, angles: np.ndarray) -> np.ndarray:
        bend = self.fingers @ np.maximum(angles, 0.0)
        return np.clip((self.free_above - bend) / (self.free_above - self.open_below), 0.0, 1.0)


def open_hand_table(spec: OpenHandSpec, joints: Dict[str, Dict[str, float]],
                    names: Sequence[str]) -> OpenHandTable:
    rows, fingers = [], []
    for joint, terms in joints.items():
        features = {n.removesuffix(POSITIVE_SUFFIX) for n in terms}
        owner = [f for f in spec.fingers if features <= set(FINGER_BENDS[f])]
        if owner:
            rows.append(list(names).index(joint))
            fingers.append([float(n in FINGER_BENDS[owner[0]]) for n in FEATURES])
    if not rows:
        raise ValueError(f"{POSTURE_KEY}.open_hand: 손가락 {list(spec.fingers)} 의 굽힘 관절이 "
                         "joints 에 없다")
    return OpenHandTable(np.array(rows), np.array(fingers), float(np.radians(spec.open_below_deg)),
                         float(np.radians(spec.free_above_deg)), spec.weight)


def spread_joints(joints: Dict[str, Dict[str, float]]) -> Dict[str, str]:
    out = {}
    for joint, terms in joints.items():
        fingers = {n.removesuffix(POSITIVE_SUFFIX).split(".")[0] for n in terms}
        if len(fingers) == 1 and all(n.removesuffix(POSITIVE_SUFFIX) in SPREAD_NAMES
                                     for n in terms):
            out[joint] = fingers.pop()
    return out


@dataclass(frozen=True)
class AbductionTable:
    rows: np.ndarray
    bends: np.ndarray
    full: float
    decay: float
    gain: float

    def flex(self, angles: np.ndarray) -> np.ndarray:
        return np.clip(np.maximum(angles[self.bends], 0.0) / self.full, 0.0, 1.0)


def abduction_table(spec: AbductionSpec, joints: Dict[str, Dict[str, float]],
                    names: Sequence[str]) -> Optional[AbductionTable]:
    spread = spread_joints(joints)
    if not spread:
        return None
    return AbductionTable(np.array([list(names).index(j) for j in spread]),
                          np.array([FEATURES.index(f"{f}.mcp") for f in spread.values()]),
                          float(np.radians(spec.full_deg)), spec.decay, spec.gain)


class PostureGuide:
    def __init__(self, spec: PostureSpec, kin: Kinematics, full_qpos, names: Sequence[str],
                 limits: np.ndarray, palm: np.ndarray, side: str):
        names = list(names)
        missing = [j for j in spec.guided_joints if j not in names]
        if missing:
            raise ValueError(f"{POSTURE_KEY}: 구동 관절(target_joint_names)에 없는 관절 {missing}")
        self.spec = spec
        self.side = side
        self._limits = np.asarray(limits, dtype=float)
        n = len(names)
        self._gain = np.zeros((n, len(FEATURES)))
        self._positive_gain = np.zeros((n, len(FEATURES)))
        weight = np.zeros(n)
        for joint, terms in spec.joints.items():
            row = names.index(joint)
            weight[row] = spec.weight
            for name, gain in terms.items():
                target = self._positive_gain if name.endswith(POSITIVE_SUFFIX) else self._gain
                target[row, FEATURES.index(name.removesuffix(POSITIVE_SUFFIX))] += gain
        self._open_pose = np.clip(np.zeros(n), self._limits[:, 0], self._limits[:, 1])
        self.thumb = None
        if spec.thumb_base is not None:
            gains = spec.joints.get(spec.thumb_base.flex_joint, {}).values()
            self.thumb = thumb_base_table(spec.thumb_base, kin, full_qpos, names, self._limits,
                                          self._open_pose, palm, -1.0 if sum(gains) < 0 else 1.0)
            weight[list(self.thumb.index)] = spec.weight
        for joint, value in spec.joint_weights.items():
            weight[names.index(joint)] = value
        self.weight = weight
        self.open = (None if spec.open_hand is None
                     else open_hand_table(spec.open_hand, spec.joints, names))
        self.abduction = (None if spec.abduction is None
                          else abduction_table(spec.abduction, spec.joints, names))

    def target(self, xr: np.ndarray) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        angles = human_angles(xr, self.side)
        q = self._gain @ angles + self._positive_gain @ np.maximum(angles, 0.0)
        if self.thumb is not None:
            base = human_thumb_base(xr, self.side)
            if base is None:
                return None
            q[list(self.thumb.index)] = self.thumb.solve(*base)
        weight = self.weight.copy()
        if self.abduction is not None:
            rows = self.abduction.rows
            f = self.abduction.flex(angles)
            rest = self._open_pose[rows]
            q[rows] = rest + (q[rows] - rest) * (1.0 - f) ** self.abduction.decay
            weight[rows] *= 1.0 + self.abduction.gain * f
        if self.open is not None:
            rows = self.open.rows
            pull = self.open.share(angles) * self.open.weight
            pulled = weight[rows] * q[rows] + pull * self._open_pose[rows]
            q[rows] = pulled / (weight[rows] + pull)
            weight[rows] += pull
        q = np.clip(q, self._limits[:, 0], self._limits[:, 1])
        if not (np.all(np.isfinite(q)) and np.all(np.isfinite(weight))):
            return None
        return q, weight
