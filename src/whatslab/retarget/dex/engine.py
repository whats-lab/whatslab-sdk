from __future__ import annotations

import fnmatch
import os
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np
import yaml
from scipy.spatial.transform import Rotation

from whatslab.core.constants import SIDES
from whatslab.kinematics import Kinematics
from whatslab.paths import configs_root, models_root

from ..retargeter import DEX_PREFIX
from .optimizer import RetargetOptimizer, VectorTerm, dexpilot_pairs
from .preprocess import mano_palm_frame, mirror_xr_joints, palm_frame, xr_to_mano

HANDS_DIR = "hands"
RETARGETING_TYPES = ("dexpilot", "vector", "position")
_DEX_KEYS = frozenset({
    "type", "wrist_link_name", "finger_tip_link_names", "target_joint_names",
    "target_origin_link_names", "target_task_link_names", "target_link_human_indices",
    "vector_weight", "position_weight", "scaling_factor", "normal_delta", "huber_delta",
    "project_dist", "escape_dist", "low_pass_alpha",
})
_VECTOR_LINK_KEYS = ("target_origin_link_names", "target_task_link_names",
                     "target_link_human_indices")
_DEFAULT_HUMAN_HAND_LENGTH = 0.19
_MANO_TIP_STEP = 4


def hand_config_names() -> List[str]:
    d = os.path.join(configs_root(), HANDS_DIR)
    if not os.path.isdir(d):
        return []
    return sorted(os.path.splitext(f)[0] for f in os.listdir(d) if f.endswith(".yaml"))


def resolve_hand_config(name: str, side: Optional[str] = None) -> str:
    if name.startswith(DEX_PREFIX):
        name = name[len(DEX_PREFIX):]
    p = os.path.expanduser(name)
    if os.path.isfile(p):
        return os.path.abspath(p)
    d = os.path.join(configs_root(), HANDS_DIR)
    cands = [os.path.join(d, f"{name}.yaml")]
    if side is not None:
        cands.append(os.path.join(d, f"{name}_{side}.yaml"))
    for c in cands:
        if os.path.isfile(c):
            return c
    raise FileNotFoundError(
        f"손 리타게팅 config 없음: {name!r} (side={side}) — 가능한 것: {hand_config_names()}"
    )


@dataclass
class DexHandConfig:
    name: str
    side: str
    urdf: str
    retargeting: Dict
    palm_links: Tuple[Tuple[str, ...], Tuple[str, ...], Tuple[str, ...]]
    fixed_joints: Dict[str, float] = field(default_factory=dict)
    link_offsets: Dict[str, Tuple[float, float, float]] = field(default_factory=dict)
    joint_limits: Dict[str, Tuple[float, float]] = field(default_factory=dict)
    human_hand_length: float = _DEFAULT_HUMAN_HAND_LENGTH
    mano_to_root_rpy: Optional[Tuple[float, float, float]] = None
    max_evals: Optional[int] = None
    path: Optional[str] = None

    @staticmethod
    def from_dict(d: dict, path: Optional[str] = None) -> "DexHandConfig":
        side = d.get("side")
        if side not in SIDES:
            raise ValueError(f"hand config side 는 left|right: {side!r}")
        rt = dict(d.get("retargeting") or {})
        unknown = set(rt) - _DEX_KEYS
        if unknown:
            raise ValueError(f"retargeting 에 모르는 키: {sorted(unknown)}")
        palm = d.get("palm_links") or {}
        if not {"index", "middle", "pinky"} <= set(palm):
            raise ValueError(f"palm_links 에 index/middle/pinky 가 필요하다: {sorted(palm)}")
        palm_links = tuple(
            (str(palm[k]),) if isinstance(palm[k], str) else tuple(str(n) for n in palm[k])
            for k in ("index", "middle", "pinky")
        )
        frame = d.get("mano_to_root")
        limits = {
            str(k): (float(v[0]), float(v[1]))
            for k, v in (d.get("joint_limits_deg") or {}).items()
        }
        return DexHandConfig(
            name=str(d.get("name") or os.path.splitext(os.path.basename(path or "hand"))[0]),
            side=side,
            urdf=str(d["urdf"]),
            retargeting=rt,
            palm_links=palm_links,
            fixed_joints={str(k): float(v) for k, v in (d.get("fixed_joints") or {}).items()},
            link_offsets={str(k): tuple(float(x) for x in v)
                          for k, v in (d.get("link_offsets") or {}).items()},
            joint_limits=limits,
            human_hand_length=float(d.get("human_hand_length", _DEFAULT_HUMAN_HAND_LENGTH)),
            mano_to_root_rpy=None if frame is None else tuple(float(v) for v in frame["rpy"]),
            max_evals=None if d.get("max_evals") is None else int(d["max_evals"]),
            path=path,
        )

    @staticmethod
    def load(name: str, side: Optional[str] = None) -> "DexHandConfig":
        p = resolve_hand_config(name, side)
        with open(p) as f:
            return DexHandConfig.from_dict(yaml.safe_load(f) or {}, path=p)


class DexEngine:
    def __init__(
        self,
        config,
        side: Optional[str] = None,
        urdf_root: Optional[str] = None,
        max_evals: Optional[int] = None,
        **overrides,
    ):
        cfg = config if isinstance(config, DexHandConfig) else DexHandConfig.load(config, side)
        self.config = cfg
        self.side = cfg.side if side is None else str(side).lower()
        if self.side not in SIDES:
            raise ValueError(f"side 는 left|right: {side!r}")
        self.mirror = self.side != cfg.side
        unknown = set(overrides) - _DEX_KEYS
        if unknown:
            raise TypeError(
                f"DexEngine 가 모르는 인자: {sorted(unknown)} — "
                f"retargeting 키만 덮어쓸 수 있다: {sorted(_DEX_KEYS)}"
            )
        self.urdf_path = os.path.join(urdf_root or models_root(), cfg.urdf)
        if not os.path.isfile(self.urdf_path):
            raise FileNotFoundError(f"{cfg.name}: URDF 없음 {self.urdf_path}")

        rt = {**cfg.retargeting, **overrides}
        kind = str(rt.get("type", "DexPilot")).lower()
        if kind not in RETARGETING_TYPES:
            raise ValueError(f"{cfg.name}: retargeting.type 은 DexPilot|Vector|Position: {kind!r}")
        self.type = kind.upper()
        kin = Kinematics(self.urdf_path)
        missing = [n for n in cfg.fixed_joints if n not in kin.joint_names]
        if missing:
            raise ValueError(f"{cfg.name}: fixed_joints 가 URDF 에 없다 {missing}")
        shifted = {link: f"{link}_offset" for link in cfg.link_offsets}
        for link, offset in cfg.link_offsets.items():
            kin.add_frame(shifted[link], link, offset)
        target_joints = rt.get("target_joint_names")
        if target_joints is None:
            target_joints = [n for n in kin.joint_names
                             if n not in cfg.fixed_joints and n not in kin.mimic_names]
        q0 = kin.neutral()
        for n, v in cfg.fixed_joints.items():
            q0[kin.joint_names.index(n)] = v
        pos = self._frame_positions(kin, q0)
        wrist = rt.get("wrist_link_name")
        if wrist is None:
            raise ValueError(f"{cfg.name}: retargeting.wrist_link_name 이 필요하다")
        wrist = shifted.get(wrist, wrist)
        self._rot_robot = palm_frame(
            pos(wrist),
            *(np.mean([pos(n) for n in links], axis=0) for links in cfg.palm_links),
            side=cfg.side,
        )
        if self._rot_robot is None:
            raise ValueError(f"{cfg.name}: palm_links 로 로봇 손바닥 프레임을 정할 수 없다")
        self._rot_fixed = (
            None
            if cfg.mano_to_root_rpy is None
            else Rotation.from_euler("xyz", cfg.mano_to_root_rpy).as_matrix()
        )
        tips = [shifted.get(n, n) for n in rt.get("finger_tip_link_names") or []]
        if rt.get("scaling_factor") is None:
            ref = tips[2] if len(tips) > 2 else (rt.get("target_task_link_names") or [wrist])[-1]
            rt["scaling_factor"] = float(
                np.linalg.norm(pos(ref) - pos(wrist)) / cfg.human_hand_length
            )
        self.scaling_factor = float(rt["scaling_factor"])

        weights = {k: float(rt.get(f"{k}_weight", 0.0)) for k in ("vector", "position")}
        if kind in weights:
            if f"{kind}_weight" in rt:
                raise ValueError(
                    f"{cfg.name}: type {rt['type']} 에는 {kind}_weight 를 주지 않는다"
                )
            weights[kind] = 1.0
        given = [k for k in _VECTOR_LINK_KEYS if k in rt]
        if given and weights["vector"] == 0.0:
            raise ValueError(
                f"{cfg.name}: {given} 는 type Vector 나 vector_weight > 0 에서만 쓴다"
            )
        uses_tips = kind == "dexpilot" or weights["position"] > 0.0 or (
            weights["vector"] > 0.0 and "target_task_link_names" not in rt
        )
        if uses_tips and not tips:
            raise ValueError(f"{cfg.name}: retargeting.finger_tip_link_names 가 필요하다")
        tip_human = [_MANO_TIP_STEP * (i + 1) for i in range(len(tips))]
        human: List[List[int]] = [[], []]
        terms: List[VectorTerm] = []
        huber = float(rt.get("huber_delta", 2e-2))
        if kind == "dexpilot":
            o, t = dexpilot_pairs(len(tips))
            human = [[_MANO_TIP_STEP * i for i in o], [_MANO_TIP_STEP * i for i in t]]
        if weights["vector"] > 0.0:
            origins = rt.get("target_origin_link_names") or [wrist] * len(tips)
            tasks = rt.get("target_task_link_names") or tips
            idx = rt.get("target_link_human_indices") or [[0] * len(tips), tip_human]
            if not (len(origins) == len(tasks) == len(idx[0]) == len(idx[1])):
                raise ValueError(f"{cfg.name}: vector 링크·human indices 길이가 다르다")
            terms.append(VectorTerm(list(origins), list(tasks), weights["vector"], huber))
            human = [human[0] + [int(i) for i in idx[0]], human[1] + [int(i) for i in idx[1]]]
        if weights["position"] > 0.0:
            terms.append(VectorTerm([wrist] * len(tips), tips, weights["position"], huber))
            human = [human[0] + [0] * len(tips), human[1] + tip_human]
        self._human_idx = np.array(human, dtype=int)

        self.optimizer = RetargetOptimizer(
            kin,
            target_joints,
            q0,
            (wrist, tips) if kind == "dexpilot" else None,
            terms,
            scaling=self.scaling_factor,
            normal_delta=float(rt.get("normal_delta", 4e-3)),
            project_dist=float(rt.get("project_dist", 0.03)),
            escape_dist=float(rt.get("escape_dist", 0.05)),
        )
        self.kinematics = kin
        self.wrist_frame = wrist
        self.tip_frames = list(tips)
        self.joint_names: List[str] = list(target_joints)
        self.low_pass_alpha = float(rt.get("low_pass_alpha", 0.1))
        self._smoothed: Optional[np.ndarray] = None
        self.last_failure: Optional[str] = None
        self._apply_limits(kin.joint_limits[self.optimizer.idx_target], cfg.joint_limits)
        self.max_evals = cfg.max_evals if max_evals is None else int(max_evals)
        if self.max_evals:
            self.optimizer.set_max_evals(self.max_evals)

    @staticmethod
    def _frame_positions(kin: Kinematics, q):
        tf = kin.link_placements(q)

        def pos(name: str) -> np.ndarray:
            return tf[kin.frame_ids([name])[0], :3, 3].copy()

        return pos

    def _apply_limits(self, lim: np.ndarray, overrides: Dict[str, Tuple[float, float]]) -> None:
        lim = np.array(lim, dtype=float)
        for pat, (lo, hi) in overrides.items():
            hit = [i for i, n in enumerate(self.joint_names) if fnmatch.fnmatchcase(n, pat)]
            if not hit:
                raise ValueError(
                    f"{self.config.name}: joint_limits_deg 패턴에 맞는 관절이 없다: {pat}"
                )
            for i in hit:
                lim[i] = np.radians([lo, hi])
        self.optimizer.set_joint_limits(lim)
        self._limits = lim
        self._last = lim.mean(1).astype(np.float32)

    @property
    def joint_limits(self) -> np.ndarray:
        return self._limits.copy()

    def human_keypoints(self, xr_joints) -> Optional[np.ndarray]:
        xr = mirror_xr_joints(xr_joints) if self.mirror else xr_joints
        mano = xr_to_mano(xr)
        if self._rot_fixed is not None:
            return mano @ self._rot_fixed.T
        palm = mano_palm_frame(mano, self.config.side)
        if palm is None:
            return None
        return mano @ (self._rot_robot @ palm.T).T

    def reference(self, keypoints: np.ndarray) -> np.ndarray:
        return keypoints[self._human_idx[1]] - keypoints[self._human_idx[0]]

    def _fail(self, reason: str) -> None:
        self.last_failure = reason

    def solve(self, xr_joints) -> Optional[np.ndarray]:
        kp = self.human_keypoints(xr_joints)
        if kp is None:
            return self._fail("손바닥 프레임: 손목·검지·중지·소지 MCP 로 방향을 정할 수 없다")
        if not np.all(np.isfinite(kp)):
            return self._fail("손 키포인트에 유한하지 않은 값이 있다")
        start = np.clip(self._last, self._limits[:, 0], self._limits[:, 1])
        x = self.optimizer.solve(self.reference(kp), start)
        if x is None:
            return self._fail(f"SLSQP 가 풀지 못했다: {self.optimizer.last_result.message}")
        self._last = x
        full = self.optimizer.full_qpos(self._last)
        if 0.0 <= self.low_pass_alpha <= 1.0:
            if self._smoothed is not None:
                full = self._smoothed + self.low_pass_alpha * (full - self._smoothed)
            self._smoothed = full
        self.last_failure = None
        return full[self.optimizer.idx_target]

    def full_qpos(self, q: np.ndarray) -> np.ndarray:
        return self.optimizer.full_qpos(np.asarray(q, dtype=float))

    @property
    def robot_joint_names(self) -> List[str]:
        return list(self.kinematics.joint_names)

    def reset(self) -> None:
        self._last = self._limits.mean(1).astype(np.float32)
        self._smoothed = None
        self.last_failure = None
        self.optimizer.reset()


__all__ = [
    "DexHandConfig",
    "DexEngine",
    "hand_config_names",
    "resolve_hand_config",
]
