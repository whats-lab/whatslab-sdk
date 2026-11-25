from __future__ import annotations

import fnmatch
import os
from dataclasses import dataclass, field
from typing import Dict, FrozenSet, List, Optional, Tuple

import numpy as np
import yaml
from scipy.spatial.transform import Rotation

from whatslab.core.constants import SIDES
from whatslab.kinematics import Kinematics, coupling_overrides, parse_couplings
from whatslab.paths import configs_root, models_root

from ..retargeter import DEX_PREFIX
from .optimizer import RetargetOptimizer, dexpilot_pairs
from .posture import POSTURE_KEY, PostureGuide, PostureSpec, parse_posture
from .preprocess import mano_palm_frame, mirror_xr_joints, palm_frame, xr_to_mano

HANDS_DIR = "hands"
_DEX_KEYS = frozenset({
    "type", "wrist_link_name", "finger_tip_link_names", "target_joint_names",
    "scaling_factor", "normal_delta", "project_dist", "project_full_dist", "escape_dist",
    "low_pass_alpha",
})
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
    couplings: Dict[str, Tuple[str, float, float]] = field(default_factory=dict)
    coupling_overrides: FrozenSet[str] = frozenset()
    joint_limits: Dict[str, Tuple[float, float]] = field(default_factory=dict)
    human_hand_length: float = _DEFAULT_HUMAN_HAND_LENGTH
    mano_to_root_rpy: Optional[Tuple[float, float, float]] = None
    max_evals: Optional[int] = None
    posture: Optional[PostureSpec] = None
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
            couplings=parse_couplings(d.get("couplings")),
            coupling_overrides=coupling_overrides(d.get("couplings")),
            joint_limits=limits,
            human_hand_length=float(d.get("human_hand_length", _DEFAULT_HUMAN_HAND_LENGTH)),
            mano_to_root_rpy=None if frame is None else tuple(float(v) for v in frame["rpy"]),
            max_evals=None if d.get("max_evals") is None else int(d["max_evals"]),
            posture=parse_posture(d.get(POSTURE_KEY)),
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
        posture_prior: bool = True,
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
        if str(rt.get("type", "DexPilot")).lower() != "dexpilot":
            raise ValueError(f"{cfg.name}: retargeting.type 은 DexPilot 만 된다: {rt['type']!r}")
        kin = Kinematics(self.urdf_path)
        missing = [n for n in cfg.fixed_joints if n not in kin.joint_names]
        if missing:
            raise ValueError(f"{cfg.name}: fixed_joints 가 URDF 에 없다 {missing}")
        fixed = sorted({n for name, (src, _, _) in cfg.couplings.items() for n in (name, src)
                        if n in cfg.fixed_joints})
        if fixed:
            raise ValueError(f"{cfg.name}: couplings 의 관절이 fixed_joints 에도 있다 {fixed}")
        kin.couple(cfg.couplings, cfg.coupling_overrides)
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
        if not tips:
            raise ValueError(f"{cfg.name}: retargeting.finger_tip_link_names 가 필요하다")
        if rt.get("scaling_factor") is None:
            ref = tips[2] if len(tips) > 2 else tips[-1]
            rt["scaling_factor"] = float(
                np.linalg.norm(pos(ref) - pos(wrist)) / cfg.human_hand_length
            )
        self.scaling_factor = float(rt["scaling_factor"])

        o, t = dexpilot_pairs(len(tips))
        self._human_idx = _MANO_TIP_STEP * np.array([o, t], dtype=int)

        project_dist = float(rt.get("project_dist", 0.03))
        self.optimizer = RetargetOptimizer(
            kin,
            target_joints,
            q0,
            wrist,
            tips,
            scaling=self.scaling_factor,
            normal_delta=float(rt.get("normal_delta", 4e-3)),
            project_dist=project_dist,
            project_full_dist=float(rt.get("project_full_dist", project_dist / 2)),
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
        self.posture = None
        if cfg.posture is not None and posture_prior:
            self.posture = PostureGuide(cfg.posture, kin, self.optimizer.full_qpos,
                                        self.joint_names, self._limits, self._rot_robot, cfg.side)
        self._posture_override = None

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
    def robot_palm(self) -> np.ndarray:
        return self._rot_robot.copy()

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

    def set_posture_target(self, center, weight=None) -> None:
        if center is None:
            self._posture_override = None
            return
        center = np.asarray(center, dtype=float)
        if center.shape != (len(self.joint_names),):
            raise ValueError(f"자세 목표는 ({len(self.joint_names)},) 이어야 한다 — "
                             f"받은 shape {center.shape}")
        if weight is None:
            if self.posture is None:
                raise ValueError(f"{self.config.name}: {POSTURE_KEY} 가 없어 weight 가 필요하다")
            weight = self.posture.weight
        weight = np.broadcast_to(np.asarray(weight, dtype=float), center.shape).copy()
        if not (np.all(np.isfinite(center)) and np.all(np.isfinite(weight))
                and np.all(weight >= 0.0)):
            raise ValueError("자세 목표·가중치는 유한하고 가중치는 0 이상이어야 한다")
        self._posture_override = (center, weight)

    def posture_target(self, xr_joints) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        if self._posture_override is not None:
            return self._posture_override
        if self.posture is None:
            return None
        xr = mirror_xr_joints(xr_joints) if self.mirror else np.asarray(xr_joints, dtype=float)
        return self.posture.target(xr)

    def solve(self, xr_joints) -> Optional[np.ndarray]:
        kp = self.human_keypoints(xr_joints)
        if kp is None:
            return self._fail("손바닥 프레임: 손목·검지·중지·소지 MCP 로 방향을 정할 수 없다")
        if not np.all(np.isfinite(kp)):
            return self._fail("손 키포인트에 유한하지 않은 값이 있다")
        prior = self.posture_target(xr_joints)
        if prior is None and (self.posture is not None or self._posture_override is not None):
            return self._fail(f"{POSTURE_KEY}: 사람 손 각도로 자세 목표를 정할 수 없다")
        start = np.clip(self._last, self._limits[:, 0], self._limits[:, 1])
        x = self.optimizer.solve(self.reference(kp), start, prior)
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


__all__ = [
    "DexHandConfig",
    "DexEngine",
    "hand_config_names",
    "resolve_hand_config",
]
