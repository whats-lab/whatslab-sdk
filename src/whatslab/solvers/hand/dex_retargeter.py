from __future__ import annotations

import copy
import fnmatch
import logging
import os
from dataclasses import dataclass, field, fields
from typing import Dict, List, Optional, Tuple

import numpy as np
import pinocchio as pin
import yaml
from dex_retargeting.retargeting_config import RetargetingConfig
from scipy.spatial.transform import Rotation

from whatslab.paths import configs_root, models_root

from .xr_input import mano_palm_frame, mirror_xr_joints, palm_frame, xr_to_mano

DEX_PREFIX = "dex:"
HANDS_DIR = "hands"
_SIDES = ("left", "right")
_DEX_KEYS = frozenset(f.name for f in fields(RetargetingConfig))
_DEFAULT_HUMAN_HAND_LENGTH = 0.19

logging.getLogger("dex_retargeting.yourdfpy").setLevel(logging.ERROR)


def is_dex_config(name) -> bool:
    return isinstance(name, str) and name.startswith(DEX_PREFIX)


def hand_config_names() -> List[str]:
    d = os.path.join(configs_root(), HANDS_DIR)
    if not os.path.isdir(d):
        return []
    return sorted(os.path.splitext(f)[0] for f in os.listdir(d) if f.endswith(".yaml"))


def resolve_hand_config(name: str, side: Optional[str] = None) -> str:
    if is_dex_config(name):
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
    raise FileNotFoundError(f"손 리타게팅 config 없음: {name!r} (side={side}) — "
                            f"가능한 것: {hand_config_names()}")


@dataclass
class DexHandConfig:

    name: str
    side: str
    urdf: str
    retargeting: Dict
    palm_links: Tuple[str, str, str]
    fixed_joints: Dict[str, float] = field(default_factory=dict)
    joint_limits: Dict[str, Tuple[float, float]] = field(default_factory=dict)
    human_hand_length: float = _DEFAULT_HUMAN_HAND_LENGTH
    mano_to_root_rpy: Optional[Tuple[float, float, float]] = None
    max_evals: Optional[int] = None
    path: Optional[str] = None

    @staticmethod
    def from_dict(d: dict, path: Optional[str] = None) -> "DexHandConfig":
        side = d.get("side")
        if side not in _SIDES:
            raise ValueError(f"hand config side 는 left|right: {side!r}")
        rt = dict(d.get("retargeting") or {})
        unknown = set(rt) - _DEX_KEYS
        if unknown:
            raise ValueError(f"retargeting 에 dex-retargeting 이 모르는 키: {sorted(unknown)}")
        palm = d.get("palm_links") or {}
        try:
            palm_links = (str(palm["index"]), str(palm["middle"]), str(palm["pinky"]))
        except KeyError as e:
            raise ValueError(f"palm_links 에 index/middle/pinky 가 필요하다: {e}") from None
        frame = d.get("mano_to_root")
        limits = {str(k): (float(v[0]), float(v[1]))
                  for k, v in (d.get("joint_limits_deg") or {}).items()}
        return DexHandConfig(
            name=str(d.get("name") or os.path.splitext(os.path.basename(path or "hand"))[0]),
            side=side, urdf=str(d["urdf"]), retargeting=rt, palm_links=palm_links,
            fixed_joints={str(k): float(v) for k, v in (d.get("fixed_joints") or {}).items()},
            joint_limits=limits,
            human_hand_length=float(d.get("human_hand_length", _DEFAULT_HUMAN_HAND_LENGTH)),
            mano_to_root_rpy=None if frame is None else tuple(float(v) for v in frame["rpy"]),
            max_evals=None if d.get("max_evals") is None else int(d["max_evals"]),
            path=path)

    @staticmethod
    def load(name: str, side: Optional[str] = None) -> "DexHandConfig":
        p = resolve_hand_config(name, side)
        with open(p) as f:
            return DexHandConfig.from_dict(yaml.safe_load(f) or {}, path=p)


class XRHandRetargeter:

    def __init__(self, config, side: Optional[str] = None, urdf_root: Optional[str] = None,
                 max_evals: Optional[int] = None, **overrides):
        cfg = config if isinstance(config, DexHandConfig) else DexHandConfig.load(config, side)
        self.config = cfg
        self.side = cfg.side if side is None else str(side).lower()
        if self.side not in _SIDES:
            raise ValueError(f"side 는 left|right: {side!r}")
        self.mirror = self.side != cfg.side
        unknown = set(overrides) - _DEX_KEYS
        if unknown:
            raise TypeError(f"XRHandRetargeter 가 모르는 인자: {sorted(unknown)} — "
                            f"dex-retargeting RetargetingConfig 필드만 덮어쓸 수 있다")
        self.urdf_path = os.path.join(urdf_root or models_root(), cfg.urdf)
        if not os.path.isfile(self.urdf_path):
            raise FileNotFoundError(f"{cfg.name}: URDF 없음 {self.urdf_path}")

        rt = copy.deepcopy(cfg.retargeting)
        rt.update(overrides)
        rt["urdf_path"] = self.urdf_path
        rt.setdefault("type", "DexPilot")
        model = pin.buildModelFromUrdf(self.urdf_path)
        dof_names = [n for i, n in enumerate(model.names) if model.nqs[i] > 0]
        missing = [n for n in cfg.fixed_joints if n not in dof_names]
        if missing:
            raise ValueError(f"{cfg.name}: fixed_joints 가 URDF 에 없다 {missing}")
        if rt.get("target_joint_names") is None:
            rt["target_joint_names"] = [n for n in dof_names if n not in cfg.fixed_joints]
        q0 = pin.neutral(model)
        for n, v in cfg.fixed_joints.items():
            q0[model.idx_qs[model.getJointId(n)]] = v
        pos = self._frame_positions(model, q0)
        wrist = rt.get("wrist_link_name")
        if wrist is None:
            raise ValueError(f"{cfg.name}: retargeting.wrist_link_name 이 필요하다")
        self._R_robot = palm_frame(pos(wrist), *(pos(n) for n in cfg.palm_links), side=cfg.side)
        self._R_fixed = (None if cfg.mano_to_root_rpy is None else
                         Rotation.from_euler("xyz", cfg.mano_to_root_rpy).as_matrix())
        tips = rt.get("finger_tip_link_names") or []
        if rt.get("scaling_factor") is None:
            ref = tips[2] if len(tips) > 2 else (rt.get("target_link_names") or [wrist])[-1]
            rt["scaling_factor"] = float(np.linalg.norm(pos(ref) - pos(wrist))
                                         / cfg.human_hand_length)
        self.scaling_factor = float(rt["scaling_factor"])
        self._wrist_root = pos(wrist)

        self._seq = RetargetingConfig.from_dict(rt).build()
        opt = self._seq.optimizer
        self.type = opt.retargeting_type
        self.joint_names: List[str] = list(opt.target_joint_names)
        dof = opt.robot.dof_joint_names
        self._fixed_qpos = np.array([cfg.fixed_joints.get(dof[i], 0.0) for i in opt.idx_pin2fixed],
                                    dtype=float)
        self._idx_target = np.asarray(opt.idx_pin2target, dtype=int)
        self._apply_limits(cfg.joint_limits)
        self.max_evals = cfg.max_evals if max_evals is None else int(max_evals)
        if self.max_evals:
            opt.opt.set_maxeval(self.max_evals)
        self._human_idx = (None if opt.target_link_human_indices is None
                           else np.asarray(opt.target_link_human_indices, dtype=int))
        self._last_mano: Optional[np.ndarray] = None

    @staticmethod
    def _frame_positions(model, q):
        data = model.createData()
        pin.framesForwardKinematics(model, data, q)

        def pos(name: str) -> np.ndarray:
            if not model.existFrame(name):
                raise ValueError(f"URDF 에 없는 링크: {name}")
            return data.oMf[model.getFrameId(name)].translation.copy()
        return pos

    def _apply_limits(self, overrides: Dict[str, Tuple[float, float]]) -> None:
        lim = np.array(self._seq.joint_limits, dtype=float)
        for pat, (lo, hi) in overrides.items():
            hit = [i for i, n in enumerate(self.joint_names) if fnmatch.fnmatchcase(n, pat)]
            if not hit:
                raise ValueError(f"{self.config.name}: joint_limits_deg 패턴에 맞는 관절이 없다: {pat}")
            for i in hit:
                lim[i] = np.radians([lo, hi])
        self._seq.optimizer.set_joint_limit(lim)
        self._seq.joint_limits = lim
        self._seq.last_qpos = lim.mean(1).astype(np.float32)

    @property
    def joint_limits(self) -> np.ndarray:
        return np.array(self._seq.joint_limits, dtype=float)

    @property
    def mano_to_root(self) -> Optional[np.ndarray]:
        return None if self._R_fixed is None else self._R_fixed.copy()

    def human_keypoints(self, xr_joints) -> np.ndarray:
        xr = mirror_xr_joints(xr_joints) if self.mirror else xr_joints
        mano = xr_to_mano(xr)
        if self._R_fixed is not None:
            R = self._R_fixed
        else:
            R = self._R_robot @ mano_palm_frame(mano, self.config.side).T
        return mano @ R.T

    def reference(self, keypoints: np.ndarray) -> np.ndarray:
        if self.type == "POSITION":
            return self._wrist_root + self.scaling_factor * keypoints[self._human_idx]
        return keypoints[self._human_idx[1]] - keypoints[self._human_idx[0]]

    def compute(self, xr_joints) -> np.ndarray:
        kp = self.human_keypoints(xr_joints)
        self._last_mano = kp
        q = self._seq.retarget(self.reference(kp), fixed_qpos=self._fixed_qpos)
        return np.asarray(q, dtype=float)[self._idx_target]

    @property
    def last_keypoints(self) -> Optional[np.ndarray]:
        return None if self._last_mano is None else self._last_mano.copy()

    def full_qpos(self, q: np.ndarray) -> np.ndarray:
        out = np.zeros(self._seq.optimizer.robot.dof)
        out[self._seq.optimizer.idx_pin2fixed] = self._fixed_qpos
        out[self._idx_target] = np.asarray(q, dtype=float)
        return out

    @property
    def robot_joint_names(self) -> List[str]:
        return list(self._seq.optimizer.robot.dof_joint_names)

    def reset(self) -> None:
        self._seq.reset()
        if self._seq.filter is not None:
            self._seq.filter.reset()
        proj = getattr(self._seq.optimizer, "projected", None)
        if proj is not None:
            proj[:] = False
        self._last_mano = None


__all__ = ["DEX_PREFIX", "DexHandConfig", "XRHandRetargeter", "hand_config_names",
           "is_dex_config", "resolve_hand_config"]
