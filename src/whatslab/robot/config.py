from __future__ import annotations

import os
from dataclasses import dataclass, field
from typing import Dict, List, Optional

import numpy as np
import yaml
from scipy.spatial.transform import Rotation

from whatslab.paths import configs_root, models_root


def origin_to_T(xyz, rpy) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = Rotation.from_euler(
        "xyz", list(rpy) if rpy is not None else [0, 0, 0]).as_matrix()
    T[:3, 3] = np.asarray(xyz if xyz is not None else [0, 0, 0], dtype=float)
    return T


@dataclass
class Origin:
    xyz: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    rpy: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])

    @property
    def T(self) -> np.ndarray:
        return origin_to_T(self.xyz, self.rpy)

    @staticmethod
    def from_dict(d) -> "Origin":
        d = d or {}
        return Origin(xyz=list(d.get("xyz", [0, 0, 0])),
                      rpy=list(d.get("rpy", [0, 0, 0])))


@dataclass
class RobotSpec:

    name: str
    kind: str
    urdf: str
    axis_align: Origin = field(default_factory=Origin)
    ee_parent: Optional[str] = None
    ee_origin: Origin = field(default_factory=Origin)
    base_frame: Optional[str] = None
    retarget: Optional[str] = None
    ee_align: Origin = field(default_factory=Origin)

    @staticmethod
    def from_dict(d: dict) -> "RobotSpec":
        kind = d.get("kind")
        if kind not in ("arm", "hand"):
            raise ValueError(f"robot kind 는 arm|hand: {kind!r}")
        ee = d.get("ee") or {}
        spec = RobotSpec(
            name=d["name"], kind=kind, urdf=d["urdf"],
            axis_align=Origin.from_dict(d.get("axis_align")),
            ee_parent=ee.get("parent"),
            ee_origin=Origin.from_dict(ee.get("origin")),
            base_frame=d.get("base_frame"),
            ee_align=Origin.from_dict(d.get("ee_align")),
            retarget=d.get("retarget"),
        )
        if kind == "arm" and not spec.ee_parent:
            raise ValueError(f"arm robot({spec.name})은 ee.parent 필수")
        if kind == "hand" and not spec.base_frame:
            raise ValueError(f"hand robot({spec.name})은 base_frame 필수")
        return spec

    def urdf_abspath(self) -> str:
        return os.path.join(models_root(), self.urdf)


@dataclass
class SolverCfg:
    backend: str = "diff"
    w_pos: float = 20.0
    w_ori: float = 10.0
    max_joint_velocity: float = 5.0
    reach_max: Optional[float] = None
    max_iter: Optional[int] = None
    iters_per_call: Optional[int] = None
    tol: Optional[float] = None
    dp_max: Optional[float] = None
    dtheta_max: Optional[float] = None
    dq_max_tick: Optional[float] = None
    sugihara_bias: Optional[float] = None
    k_posture: Optional[float] = None
    proj_rcond: Optional[float] = None
    k_limit: Optional[float] = None
    limit_margin: Optional[float] = None
    joint_weights: Optional[Dict[str, float]] = None

    @staticmethod
    def from_dict(d) -> "SolverCfg":
        d = d or {}

        def _opt(key, cast):
            v = d.get(key)
            return None if v is None else cast(v)

        return SolverCfg(
            backend=d.get("backend", "diff"),
            w_pos=float(d.get("w_pos", 20.0)),
            w_ori=float(d.get("w_ori", 10.0)),
            max_joint_velocity=float(d.get("max_joint_velocity", 5.0)),
            reach_max=d.get("reach_max"),
            max_iter=_opt("max_iter", int),
            iters_per_call=_opt("iters_per_call", int),
            tol=_opt("tol", float),
            dp_max=_opt("dp_max", float),
            dtheta_max=_opt("dtheta_max", float),
            dq_max_tick=_opt("dq_max_tick", float),
            sugihara_bias=_opt("sugihara_bias", float),
            k_posture=_opt("k_posture", float),
            proj_rcond=_opt("proj_rcond", float),
            k_limit=_opt("k_limit", float),
            limit_margin=_opt("limit_margin", float),
            joint_weights=({str(k): float(v)
                            for k, v in d["joint_weights"].items()}
                           if d.get("joint_weights") else None),
        )


@dataclass
class CalibrationCfg:

    enabled: bool = True
    input_reach: Optional[float] = None

    @property
    def complete(self) -> bool:
        return self.input_reach is not None

    @staticmethod
    def from_dict(d) -> "CalibrationCfg":
        d = d or {}
        return CalibrationCfg(enabled=bool(d.get("enabled", True)),
                              input_reach=d.get("input_reach"))


@dataclass
class HandSolverCfg:

    onnx_path: Optional[str] = None
    tables_path: Optional[str] = None
    threads: Optional[int] = None

    @staticmethod
    def from_dict(d) -> "HandSolverCfg":
        d = d or {}
        if "backend" in d:
            raise ValueError("hand_solver.backend 는 제거됐다 — 리타게팅 엔진은"
                             " 통합 ONNX(UniRetargeter) 하나뿐이다")
        thr = d.get("threads")
        return HandSolverCfg(
            onnx_path=(None if d.get("onnx_path") is None
                       else str(d["onnx_path"])),
            tables_path=(None if d.get("tables_path") is None
                         else str(d["tables_path"])),
            threads=None if thr is None else int(thr))

    def kwargs(self) -> dict:
        return {k: v for k, v in (("onnx_path", self.onnx_path),
                                  ("tables_path", self.tables_path),
                                  ("threads", self.threads)) if v is not None}


@dataclass
class RigConfig:
    name: str
    arm: Optional[RobotSpec]
    hand: Optional[RobotSpec]
    mount: Origin
    attach: Origin
    lock_joints: List[str]
    target_ee: Optional[str]
    solver: SolverCfg
    calibration: CalibrationCfg
    hand_solver: "HandSolverCfg" = field(default_factory=lambda: HandSolverCfg())
    path: Optional[str] = None

    def resolve_target_ee(self) -> str:
        if self.target_ee:
            return self.target_ee
        if self.hand is not None and self.hand.base_frame:
            return self.hand.base_frame
        return "ee"


def _load_yaml(path: str) -> dict:
    with open(path) as f:
        return yaml.safe_load(f) or {}


def _resolve_config(path: str, subdir: str) -> str:
    p = os.path.expanduser(path)
    if os.path.isabs(p) or os.path.exists(p):
        return os.path.abspath(p)
    root = configs_root()
    for cand in ((os.path.join(root, path),
                  os.path.join(root, subdir, path)) if root else ()):
        if os.path.exists(cand):
            return os.path.abspath(cand)
    return os.path.abspath(p)


def load_robot(path: str) -> RobotSpec:
    p = _resolve_config(path, "robots")
    if not os.path.exists(p):
        raise FileNotFoundError(f"robot config 없음: {path}")
    return RobotSpec.from_dict(_load_yaml(p))


def load_rig(path: str) -> RigConfig:
    p = os.path.expanduser(path)
    if not os.path.isabs(p) and not os.path.exists(p):
        root = configs_root()
        if root and os.path.exists(os.path.join(root, p)):
            p = os.path.join(root, p)
    p = os.path.abspath(p)
    if not os.path.exists(p):
        raise FileNotFoundError(f"rig config 없음: {path}")
    d = _load_yaml(p)
    cfg_root = os.path.dirname(os.path.dirname(p))

    def _load_robot(ref) -> RobotSpec:
        if isinstance(ref, dict):
            return RobotSpec.from_dict(ref)
        for base in (os.path.dirname(p), cfg_root):
            rp = os.path.join(base, ref)
            if os.path.exists(rp):
                return RobotSpec.from_dict(_load_yaml(rp))
        raise FileNotFoundError(f"robot config 없음: {ref} (기준: {cfg_root})")

    robots = d.get("robots") or {}
    arm = _load_robot(robots["arm"]) if robots.get("arm") else None
    hand = _load_robot(robots["hand"]) if robots.get("hand") else None
    if arm is None and hand is None:
        raise ValueError("rig 에 robots.arm 또는 robots.hand 최소 하나 필요")
    if arm is not None and arm.kind != "arm":
        raise ValueError(f"robots.arm 은 kind=arm: {arm.name}({arm.kind})")
    if hand is not None and hand.kind != "hand":
        raise ValueError(f"robots.hand 는 kind=hand: {hand.name}({hand.kind})")

    rig = RigConfig(
        name=d.get("name", os.path.splitext(os.path.basename(p))[0]),
        arm=arm, hand=hand,
        mount=Origin.from_dict(d.get("mount")),
        attach=Origin.from_dict(d.get("attach")),
        lock_joints=list(d.get("lock_joints") or []),
        target_ee=d.get("target_ee"),
        solver=SolverCfg.from_dict(d.get("solver")),
        calibration=CalibrationCfg.from_dict(d.get("calibration")),
        hand_solver=HandSolverCfg.from_dict(d.get("hand_solver")),
        path=p,
    )
    if arm is None and rig.calibration.complete:
        print("[rig] WARN: hand 단독 rig — calibration 은 무시됩니다", flush=True)
    return rig


def _dump_atomic(path: str, d: dict) -> None:
    tmp = f"{path}.tmp.{os.getpid()}"
    with open(tmp, "w") as f:
        yaml.safe_dump(d, f, allow_unicode=True, sort_keys=False)
        f.flush()
        os.fsync(f.fileno())
    os.replace(tmp, path)


def save_calibration(rig: RigConfig, input_reach: float) -> None:
    assert rig.path, "rig 가 파일에서 로드되지 않음"
    with open(rig.path) as f:
        d = yaml.safe_load(f) or {}
    cal = d.setdefault("calibration", {})
    cal.setdefault("enabled", True)
    cal["input_reach"] = round(float(input_reach), 4)
    _dump_atomic(rig.path, d)
    rig.calibration = CalibrationCfg.from_dict(cal)


def save_reach_max(rig: RigConfig, reach_max: float) -> None:
    assert rig.path, "rig 가 파일에서 로드되지 않음"
    with open(rig.path) as f:
        d = yaml.safe_load(f) or {}
    d.setdefault("solver", {})["reach_max"] = round(float(reach_max), 4)
    _dump_atomic(rig.path, d)
    rig.solver.reach_max = float(reach_max)
