"""로봇/rig config 로더 — 책임 일대일 (docs/DESIGN_robot_rig.md 기준).

  URDF        : 기구학(관절 이름/한계)의 유일한 출처 — config 에 복제 금지
  robot yaml  : "이 로봇이 무엇인가" (urdf, axis_align, TCP/base_frame, reach_max)
  rig yaml    : 조립(mount/attach/target_ee) + 운용(lock/solver) + 세션(calibration)
  입력 소스/드라이버 : rig 에 없음 — 파이프라인 인자 (사용자 코드)

모든 고정 변환은 URDF <origin> 관례(xyz + rpy, 이동은 부모 프레임 기준).
"""
from __future__ import annotations

import os
from dataclasses import dataclass, field
from typing import List, Optional

import numpy as np
import yaml
from scipy.spatial.transform import Rotation

from whatslab.paths import configs_root, models_root


def origin_to_T(xyz, rpy) -> np.ndarray:
    """URDF <origin>(xyz+rpy) → 4x4. rpy 는 회전만 표현하므로 det=+1 자동 보장."""
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
    """개별 robot config — arm/hand 공통 스키마 (kind 로 구분)."""

    name: str
    kind: str                                            # "arm" | "hand"
    urdf: str                                            # models 루트 기준 상대경로
    axis_align: Origin = field(default_factory=Origin)   # URDF 관례 → 정준 정렬
    # arm 전용
    ee_parent: Optional[str] = None
    ee_origin: Origin = field(default_factory=Origin)    # parent→TCP (URDF origin)
    # hand 전용
    base_frame: Optional[str] = None                     # 장착점(손목)
    retarget: Optional[str] = None                       # 리타게팅 config 이름(CONFIG_REGISTRY)
    # target_ee 프레임(=IK 제어 프레임)을 정준 tool 규약(손끝=X, 손바닥=-Z)에
    # 정렬하는 국소 회전. 메쉬는 안 움직이고 IK 제어 프레임 축만 돌린다
    # (arm_ik ee_local_rpy). rpy 만 적용(xyz 는 현재 미사용). align_frames ee 모드로 튜닝.
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
    backend: str = "diff"            # dls | diff
    w_pos: float = 20.0
    w_ori: float = 10.0
    max_joint_velocity: float = 5.0
    reach_max: Optional[float] = None              # target_ee 안전 반경(베이스 기준)

    @staticmethod
    def from_dict(d) -> "SolverCfg":
        d = d or {}
        return SolverCfg(
            backend=d.get("backend", "diff"),
            w_pos=float(d.get("w_pos", 20.0)),
            w_ori=float(d.get("w_ori", 10.0)),
            max_joint_velocity=float(d.get("max_joint_velocity", 5.0)),
            reach_max=d.get("reach_max"),
        )


@dataclass
class CalibrationCfg:
    """사용자 측정치 — 입력 도달반경(input_reach) 하나. uniform reach 스케일용.

    model.solve 가 정준 위치를 `s = reach_max / input_reach` **단일 스칼라**로
    등방(isotropic) 스케일한다(원점 0 기준, 중심 빼기 없음). input_reach 는 사람
    손 최대 도달반경 |p|=√(x²+y²+z²) (calibrate 도구가 측정). None 이면 스케일 없음.
    """

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
class RigConfig:
    """통합 config — 조립·운용·세션. 소스/드라이버는 여기 없음(파이프라인 인자)."""

    name: str
    arm: Optional[RobotSpec]
    hand: Optional[RobotSpec]
    mount: Origin                    # 정준 → 루트 로봇 베이스 (arm 없으면 hand)
    attach: Origin                   # arm TCP → hand base_frame (조합 시)
    lock_joints: List[str]
    target_ee: Optional[str]
    solver: SolverCfg
    calibration: CalibrationCfg
    path: Optional[str] = None       # 로드 원본 (캘리브 갱신용)

    def resolve_target_ee(self) -> str:
        """target_ee → hand.base_frame → arm TCP('ee') 순 fallback."""
        if self.target_ee:
            return self.target_ee
        if self.hand is not None and self.hand.base_frame:
            return self.hand.base_frame
        return "ee"


def _load_yaml(path: str) -> dict:
    with open(path) as f:
        return yaml.safe_load(f) or {}


def _resolve_config(path: str, subdir: str) -> str:
    """config 경로 해석: 절대 → 그대로 / 상대 → cwd, 다음 configs/<subdir> 기준."""
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
    """단일 robot yaml 로드 (configs/robots 기준 상대경로 허용)."""
    p = _resolve_config(path, "robots")
    if not os.path.exists(p):
        raise FileNotFoundError(f"robot config 없음: {path}")
    return RobotSpec.from_dict(_load_yaml(p))


def load_rig(path: str) -> RigConfig:
    """rig yaml 로드 + robot 참조 해석 + 검증.

    path 해석: 절대경로 → 그대로 / 상대경로 → cwd, 다음 configs_root() 기준.
    rig 내부의 robot 참조("robots/nero.yaml")는 configs 루트(= rig 파일의
    부모의 부모) 기준.
    """
    p = os.path.expanduser(path)
    if not os.path.isabs(p) and not os.path.exists(p):
        root = configs_root()
        if root and os.path.exists(os.path.join(root, p)):
            p = os.path.join(root, p)
    p = os.path.abspath(p)
    if not os.path.exists(p):
        raise FileNotFoundError(f"rig config 없음: {path}")
    d = _load_yaml(p)
    cfg_root = os.path.dirname(os.path.dirname(p))        # configs/

    def _load_robot(ref) -> RobotSpec:
        if isinstance(ref, dict):                          # 인라인 정의 허용
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
        path=p,
    )
    # hand 단독 rig: 위치 개념 없음 → calibration 무시 경고
    if arm is None and rig.calibration.complete:
        print("[rig] WARN: hand 단독 rig — calibration 은 무시됩니다", flush=True)
    return rig


def save_calibration(rig: RigConfig, input_reach: float) -> None:
    """rig yaml 의 calibration.input_reach 만 in-place 갱신 (다른 섹션 불변)."""
    assert rig.path, "rig 가 파일에서 로드되지 않음"
    with open(rig.path) as f:
        d = yaml.safe_load(f) or {}
    cal = d.setdefault("calibration", {})
    cal["enabled"] = True
    cal["input_reach"] = round(float(input_reach), 4)
    with open(rig.path, "w") as f:
        yaml.safe_dump(d, f, allow_unicode=True, sort_keys=False)
    rig.calibration = CalibrationCfg.from_dict(cal)


def save_reach_max(rig: RigConfig, reach_max: float) -> None:
    """rig yaml 의 solver.reach_max 만 in-place 갱신 (다른 키·섹션 불변)."""
    assert rig.path, "rig 가 파일에서 로드되지 않음"
    with open(rig.path) as f:
        d = yaml.safe_load(f) or {}
    d.setdefault("solver", {})["reach_max"] = round(float(reach_max), 4)
    with open(rig.path, "w") as f:
        yaml.safe_dump(d, f, allow_unicode=True, sort_keys=False)
    rig.solver.reach_max = float(reach_max)
