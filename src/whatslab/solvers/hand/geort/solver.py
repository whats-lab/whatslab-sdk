"""GeoRT 소프트 프롬프트 손 리타게팅 — robotPlatform `plugins/hand_solvers/geort` 의 이식.

입력은 OpenXR 26관절 × [qx qy qz qw px py pz radius] (손목 기준, m) — `HandPose.xr_joints`
그대로이고, 위치만 쓴다.
"""

from __future__ import annotations

import dataclasses
import functools
import logging
import os
from pathlib import Path
from typing import Mapping, Optional, Tuple

import numpy as np
import yaml

from whatslab.core.constants import OPENXR26_TO_HAND21, SIDES, XR_JOINT_COUNT, XR_JOINT_FIELDS

from .errors import GeortConfigError, GeortInputError
from .keypoints import POSITION
from .prompt import KIND_ROBOT, Embodiment, PromptModel, load_prompt_model

_log = logging.getLogger(__name__)

OPENXR26_LAYOUT = "openxr26"
KEYPOINT_SHAPE = (XR_JOINT_COUNT, XR_JOINT_FIELDS)
# 패키지에 동봉한 학습 모델(orca_right·hx5_right·allegro_right·dg5f_s_right 프롬프트).
DEFAULT_MODEL = str(Path(__file__).resolve().parent / "assets" / "geort_prompt.npz")
MODEL_ENV = "WHATSLAB_GEORT_MODEL"

_REMEDY_TRACKING = "장갑 착용 상태와 손 추적 상태를 확인하세요."
_REMEDY_FILE = "GeoRT 설정 YAML(name, side, model)과 모델 파일 경로를 확인하세요."

Limits = Mapping[str, Tuple[float, float]]


def solver_limits(model: Embodiment, robot_limits: Optional[Limits]) -> np.ndarray:
    """모델 학습 범위와 로봇 URDF 범위의 교집합 (n, 2).

    robot_limits 를 줬는데 관절 이름이 하나도 안 맞으면 안전 클램프가 조용히 꺼지므로 오류,
    일부만 맞으면 빠진 관절을 경고한다.
    """
    limits = np.column_stack([model.lower, model.upper])
    if not robot_limits:
        return limits
    missing = [n for n in model.joint_names if n not in robot_limits]
    if len(missing) == len(model.joint_names):
        raise GeortConfigError(
            "GEORT_LIMITS_UNMATCHED",
            f"GeoRT 모델 '{model.name}'의 관절 이름이 로봇 관절 범위에 하나도 없습니다 "
            f"(모델: {', '.join(model.joint_names[:3])}…, "
            f"로봇: {', '.join(list(robot_limits)[:3])}…)",
            "GeoRT 모델이 이 로봇 손 URDF 용인지 확인하세요(좌우 포함 — 오른손 로봇 손 모델은 "
            "왼손 로봇 손을 구동할 수 없다).",
        )
    if missing:
        _log.warning(
            "GeoRT 모델 '%s'의 관절 %s 은(는) 로봇 URDF 에 없어 학습 범위로만 자른다",
            model.name,
            ", ".join(missing),
        )
    for i, name in enumerate(model.joint_names):
        if name not in robot_limits:
            continue
        r_lo, r_hi = robot_limits[name]
        lo, hi = max(limits[i, 0], float(r_lo)), min(limits[i, 1], float(r_hi))
        if lo > hi:
            raise GeortConfigError(
                "GEORT_LIMITS_DISJOINT",
                f"관절 '{name}'의 GeoRT 범위 [{limits[i, 0]:.3f}, {limits[i, 1]:.3f}]와 "
                f"로봇 범위 [{r_lo:.3f}, {r_hi:.3f}] rad가 겹치지 않습니다",
                "GeoRT 모델이 이 로봇용인지 확인하세요.",
            )
        limits[i] = (lo, hi)
    return limits


class GeortHandSolver:
    """keypoints(26×8) → 로봇 손 관절각. 모델이 오른손용이면 왼손 입력은 좌우 반전해 푼다."""

    input_layout = OPENXR26_LAYOUT

    def __init__(
        self,
        model: Embodiment,
        human_side: str = "right",
        robot_limits: Optional[Limits] = None,
        low_pass_alpha: float = 1.0,
    ) -> None:
        if human_side not in SIDES:
            raise GeortConfigError(
                "GEORT_CONFIG_INVALID", f"human_side {human_side!r}는 left|right 여야 합니다"
            )
        if not 0.0 < low_pass_alpha <= 1.0:
            raise GeortConfigError(
                "GEORT_CONFIG_INVALID",
                f"low_pass_alpha {low_pass_alpha}는 (0, 1] 범위여야 합니다",
                "설정 파일의 low_pass_alpha를 고치세요.",
            )
        self._model = model
        self._alpha = low_pass_alpha
        self._last: Optional[np.ndarray] = None
        self._human_side = human_side
        self._mirror = human_side != model.side
        self._human_ids = model.human_ids
        self._limits = solver_limits(model, robot_limits)
        self._limits.setflags(write=False)

    @classmethod
    def from_config(
        cls,
        config_path: "str | os.PathLike[str]",
        human_side: str = "right",
        robot_limits: Optional[Limits] = None,
        low_pass_alpha: Optional[float] = None,
    ) -> "GeortHandSolver":
        """GeoRT 설정 YAML(name/side/model/low_pass_alpha)로 만든다.

        ``low_pass_alpha`` 를 주면 설정값 대신 쓴다.
        """
        cfg = read_config(config_path)
        alpha = cfg.low_pass_alpha if low_pass_alpha is None else float(low_pass_alpha)
        return cls(load_embodiment(cfg), human_side, robot_limits, alpha)

    @property
    def joint_names(self) -> Tuple[str, ...]:
        """출력 관절 이름(모델 순서)."""
        return self._model.joint_names

    @property
    def hand_side(self) -> str:
        """입력 사람 손 side."""
        return self._human_side

    @property
    def joint_limits(self) -> np.ndarray:
        """출력을 자르는 관절 범위 (n, 2)."""
        return self._limits

    @property
    def model(self) -> Embodiment:
        """쓰는 프롬프트 모델의 손."""
        return self._model

    @property
    def low_pass_alpha(self) -> float:
        """출력 저역통과 계수(1 이면 끔)."""
        return self._alpha

    def reset(self) -> None:
        """저역통과 상태를 비운다."""
        self._last = None

    def keypoints(self, keypoints: np.ndarray) -> np.ndarray:
        """OpenXR 26×8 을 모델 입력 키포인트로. 풀 수 없으면 GeortInputError."""
        kp = np.asarray(keypoints, dtype=np.float64)
        if kp.shape != KEYPOINT_SHAPE:
            raise ValueError(
                f"geort solver expects keypoints of shape {KEYPOINT_SHAPE} "
                f"(openxr26 layout), got {kp.shape}"
            )
        if not np.all(
            np.isfinite(kp[list(OPENXR26_TO_HAND21), POSITION])
        ):  # palm·검지~소지 중수골은 안 쓴다(엄지 중수골은 쓴다)
            raise GeortInputError(
                "GEORT_INPUT_NON_FINITE",
                "손 추적 위치에 유한하지 않은 값이 있어 리타게팅하지 않았습니다",
                _REMEDY_TRACKING,
            )
        points = self._model.frame.keypoints(kp, mirror=self._mirror)[self._human_ids]
        if not np.all(np.isfinite(points)):
            raise GeortInputError(
                "GEORT_SOLVE_FAILED",
                "손목과 손가락 뿌리 위치로 손바닥 방향을 정할 수 없습니다",
                _REMEDY_TRACKING,
            )
        return points

    def solve(self, keypoints: np.ndarray) -> np.ndarray:
        """OpenXR 26×8 → 관절각(범위로 자르고 저역통과)."""
        q = np.clip(
            self._model.forward(self.keypoints(keypoints)), self._limits[:, 0], self._limits[:, 1]
        )
        if self._alpha < 1.0:
            self._last = q if self._last is None else self._last + self._alpha * (q - self._last)
            return self._last.copy()
        return q


@dataclasses.dataclass(frozen=True)
class GeortConfig:
    """GeoRT 설정 YAML 내용."""
    name: str
    side: str
    model_path: Path
    low_pass_alpha: float = 1.0


def _invalid(path: Path, reason: str) -> GeortConfigError:
    return GeortConfigError(
        "GEORT_CONFIG_INVALID",
        f"GeoRT 설정 파일 형식이 잘못됐습니다: {path} ({reason})",
        _REMEDY_FILE,
    )


def _model_path(raw: Optional[str], base: Path) -> Path:
    """모델 경로: 설정의 model > $WHATSLAB_GEORT_MODEL > 동봉 모델.

    설정의 상대경로는 설정 파일 기준, 환경변수의 상대경로는 현재 디렉토리 기준이다.
    """
    if raw:
        model = Path(raw).expanduser()
        return model if model.is_absolute() else base / model
    return Path(os.environ.get(MODEL_ENV) or DEFAULT_MODEL).expanduser().resolve()


def resolve_config_path(path: "str | os.PathLike[str]") -> Path:
    """절대·현재 디렉토리 기준 경로가 없으면 whatslab configs 루트 기준으로 찾는다."""
    from whatslab.paths import configs_root

    p = Path(path).expanduser()
    if p.is_absolute() or p.exists():
        return p
    alt = Path(configs_root()) / p
    return alt if alt.exists() else p


def read_config(config_path: "str | os.PathLike[str]") -> GeortConfig:
    """GeoRT 설정 YAML 을 읽고 검사한다. 실패하면 GeortConfigError."""
    path = resolve_config_path(config_path)
    try:
        raw = yaml.safe_load(path.read_text(encoding="utf-8"))
    except FileNotFoundError as e:
        raise GeortConfigError(
            "GEORT_CONFIG_MISSING",
            f"GeoRT 설정 파일을 찾을 수 없습니다: {path}",
            "설정 경로를 확인하세요.",
        ) from e
    except OSError as e:
        raise GeortConfigError(
            "GEORT_CONFIG_MISSING",
            f"GeoRT 설정 파일을 읽을 수 없습니다: {path} ({e.strerror or e})",
            "파일 경로와 읽기 권한을 확인하세요.",
        ) from e
    except yaml.YAMLError as e:
        raise _invalid(path, "YAML 문법 오류") from e
    if not isinstance(raw, dict):
        raise _invalid(path, "최상위가 키-값 목록이 아닙니다")
    missing = [k for k in ("name", "side") if not isinstance(raw.get(k), str)]
    if missing:
        raise _invalid(path, f"{', '.join(missing)} 항목이 없습니다")
    if raw.get("model") is not None and not isinstance(raw.get("model"), str):
        raise _invalid(path, "model 은 문자열 경로여야 합니다")
    alpha = raw.get("low_pass_alpha", 1.0)
    if isinstance(alpha, bool) or not isinstance(alpha, (int, float)) or not 0.0 < alpha <= 1.0:
        raise _invalid(path, "low_pass_alpha는 0보다 크고 1 이하인 수여야 합니다")
    return GeortConfig(
        raw["name"], raw["side"], _model_path(raw.get("model"), path.parent), float(alpha)
    )


@functools.lru_cache(maxsize=4)
def _cached_prompt_model(path: str, mtime_ns: int) -> PromptModel:
    return load_prompt_model(Path(path))


def _load_prompt_model_cached(path: Path) -> PromptModel:
    """같은 모델 파일은 한 번만 읽는다(양손·여러 솔버).

    파일이 바뀌면(mtime) 다시 읽는다. 모델은 불변이라 솔버끼리 공유해도 된다.
    형식 검사는 load_prompt_model 이 한다.
    """
    path = path.expanduser()
    try:
        mtime = path.stat().st_mtime_ns
    except OSError:
        return load_prompt_model(
            path
        )  # 없는 파일 — load_prompt_model 이 GEORT_MODEL_MISSING 을 낸다
    return _cached_prompt_model(str(path.resolve()), mtime)


def load_embodiment(cfg: GeortConfig) -> Embodiment:
    """설정이 가리키는 모델에서 로봇 손 하나를 꺼낸다."""
    model = _load_prompt_model_cached(cfg.model_path)
    embodiment = model.embodiments.get(cfg.name)
    if embodiment is None or embodiment.kind != KIND_ROBOT or model.side != cfg.side:
        raise GeortConfigError(
            "GEORT_MODEL_MISMATCH",
            f"GeoRT 프롬프트 모델 {cfg.model_path.name}에 {cfg.side} 손 "
            f"'{cfg.name}'용 프롬프트가 없습니다 "
            f"(있는 것: {', '.join(sorted(model.embodiments))})",
            "설정 파일의 name이 모델에 학습된 손 이름인지 확인하세요.",
        )
    return embodiment


def masked_keypoints(keypoints: np.ndarray, valid: Optional[np.ndarray]) -> np.ndarray:
    """무효 관절의 위치를 NaN 으로 — 원점(항등)으로 남은 관절이 솔버에 들어가지 않게 한다."""
    kp = np.array(keypoints, dtype=np.float64, copy=True)
    if valid is not None:
        kp[~np.asarray(valid, dtype=bool), POSITION] = np.nan
    return kp
