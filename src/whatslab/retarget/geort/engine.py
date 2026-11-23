from __future__ import annotations

import dataclasses
import functools
import os
from pathlib import Path
from typing import Optional, Tuple

import numpy as np
import yaml

from whatslab.core.constants import OPENXR26_TO_HAND21, SIDES, XR_JOINT_COUNT, XR_JOINT_FIELDS
from whatslab.paths import configs_root

from .errors import GeortConfigError, GeortInputError
from .preprocess import POSITION
from .prompt import Embodiment, PromptModel, load_prompt_model

KEYPOINT_SHAPE = (XR_JOINT_COUNT, XR_JOINT_FIELDS)
DEFAULT_MODEL = str(Path(__file__).resolve().parent / "assets" / "geort_prompt.npz")
MODEL_ENV = "WHATSLAB_GEORT_MODEL"

_REMEDY_TRACKING = "장갑 착용 상태와 손 추적 상태를 확인하세요."
_REMEDY_FILE = "GeoRT 설정 YAML(name, side, model)과 모델 파일 경로를 확인하세요."


class GeortEngine:

    def __init__(
        self,
        model: Embodiment,
        side: str = "right",
        low_pass_alpha: float = 1.0,
    ) -> None:
        if side not in SIDES:
            raise GeortConfigError(
                "GEORT_CONFIG_INVALID", f"side {side!r}는 left|right 여야 합니다"
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
        self._mirror = side != model.side
        self._limits = np.column_stack([model.lower, model.upper])
        self._limits.setflags(write=False)

    @classmethod
    def from_config(
        cls,
        config: "str | os.PathLike[str]",
        side: str = "right",
        low_pass_alpha: Optional[float] = None,
    ) -> "GeortEngine":
        cfg = read_config(config)
        alpha = cfg.low_pass_alpha if low_pass_alpha is None else float(low_pass_alpha)
        return cls(load_embodiment(cfg), side, alpha)

    @property
    def joint_names(self) -> Tuple[str, ...]:
        return self._model.joint_names

    @property
    def joint_limits(self) -> np.ndarray:
        return self._limits

    @property
    def model(self) -> Embodiment:
        return self._model

    def reset(self) -> None:
        self._last = None

    def keypoints(self, keypoints: np.ndarray) -> np.ndarray:
        kp = np.asarray(keypoints, dtype=np.float64)
        if kp.shape != KEYPOINT_SHAPE:
            raise ValueError(
                f"geort engine expects keypoints of shape {KEYPOINT_SHAPE} "
                f"(openxr26 layout), got {kp.shape}"
            )
        if not np.all(
            np.isfinite(kp[list(OPENXR26_TO_HAND21), POSITION])
        ):
            raise GeortInputError(
                "GEORT_INPUT_NON_FINITE",
                "손 추적 위치에 유한하지 않은 값이 있어 리타게팅하지 않았습니다",
                _REMEDY_TRACKING,
            )
        points = self._model.frame.keypoints(kp, mirror=self._mirror)
        if not np.all(np.isfinite(points)):
            raise GeortInputError(
                "GEORT_SOLVE_FAILED",
                "손목과 손가락 뿌리 위치로 손바닥 방향을 정할 수 없습니다",
                _REMEDY_TRACKING,
            )
        return points

    def solve(self, keypoints: np.ndarray) -> np.ndarray:
        q = np.clip(
            self._model.forward(self.keypoints(keypoints)), self._limits[:, 0], self._limits[:, 1]
        )
        if self._alpha < 1.0:
            self._last = q if self._last is None else self._last + self._alpha * (q - self._last)
            return self._last.copy()
        return q


@dataclasses.dataclass(frozen=True)
class GeortConfig:
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
    if raw:
        model = Path(raw).expanduser()
        return model if model.is_absolute() else base / model
    return Path(os.environ.get(MODEL_ENV) or DEFAULT_MODEL).expanduser().resolve()


def config_path(config: "str | os.PathLike[str]") -> Path:
    if not str(config).endswith((".yaml", ".yml")):
        return Path(configs_root()) / "geort" / f"{config}.yaml"
    p = Path(config).expanduser()
    if p.is_absolute() or p.exists():
        return p
    alt = Path(configs_root()) / p
    return alt if alt.exists() else p


def read_config(config: "str | os.PathLike[str]") -> GeortConfig:
    path = config_path(config)
    raw = yaml.safe_load(path.read_text(encoding="utf-8"))
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
    path = path.expanduser()
    return _cached_prompt_model(str(path.resolve()), path.stat().st_mtime_ns)


def load_embodiment(cfg: GeortConfig) -> Embodiment:
    model = _load_prompt_model_cached(cfg.model_path)
    embodiment = model.embodiments.get(cfg.name)
    if embodiment is None or model.side != cfg.side:
        raise GeortConfigError(
            "GEORT_MODEL_MISMATCH",
            f"GeoRT 프롬프트 모델 {cfg.model_path.name}에 {cfg.side} 손 "
            f"'{cfg.name}'용 프롬프트가 없습니다 "
            f"(있는 것: {', '.join(sorted(model.embodiments))})",
            "설정 파일의 name이 모델에 학습된 손 이름인지 확인하세요.",
        )
    return embodiment

