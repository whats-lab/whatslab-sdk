from __future__ import annotations

import logging
from typing import List, Optional, Protocol, Sequence, runtime_checkable

import numpy as np

from whatslab.core.types import HandCommand, HandPose, InputSample

_log = logging.getLogger(__name__)

DEX_PREFIX = "dex:"
GEORT_PREFIX = "geort:"
ENGINES = (DEX_PREFIX, GEORT_PREFIX)


class HandInputError(ValueError):
    pass


@runtime_checkable
class Engine(Protocol):
    joint_names: Sequence[str]
    joint_limits: np.ndarray

    def solve(self, xr_joints: np.ndarray) -> np.ndarray: ...
    def reset(self) -> None: ...


def masked_keypoints(keypoints: np.ndarray, valid: Optional[np.ndarray]) -> np.ndarray:
    kp = np.array(keypoints, dtype=np.float64, copy=True)
    if valid is not None:
        kp[~np.asarray(valid, dtype=bool), 4:7] = np.nan
    return kp


def engine_of(config) -> str:
    if isinstance(config, str):
        for prefix in ENGINES:
            if config.startswith(prefix) and len(config) > len(prefix):
                return prefix
    raise ValueError(
        "손 리타게팅 이름은 dex:<손> (DexPilot) 또는 geort:<설정> (GeoRT) — "
        f"받은 값 {config!r}"
    )


class HandRetargeter:

    def __init__(
        self,
        config: str = "dex:orca",
        side: str = "right",
        urdf_root=None,
        **kwargs,
    ):
        self.engine_name = engine_of(config)
        name = config[len(self.engine_name):]
        if self.engine_name == DEX_PREFIX:
            from .dex import DexEngine

            self._engine: Engine = DexEngine(name, side=side, urdf_root=urdf_root, **kwargs)
        else:
            from .geort import GeortEngine

            self._engine = GeortEngine.from_config(name, side=side, **kwargs)
        lim = self._engine.joint_limits
        self._last = np.clip(np.zeros(len(self._engine.joint_names)), lim[:, 0], lim[:, 1])
        self._tracking = False
        self.last_error: Optional[ValueError] = None

    @property
    def joint_names(self) -> List[str]:
        return list(self._engine.joint_names)

    @property
    def engine(self) -> Engine:
        return self._engine

    def reset(self) -> None:
        self._engine.reset()
        self._tracking = False
        self.last_error = None

    def _solve(self, hand: HandPose) -> np.ndarray:
        return self._engine.solve(masked_keypoints(hand.xr_joints, hand.xr_valid))

    def compute(self, sample: InputSample) -> HandCommand:
        hand = sample.hand
        if hand is None or not hand.tracked or hand.xr_joints is None:
            if self._tracking:
                self.reset()
            self.last_error = None
        else:
            try:
                self._last = np.asarray(self._solve(hand), dtype=float)
                self._tracking = True
                self.last_error = None
            except HandInputError as e:
                first = self.last_error is None
                if first:
                    _log.warning("손 리타게팅 입력을 풀 수 없어 직전 명령을 유지한다: %s", e)
                self.reset()
                self.last_error = e
        return HandCommand(joint_names=self.joint_names, joint_angles=self._last.copy())
