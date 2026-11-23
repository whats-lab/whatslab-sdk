from __future__ import annotations

import logging
import os
from typing import List, Optional

import numpy as np

from whatslab.core.types import HandCommand, HandPose, InputSample

from .xr_input import HandInputError, masked_keypoints

_log = logging.getLogger(__name__)

DEX_PREFIX = "dex:"
GEORT_PREFIX = "geort:"
ENGINES = (DEX_PREFIX, GEORT_PREFIX)


def engine_of(config_name) -> str:
    if isinstance(config_name, str):
        for prefix in ENGINES:
            if config_name.startswith(prefix) and len(config_name) > len(prefix):
                return prefix
    raise ValueError(
        "손 리타게팅 이름은 dex:<손> (DexPilot) 또는 geort:<설정> (GeoRT) — "
        f"받은 값 {config_name!r}"
    )


def geort_config_path(name: str) -> str:
    if name.endswith((".yaml", ".yml")):
        return name
    from whatslab.paths import configs_root

    return os.path.join(configs_root(), "geort", f"{name}.yaml")


class HandRetargetController:

    def __init__(
        self,
        hand_type: str,
        config_name: str = "dex:orca",
        urdf_root=None,
        **kwargs,
    ):
        self.engine_name = engine_of(config_name)
        name = config_name[len(self.engine_name):]
        if self.engine_name == DEX_PREFIX:
            from .dex_retargeter import XRHandRetargeter

            self._engine = XRHandRetargeter(name, side=hand_type, urdf_root=urdf_root, **kwargs)
            self._run = self._engine.compute
        else:
            from .geort import GeortHandSolver

            self._engine = GeortHandSolver.from_config(
                geort_config_path(name), human_side=hand_type, **kwargs
            )
            self._run = self._engine.solve
        lim = self._engine.joint_limits
        self._last = np.clip(np.zeros(len(self._engine.joint_names)), lim[:, 0], lim[:, 1])
        self._tracking = False
        self.last_error: Optional[ValueError] = None

    @property
    def joint_names(self) -> List[str]:
        return list(self._engine.joint_names)

    @property
    def engine(self):
        return self._engine

    def reset(self) -> None:
        self._engine.reset()
        self._tracking = False
        self.last_error = None

    @property
    def robot_side(self) -> str:
        if self.engine_name == DEX_PREFIX:
            return self._engine.config.side
        return self._engine.model.side

    def _solve(self, hand: HandPose) -> np.ndarray:
        return self._run(masked_keypoints(hand.xr_joints, hand.xr_valid))

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
