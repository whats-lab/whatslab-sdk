from __future__ import annotations

from typing import List

import numpy as np

from whatslab.core.types import HandCommand, InputSample
from .uni_retargeter import UniRetargeter


class HandRetargetController:

    def __init__(self, hand_type: str, config_name: str = "base_hand",
                 urdf_root=None, **kwargs):
        kwargs.pop("backend", None)
        self._engine = UniRetargeter(hand_type, config_name,
                                     urdf_root=urdf_root, **kwargs)
        self._last = np.zeros(len(self._engine.joint_names))

    @property
    def joint_names(self) -> List[str]:
        return self._engine.joint_names

    @property
    def engine(self) -> UniRetargeter:
        return self._engine

    def compute(self, sample: InputSample) -> HandCommand:
        if sample.hand is None or not sample.hand.tracked:
            return HandCommand(joint_names=self.joint_names,
                               joint_angles=self._last.copy())
        if not sample.hand.joint_angles:
            return HandCommand(joint_names=self.joint_names,
                               joint_angles=self._last.copy())
        qpos = self._engine.compute(sample.hand.joint_angles)
        self._last = qpos
        return HandCommand(joint_names=self.joint_names, joint_angles=qpos)
