from __future__ import annotations

from typing import List, Union

import numpy as np

from whatslab.core.types import HandCommand, InputSample
from .dex_retargeter import DEX_PREFIX, XRHandRetargeter, is_dex_config
from .uni_retargeter import UniRetargeter


class HandRetargetController:

    def __init__(self, hand_type: str, config_name: str = "base_hand",
                 urdf_root=None, **kwargs):
        self.xr_input = is_dex_config(config_name)
        if self.xr_input:
            self._engine: Union[XRHandRetargeter, UniRetargeter] = XRHandRetargeter(
                config_name[len(DEX_PREFIX):], side=hand_type, urdf_root=urdf_root, **kwargs)
            lim = self._engine.joint_limits
            self._last = np.clip(np.zeros(len(self._engine.joint_names)), lim[:, 0], lim[:, 1])
        else:
            self._engine = UniRetargeter(hand_type, config_name,
                                         urdf_root=urdf_root, **kwargs)
            self._last = np.zeros(len(self._engine.joint_names))

    @property
    def joint_names(self) -> List[str]:
        return self._engine.joint_names

    @property
    def engine(self) -> Union[XRHandRetargeter, UniRetargeter]:
        return self._engine

    def _hold(self) -> HandCommand:
        return HandCommand(joint_names=self.joint_names, joint_angles=self._last.copy())

    def compute(self, sample: InputSample) -> HandCommand:
        hand = sample.hand
        if hand is None or not hand.tracked:
            return self._hold()
        if self.xr_input:
            if hand.xr_joints is None:
                return self._hold()
            qpos = self._engine.compute(hand.xr_joints)
        else:
            if not hand.joint_angles:
                return self._hold()
            qpos = self._engine.compute(hand.joint_angles)
        self._last = qpos
        return HandCommand(joint_names=self.joint_names, joint_angles=qpos)
