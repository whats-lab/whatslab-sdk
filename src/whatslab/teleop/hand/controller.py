"""HandController 어댑터: InputSample(HandPose) → HandCommand.

저수준 엔진 HandRetargeter(센서 17×4 배열 입력)를 core.HandController 프로토콜
(compute(InputSample)->HandCommand)에 맞춘다. 골격↔배열 변환은 HandPose 가
담당하므로(to_sensor_array), 이 경계에만 배열 규약이 남는다.
"""
from __future__ import annotations

from typing import List

import numpy as np

from whatslab.core.types import HandCommand, InputSample
from .retargeter import HandRetargeter


class HandRetargetController:
    """core.HandController 구현. HandPose 를 받아 로봇 손 관절각을 낸다."""

    def __init__(self, hand_type: str, config_name: str = "base_hand",
                 urdf_root=None, **kwargs):
        self._engine = HandRetargeter(hand_type, config_name, urdf_root=urdf_root, **kwargs)
        self._last = np.zeros(len(self._engine.joint_names))

    @property
    def joint_names(self) -> List[str]:
        return self._engine.joint_names

    @property
    def engine(self) -> HandRetargeter:
        return self._engine

    def compute(self, sample: InputSample) -> HandCommand:
        if sample.hand is None or not sample.hand.tracked:
            # 추적 없음 → 직전 명령 유지 (급변 방지)
            return HandCommand(joint_names=self.joint_names, joint_angles=self._last.copy())
        qpos = self._engine.compute(sample.hand.to_sensor_array())
        self._last = qpos
        return HandCommand(joint_names=self.joint_names, joint_angles=qpos)
