from __future__ import annotations

import logging
from typing import Dict, Optional

import numpy as np
from scipy.spatial.transform import Rotation

from whatslab.core.types import Pose
from whatslab.receiver.glove.human_hand import GloveHumanHandReceiver
from whatslab.receiver.quest.controller import QuestControllerReceiver

from .base import TeleopModel

logger = logging.getLogger(__name__)

_OPPOSITE = {"left": "right", "right": "left"}


class GloveModel(TeleopModel):

    def __init__(self, robot):
        self.hand_source = GloveHumanHandReceiver()
        self.arm_source = QuestControllerReceiver()
        super().__init__(robot)

    def _get_raw_target(self) -> Dict[str, Optional[Pose]]:
        
        out: Dict[str, Optional[Pose]] = {}
        for s in self.SIDES:                              # s = 출력 side = 글러브 side
            hand = self.hand_source.get(s).hand           # 손목/손가락 = 이 side 글러브 (None 가능)
            arm_s = self.arm_source.get(_OPPOSITE[s])     # 팔 위치 = 반대 손 컨트롤러
            ctrl = arm_s.controller
            if ctrl is None:
                out[s] = None
                continue
            hmd = arm_s.hmd.quat if arm_s.hmd is not None else None
            if hand is not None and hand.wrist is not None:
                quat = self._head_relative(hand.wrist.quat, hmd)   # 몸 회전 불변(머리 yaw 상대)
            else:
                # 글러브 손목이 없으면 팔 목표를 아예 못 만들어 팔이 멈춘다. 컨트롤러
                # 자체 회전으로 강등해 Quest 단독으로도 동작하게 하되, 조용히 넘어가지
                # 않도록 한 번은 알린다(글러브 미연결과 정상 동작을 구분해야 한다).
                self._warn_no_glove_wrist(s)
                quat = self._head_relative(ctrl.quat, hmd)
            out[s] = Pose(ctrl.pos, quat)
        return out

    def _warn_no_glove_wrist(self, side: str) -> None:
        seen = getattr(self, "_no_wrist_warned", None)
        if seen is None:
            seen = self._no_wrist_warned = set()
        if side not in seen:
            seen.add(side)
            logger.warning("[%s] 글러브 손목 신호 없음 — 팔 자세를 컨트롤러 회전으로 대체", side)

    @staticmethod
    def _head_relative(quat, hmd_quat):
        if hmd_quat is None:
            return quat
        G = Rotation.from_quat(np.asarray(quat, dtype=float)).as_matrix()
        H = Rotation.from_quat(np.asarray(hmd_quat, dtype=float)).as_matrix()
        h = float(np.arctan2(H[1, 0], H[0, 0]))                # 머리 yaw
        c, s = np.cos(-h), np.sin(-h)
        Rz = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
        return Rotation.from_matrix(Rz @ G).as_quat()

    # -------------------------------------------------------------------- haptics
    def send_feedback(self, data) -> None:
        if data is None:
            return
        if isinstance(data, dict):
            side = data.get("side", "right")
            values = data.get("forces") or data.get("values")
        else:
            side, values = "right", data
        if not values:
            return
        self.hand_source.send_haptic(side, list(values))
