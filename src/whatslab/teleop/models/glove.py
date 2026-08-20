from __future__ import annotations

import logging
from typing import Dict, Optional

import numpy as np
from scipy.spatial.transform import Rotation

from whatslab.core.types import Pose
from whatslab.receiver.glove.human_hand import GloveHumanAnglesReceiver
from whatslab.receiver.glove.robot_hand import GloveRobotHandReceiver
from whatslab.receiver.quest.controller import QuestControllerReceiver

from ..base import TeleopModel

logger = logging.getLogger(__name__)

_OPPOSITE = {"left": "right", "right": "left"}

HAND_SOURCES = {
    "angles": GloveHumanAnglesReceiver,
    "robot": GloveRobotHandReceiver,
}


class GloveModel(TeleopModel):

    def __init__(self, robot, hand_source: str = "angles"):
        if hand_source not in HAND_SOURCES:
            raise ValueError(
                f"hand_source 는 {list(HAND_SOURCES)} 중 하나 — 받은 값 {hand_source!r}")
        self.hand_source = HAND_SOURCES[hand_source]()
        self.arm_source = QuestControllerReceiver()
        super().__init__(robot)

    def _get_raw_target(self) -> Dict[str, Optional[Pose]]:
        out: Dict[str, Optional[Pose]] = {}
        for s in self.SIDES:
            hand = self.hand_source.get(s).hand
            arm_s = self.arm_source.get(_OPPOSITE[s])
            ctrl = arm_s.controller
            if ctrl is None:
                out[s] = None
                continue
            hmd = arm_s.hmd.quat if arm_s.hmd is not None else None
            if hand is not None and hand.wrist is not None:
                quat = self._head_relative(hand.wrist.quat, hmd)
            else:
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
        h = float(np.arctan2(H[1, 0], H[0, 0]))
        c, s = np.cos(-h), np.sin(-h)
        Rz = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
        return Rotation.from_matrix(Rz @ G).as_quat()

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
