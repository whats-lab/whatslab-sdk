from __future__ import annotations

from typing import Dict, Optional

from whatslab.core.constants import SIDES
from whatslab.core.types import Pose
from whatslab.receiver.webxr.hand import WebXRHandReceiver

from ..base import TeleopModel


class QuestModel(TeleopModel):
    """Quest 맨손 텔레옵(브라우저 WebXR 핸드트래킹): 손목 → 팔 IK 목표, 손 26관절 → 손 리타게팅.

    손목 위치·방위는 수신기의 pos_frame 기준이다(기본 hmd_yaw — 둘 다 머리 yaw 를 뺀다).
    """

    def __init__(self, robot, hand_kwargs: Optional[dict] = None):
        rx = WebXRHandReceiver(**(hand_kwargs or {}))
        self.arm_source = rx
        self.hand_source = rx
        super().__init__(robot)

    def _get_raw_target(self) -> Dict[str, Optional[Pose]]:
        out: Dict[str, Optional[Pose]] = {}
        samples = self.samples()
        for s in SIDES:
            hand = samples[s][1].hand
            if hand is not None and hand.tracked and hand.wrist is not None:
                out[s] = Pose(hand.wrist.pos, hand.wrist.quat)
            else:
                out[s] = None
        return out
