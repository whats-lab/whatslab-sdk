from __future__ import annotations

from typing import Dict, Optional

from whatslab.core.types import Pose
from whatslab.receiver.quest.hand import QuestHandReceiver

from .base import TeleopModel


class QuestModel(TeleopModel):

    def __init__(self, robot):
        rx = QuestHandReceiver()
        self.arm_source = rx
        self.hand_source = rx
        super().__init__(robot)

    def _get_raw_target(self) -> Dict[str, Optional[Pose]]:
        out: Dict[str, Optional[Pose]] = {}
        for s in self.SIDES:
            hand = self.arm_source.get(s).hand
            if hand is not None and hand.tracked and hand.wrist is not None:
                out[s] = Pose(hand.wrist.pos, hand.wrist.quat)
            else:
                out[s] = None
        return out
