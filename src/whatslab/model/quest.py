"""QuestModel — Meta Quest 손 트래킹 프리셋 TeleopModel.

QuestHandReceiver 하나(손목 pose + 손가락 joints)를 팔·손 공용 소스로 쓴다.
팔 목표 = 핸드트래킹 손목 pose, 손 = 손가락 리타게팅. 햅틱 없음.

Quest 손목 자세는 이미 (HMD 기준) 상대 각도로 들어오므로 head-relative 보정을
적용하지 않는다 — 손목 pose 를 그대로 팔 EE 목표로 쓴다.

OSC 포트 등 네트워크 설정은 바뀔 일이 드물어 생성자 인자로 받지 않는다 —
바꿔야 하면 여기(리시버 생성부)에서 직접 수정한다.
"""
from __future__ import annotations

from typing import Dict, Optional

from whatslab.core.types import Pose
from whatslab.receiver.quest_hand import QuestHandReceiver

from .base import TeleopModel


class QuestModel(TeleopModel):
    """Quest 핸드트래킹 프리셋. robot(rig 하나 또는 [left, right])만 주면 끝."""

    def __init__(self, robot):
        rx = QuestHandReceiver()          # 포트=기본값(quest_base.QUEST_OSC_PORT)
        self.arm_source = rx
        self.hand_source = rx
        super().__init__(robot)

    def _get_raw_target(self) -> Dict[str, Optional[Pose]]:
        """양손 팔 목표 = 각 side 핸드트래킹 손목 pose(그대로). 미추적 side 는 None."""
        out: Dict[str, Optional[Pose]] = {}
        for s in self.SIDES:
            hand = self.arm_source.get(s).hand
            if hand is not None and hand.tracked and hand.wrist is not None:
                out[s] = Pose(hand.wrist.pos, hand.wrist.quat)
            else:
                out[s] = None
        return out
