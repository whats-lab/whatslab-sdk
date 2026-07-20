"""HandModel — 손 전용 프리셋 TeleopModel (팔 없음).

글러브 손가락 → 로봇 손 리타게팅만. 팔 IK/rig 가 필요 없다(dex_retargeting 은
rig 의 IK 와 무관) → robot=None 으로 base 를 태우고 retarget 만 직접 채운다.
그래서 팔 IK 태스크가 아닌 손 전용 수집(예: 손만 데모)에서 GloveModel 대신
쓰고, 소비처는 동일한 폴링(get_q/ready)으로 다룰 수 있다.
"""
from __future__ import annotations

from whatslab.receiver.glove_human_hand import GloveHumanHandReceiver

from .base import TeleopModel


class HandModel(TeleopModel):
    """손 전용 프리셋. hand_config(리타게팅) + side 만 주면 바로 get_q(손 관절만)."""

    def __init__(self, hand_config: str = "orca_hand", side: str = "right",
                 urdf_root: str | None = None, hand_source=None):
        from whatslab.teleop.hand import HandRetargetController
        self._side = side
        # 손 소스: 기본 글러브(AirGlove). 다른 손 입력(예: Quest 핸드트래킹)은
        # QuestHandReceiver 등을 주입 — 둘 다 InputSample.hand(손가락 quat) 제공.
        self.hand_source = hand_source if hand_source is not None else GloveHumanHandReceiver()
        super().__init__(robot=None)          # rig 없음 → ik/calib 비어있음(팔 없음)
        # 리타게터 직접 주입(rig·make_hand_controller 경유 안 함).
        self.retarget = {side: HandRetargetController(side, hand_config,
                                                      urdf_root=urdf_root)}
