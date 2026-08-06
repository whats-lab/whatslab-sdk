from __future__ import annotations

from whatslab.receiver.glove.human_hand import GloveHumanHandReceiver

from .base import TeleopModel


class HandModel(TeleopModel):

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
