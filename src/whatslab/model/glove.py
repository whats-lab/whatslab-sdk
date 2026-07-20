"""GloveModel — 글러브 손(리타게팅+손목 회전) + Quest 컨트롤러 팔(위치) + 햅틱.

한 손엔 글러브(손가락+손목 회전), 반대 손엔 Quest 컨트롤러(팔 위치)를 든다 —
물리적으로 같은 손에 둘 다 둘 수 없으므로 **글러브 손 ≠ 컨트롤러 손**(크로스핸드).

출력 로봇 side = **글러브 side**. 그 side 팔 목표 = 컨트롤러(반대 side) 위치 +
글러브 손목 회전(머리-상대). 반대 side 는 `None`(구동 안 함). 손가락 리타게팅은 base
가 `hand_source.get(글러브 side)` 로 정렬한다.
"""
from __future__ import annotations

from typing import Dict, Optional

import numpy as np
from scipy.spatial.transform import Rotation

from whatslab.core.types import Pose
from whatslab.receiver.glove_human_hand import GloveHumanHandReceiver
from whatslab.receiver.quest_controller import QuestControllerReceiver

from .base import TeleopModel

_OPPOSITE = {"left": "right", "right": "left"}


class GloveModel(TeleopModel):
    """글러브(손) + Quest 컨트롤러(팔) + 햅틱 프리셋. `GloveModel(rig)` 한 줄."""

    def __init__(self, robot):
        self.hand_source = GloveHumanHandReceiver()
        self.arm_source = QuestControllerReceiver()
        super().__init__(robot)

    def _get_raw_target(self) -> Dict[str, Optional[Pose]]:
        """글러브 side 팔 목표 = 컨트롤러(반대 side) 위치 + 글러브 손목 회전(머리-상대).
        반대 side 는 None. 컨트롤러 미추적이면 None(→ 그 side IK 스킵)."""
        
        out: Dict[str, Optional[Pose]] = {}
        for s in self.SIDES:                              # s = 출력 side = 글러브 side
            hand = self.hand_source.get(s).hand           # 손목/손가락 = 이 side 글러브
            arm_s = self.arm_source.get(_OPPOSITE[s])     # 팔 위치 = 반대 손 컨트롤러
            ctrl = arm_s.controller
            if ctrl is not None and hand.wrist is not None:
                hmd = arm_s.hmd.quat if arm_s.hmd is not None else None
                quat = self._head_relative(hand.wrist.quat, hmd)   # 몸 회전 불변(머리 yaw 상대)
                out[s] = Pose(ctrl.pos, quat)
        return out

    @staticmethod
    def _head_relative(quat, hmd_quat):
        """손목 quat 을 **머리 yaw 기준 상대**로 (Rz(-h)·G). 몸/의자를 돌려 머리와 손이
        함께 회전하면 상대 회전 불변. hmd 없으면 원본 그대로."""
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
        """접촉/힘 피드백 → 글러브 햅틱 OSC 송신(`send_haptic` 위임).

        `data`: `{"side": .., "forces": [.]*5}` 기대. dict 아니면 세기 리스트로 보고
        글러브 side. TODO(프로토콜 미확인): 세기 스케일/단위는 기존 AirGlove 포맷 재사용.
        """
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
