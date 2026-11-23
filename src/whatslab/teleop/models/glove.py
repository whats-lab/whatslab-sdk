from __future__ import annotations

import logging
from typing import Dict, Optional

from whatslab.core.constants import SIDES
from whatslab.core.types import Pose
from whatslab.receiver.glove.airglove import AirGloveReceiver
from whatslab.receiver.webxr.controller import WebXRControllerReceiver

from ..base import TeleopModel

logger = logging.getLogger(__name__)

_OPPOSITE = {"left": "right", "right": "left"}

HAND_SOURCES = {
    "airglove": AirGloveReceiver,
}

ARM_SOURCES = {
    "webxr": WebXRControllerReceiver,
}


class GloveModel(TeleopModel):

    def __init__(self, robot, hand_source: str = "airglove",
                 arm_source: str = "webxr", arm_kwargs: Optional[dict] = None,
                 hand_kwargs: Optional[dict] = None):
        if hand_source not in HAND_SOURCES:
            raise ValueError(
                f"hand_source 는 {list(HAND_SOURCES)} 중 하나 — 받은 값 {hand_source!r}")
        if arm_source not in ARM_SOURCES:
            raise ValueError(
                f"arm_source 는 {list(ARM_SOURCES)} 중 하나 — 받은 값 {arm_source!r}")
        self.hand_source = HAND_SOURCES[hand_source](**(hand_kwargs or {}))
        self.arm_source = ARM_SOURCES[arm_source](**(arm_kwargs or {}))
        super().__init__(robot)

    def _get_raw_target(self) -> Dict[str, Optional[Pose]]:
        out: Dict[str, Optional[Pose]] = {}
        samples = self.samples()
        for s in SIDES:
            hand = samples[s][1].hand
            arm_s = samples[_OPPOSITE[s]][0]
            ctrl = arm_s.controller
            if ctrl is None or not arm_s.tracked:
                out[s] = None
                continue
            if hand is not None and hand.wrist is not None:
                # 세션(방) 기준 손목 방위를 컨트롤러 자세와 같은 pos_frame 으로 옮긴다.
                quat = hand.wrist.quat
                if arm_s.hmd is not None:
                    quat = self.arm_source.relative_quat(quat, arm_s.hmd.quat, True)
            else:
                self._warn_no_glove_wrist(s)
                quat = ctrl.quat
            out[s] = Pose(ctrl.pos, quat)
        return out

    def _warn_no_glove_wrist(self, side: str) -> None:
        seen = getattr(self, "_no_wrist_warned", None)
        if seen is None:
            seen = self._no_wrist_warned = set()
        if side not in seen:
            seen.add(side)
            logger.warning(
                "[%s] 글러브 손목 자세 없음 — 팔 회전을 컨트롤러 회전으로 대체"
                "(airglove 소스는 손목 자세를 내지 않는다 — 원시값은 raw_wrist)",
                side,
            )

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
