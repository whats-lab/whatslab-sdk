from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Optional

import numpy as np

from whatslab.core.constants import SIDES
from whatslab.core.types import Pose
from whatslab.receiver.base import Y_UP_RH_TO_CANONICAL, to_canonical_pose
from whatslab.receiver.webxr.base import WEBXR_PORT, WebXRReceiverBase, parse_pose


@dataclass
class PoseSample:
    controller: Optional[Pose] = None
    hmd: Optional[Pose] = None
    tracked: bool = False
    timestamp: float = 0.0


class PoseReceiver(WebXRReceiverBase):
    """테스트 전용: 메시지의 `left` / `right` 항목 `{pos, quat}` 을 받는 최소 WebXR 수신기.

    SDK 는 손만 받지만 서버·TLS·pos_frame 은 수신기 종류와 무관하므로 자세 하나로 검증한다.

    ``stale_timeout`` 초 동안 새 자세가 없으면 추적 아님 — 컨트롤러를 내려놓아 Quest 가 손
    추적으로 바뀌면 페이지가 그 쪽 grip 을 보내지 않으므로, 마지막 자세를 계속 추적 중으로 내지
    않게 한다. 0 이면 끄기. 위치와 방위는 둘 다 pos_frame 기준이다(hmd_yaw 면 둘 다 머리 yaw
    를 뺀다).
    """

    def __init__(
        self,
        port: int = WEBXR_PORT,
        listen_ip: str = "0.0.0.0",
        stale_timeout: float = 0.2,
        pos_offset=(0.0, 0.0, 0.0),
        pos_frame: str = "hmd_yaw",
        tls: bool = True,
        certfile: str = None,
        keyfile: str = None,
    ):
        super().__init__(port, listen_ip, tls, certfile, keyfile, pos_frame=pos_frame)
        self._stale_timeout = stale_timeout
        self._pos_offset = np.asarray(pos_offset, dtype=float)
        for side in SIDES:
            s = self._state[side]
            s["pos"] = np.zeros(3)
            s["quat"] = np.array([0.0, 0.0, 0.0, 1.0])
            s["valid"] = False
            s["timestamp"] = 0.0

    def _handle(self, msg: dict) -> None:
        for side in SIDES:
            pose = parse_pose(msg.get(side))
            if pose is None:
                continue
            with self._lock:
                s = self._state[side]
                s["pos"], s["quat"] = pose
                s["valid"] = True
                s["timestamp"] = time.monotonic()

    def get(self, side: str) -> PoseSample:
        """side 의 최신 컨트롤러 샘플(정준 프레임, pos_frame 기준)."""
        with self._lock:
            s = self._state[side]
            pos = s["pos"].copy()
            quat = s["quat"].copy()
            valid = s["valid"]
            ts = s["timestamp"]
        hmd_pos, hmd_quat, hmd_valid = self.get_hmd()
        age = time.monotonic() - ts
        tracked = valid and not (self._stale_timeout > 0 and age > self._stale_timeout)
        if valid:
            pos, quat = to_canonical_pose(Y_UP_RH_TO_CANONICAL, pos, quat)
            pos = self.relative(pos, hmd_pos, hmd_quat, hmd_valid)
            quat = self.relative_quat(quat, hmd_quat, hmd_valid)
        controller = Pose(pos + self._pos_offset, quat) if valid else None
        hmd = Pose(hmd_pos, hmd_quat) if hmd_valid else None
        return PoseSample(controller=controller, hmd=hmd, tracked=tracked, timestamp=ts)

    def connected(self, side: str) -> bool:
        return self.get(side).tracked
