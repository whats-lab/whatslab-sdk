from __future__ import annotations

import time
from typing import List, Optional

import numpy as np

from whatslab.core.types import InputSample
from .base import GLOVE_CLIENT_PORT, GLOVE_OSC_PORT, GLOVE_TARGET_IP, GloveReceiverBase


class GloveRobotHandReceiver(GloveReceiverBase):

    def __init__(
        self,
        joint_names: List[str],
        glove_port: int = GLOVE_OSC_PORT,
        listen_ip: str = "0.0.0.0",
        target_ip: str = GLOVE_TARGET_IP,
        client_port: int = GLOVE_CLIENT_PORT,
        stale_timeout: float = 0.0,
        on_update: Optional[callable] = None,
    ):
        super().__init__(glove_port, listen_ip, target_ip, client_port)
        if not joint_names:
            raise ValueError("joint_names 는 비어있을 수 없다(로봇 q 순서 매핑 필수)")
        self.joint_names = list(joint_names)
        self._stale_timeout = stale_timeout
        # 새 프레임 수신 시 호출되는 콜백(side) — 이벤트 구동 소비자용(폴링 불필요)
        self._on_update = on_update

        for side in ("left", "right"):
            s = self._state[side]
            s["q"] = np.zeros(len(self.joint_names))
            s["timestamp"] = 0.0
            self._srv.add_handler(f"/glove/{side}/q", self._h_q, side)

    # ---------------------------------------------------------------- public
    def get(self, side: str) -> InputSample:
        with self._lock:
            s = self._state[side]
            q = s["q"].copy()
            ts = s["timestamp"]
            conn = self._connected[side]
        age = time.monotonic() - ts
        tracked = conn and not (self._stale_timeout > 0 and age > self._stale_timeout)
        joint_q = dict(zip(self.joint_names, (float(v) for v in q)))
        return InputSample(joint_q=joint_q, tracked=tracked, timestamp=ts)

    # ----------------------------------------------------------- OSC handlers
    def _h_q(self, address, *args):
        # dispatcher.map(address, self._h_q, side) 로 등록 — args[0]=side(주입값),
        # args[1:]=실제 OSC 메시지 인자(float q 배열, joint_names 순서).
        side = args[0]
        if isinstance(side, (list, tuple)):
            side = side[0]
        n = len(self.joint_names)
        raw = args[1:]
        if len(raw) < n:
            return
        try:
            q = np.array(raw[:n], dtype=float)
        except (TypeError, ValueError):
            return
        with self._lock:
            s = self._state[side]
            s["q"] = q
            s["timestamp"] = time.monotonic()
            self._connected[side] = True
        if self._on_update is not None:
            self._on_update(side)
