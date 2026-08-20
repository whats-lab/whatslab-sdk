from __future__ import annotations

import time

import numpy as np

from whatslab.core.types import InputSample, Pose
from ..base import norm_quat
from .base import QUEST_OSC_PORT, QuestReceiverBase

CONTROLLER_POS_OFFSET = np.array([0.02, -0.04, 0.1])


class QuestControllerReceiver(QuestReceiverBase):

    def __init__(self, quest_port: int = QUEST_OSC_PORT, listen_ip: str = "0.0.0.0",
                 stale_timeout: float = 0.0):
        super().__init__(quest_port, listen_ip)
        self._stale_timeout = stale_timeout
        for side in ("left", "right"):
            s = self._state[side]
            s["pos"] = np.zeros(3)
            s["quat"] = np.array([0.0, 0.0, 0.0, 1.0])
            s["valid"] = False
            s["timestamp"] = 0.0
            self._srv.add_handler(f"/controller/{side}/pos", self._on_pos, side)
            self._srv.add_handler(f"/controller/{side}/rot", self._on_rot, side)

    def _on_pos(self, address, *args):
        side, v = self._split(args)
        with self._lock:
            s = self._state[side]
            s["pos"] = np.array(v[:3], dtype=float)
            s["valid"] = True
            s["timestamp"] = time.monotonic()

    def _on_rot(self, address, *args):
        side, v = self._split(args)
        with self._lock:
            s = self._state[side]
            s["quat"] = norm_quat(v[:4])
            s["valid"] = True
            s["timestamp"] = time.monotonic()

    def get(self, side: str) -> InputSample:
        with self._lock:
            s = self._state[side]
            pos = s["pos"].copy()
            quat = s["quat"].copy()
            valid = s["valid"]
            ts = s["timestamp"]
        hmd_quat, hmd_valid = self.get_hmd()
        age = time.monotonic() - ts
        tracked = valid and not (self._stale_timeout > 0 and age > self._stale_timeout)
        if valid:
            pos, quat = self.to_canonical(pos, quat)
        controller = Pose(pos + CONTROLLER_POS_OFFSET, quat) if valid else None
        hmd = Pose(np.zeros(3), hmd_quat) if hmd_valid else None
        return InputSample(controller=controller, hmd=hmd, tracked=tracked, timestamp=ts)

    def connected(self, side: str) -> bool:
        return self.get(side).tracked
