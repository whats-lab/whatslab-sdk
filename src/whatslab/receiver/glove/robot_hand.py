from __future__ import annotations

import logging
import time
from typing import Dict, Optional

import numpy as np

from whatslab.core.types import HandPose, InputSample, Pose
from .base import GLOVE_CLIENT_PORT, GLOVE_OSC_PORT, GLOVE_TARGET_IP, GloveReceiverBase

logger = logging.getLogger(__name__)


class GloveRobotHandReceiver(GloveReceiverBase):

    def __init__(
        self,
        joint_map: Optional[Dict[str, str]] = None,
        glove_port: int = GLOVE_OSC_PORT,
        listen_ip: str = "0.0.0.0",
        target_ip: str = GLOVE_TARGET_IP,
        client_port: int = GLOVE_CLIENT_PORT,
        stale_timeout: float = 0.0,
        on_update: Optional[callable] = None,
    ):
        super().__init__(glove_port, listen_ip, target_ip, client_port)
        self.joint_map = dict(joint_map) if joint_map else None
        self._stale_timeout = stale_timeout
        self._on_update = on_update

        for side in ("left", "right"):
            s = self._state[side]
            s["q"] = None
            s["wrist"] = None
            s["timestamp"] = 0.0
            self._srv.add_handler(f"/{side}/joint_angles/get", self._h_joint_angles, side)
            self._srv.add_handler(f"/{side}/wrist/get", self._h_wrist, side)

    def get(self, side: str) -> InputSample:
        with self._lock:
            s = self._state[side]
            q = dict(s["q"]) if s["q"] is not None else None
            wrist = s["wrist"]
            wrist = None if wrist is None else wrist.copy()
            ts = s["timestamp"]
            conn = self._connected[side]
        age = time.monotonic() - ts
        tracked = conn and not (self._stale_timeout > 0 and age > self._stale_timeout)
        hand = None
        if wrist is not None:
            hand = HandPose(wrist=Pose(quat=wrist), tracked=False, timestamp=ts)
        return InputSample(hand=hand, joint_q=q, tracked=tracked, timestamp=ts)

    def _h_joint_angles(self, address, *args):
        side, rest = self._split(args)
        pairs = rest[1:]
        if len(pairs) < 2:
            return
        q: Dict[str, float] = {}
        prefix = f"{side}_"
        for i in range(0, len(pairs) - 1, 2):
            name, val = pairs[i], pairs[i + 1]
            if not isinstance(name, str):
                return
            try:
                v = float(val)
            except (TypeError, ValueError):
                return
            if name.startswith(prefix):
                name = name[len(prefix):]
            if self.joint_map is not None:
                name = self.joint_map.get(name)
                if name is None:
                    continue
            q[name] = v
        if not q:
            return
        self._commit(side, "q", q)

    def _h_wrist(self, address, *args):
        side, rest = self._split(args)
        quat = rest[1:]
        if len(quat) < 4:
            return
        try:
            raw = np.asarray(quat[:4], dtype=float)
        except (TypeError, ValueError):
            return
        if not np.all(np.isfinite(raw)) or np.linalg.norm(raw) < 1e-9:
            return
        self._commit(side, "wrist", raw)

    def _commit(self, side: str, key: str, value) -> None:
        with self._lock:
            s = self._state.get(side)
            if s is None:
                return
            s[key] = value
            s["timestamp"] = time.monotonic()
            self._connected[side] = True
        if self._on_update is not None:
            self._on_update(side)
