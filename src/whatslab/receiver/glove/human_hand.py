from __future__ import annotations

import logging
import time
from typing import Optional

import numpy as np

from whatslab.core.types import HandPose, InputSample, Pose
from .base import (GLOVE_CLIENT_PORT, GLOVE_OSC_PORT, GLOVE_TARGET_IP,
                   GloveReceiverBase)

logger = logging.getLogger(__name__)


AGA_RAW_FLOAT_COUNT = 72
AGA_SKIP_JOINT      = 14


def parse_aga_raw(raw_floats) -> np.ndarray:
    arr = np.asarray(raw_floats, dtype=np.float32).reshape(18, 4)
    return np.delete(arr, AGA_SKIP_JOINT, axis=0)


def _neutral() -> np.ndarray:
    q = np.zeros((17, 4), dtype=np.float32)
    q[:, 3] = 1.0
    return q


class GloveHumanHandReceiver(GloveReceiverBase):
    def __init__(
        self,
        glove_port: int = GLOVE_OSC_PORT,
        listen_ip: str = "0.0.0.0",
        target_ip: str = GLOVE_TARGET_IP,
        client_port: int = GLOVE_CLIENT_PORT,
        stale_timeout: float = 0.0,
        on_update: Optional[callable] = None,
    ):
        super().__init__(glove_port, listen_ip, target_ip, client_port)
        self._stale_timeout = stale_timeout
        self._on_update = on_update

        for side in ("left", "right"):
            s = self._state[side]
            s["quats"] = _neutral()
            s["timestamp"] = 0.0
            self._srv.add_handler(f"/{side}/quat/get", self._h_quat, side)

    def get(self, side: str) -> InputSample:
        with self._lock:
            s = self._state[side]
            q = s["quats"].copy()
            ts = s["timestamp"]
            conn = self._connected[side]
        age = time.monotonic() - ts
        tracked = conn and not (self._stale_timeout > 0 and age > self._stale_timeout)
        hand = HandPose.from_sensor_array(q, wrist_pos=None, tracked=tracked, timestamp=ts)
        return InputSample(controller=None, hand=hand, tracked=tracked, timestamp=ts)

    def _h_quat(self, address, *args):
        side = args[0]
        if isinstance(side, (list, tuple)):
            side = side[0]
        raw = self._parse_floats(args[1:], AGA_RAW_FLOAT_COUNT)
        if raw is None:
            return
        quats = parse_aga_raw(raw)
        with self._lock:
            s = self._state[side]
            s["quats"] = quats
            s["timestamp"] = time.monotonic()
            self._connected[side] = True
        if self._on_update is not None:
            self._on_update(side)

    @staticmethod
    def _parse_floats(args, count) -> Optional[np.ndarray]:
        if len(args) < count + 1:
            return None
        try:
            return np.array(args[1:count + 1], dtype=np.float32)
        except (TypeError, ValueError):
            return None


def parse_joint_angle_pairs(pairs) -> Optional[dict]:
    if len(pairs) < 2:
        return None
    out = {}
    for i in range(0, len(pairs) - 1, 2):
        name, val = pairs[i], pairs[i + 1]
        if not isinstance(name, str):
            return None
        try:
            out[name] = float(val)
        except (TypeError, ValueError):
            return None
    return out or None


class GloveHumanAnglesReceiver(GloveReceiverBase):

    def __init__(
        self,
        glove_port: int = GLOVE_OSC_PORT,
        listen_ip: str = "0.0.0.0",
        target_ip: str = GLOVE_TARGET_IP,
        client_port: int = GLOVE_CLIENT_PORT,
        stale_timeout: float = 0.0,
        on_update: Optional[callable] = None,
    ):
        super().__init__(glove_port, listen_ip, target_ip, client_port)
        self._stale_timeout = stale_timeout
        self._on_update = on_update

        for side in ("left", "right"):
            s = self._state[side]
            s["angles"] = {}
            s["wrist"] = None
            s["timestamp"] = 0.0
            self._srv.add_handler(f"/{side}/joint_angles/get", self._h_angles, side)
            self._srv.add_handler(f"/{side}/wrist/get", self._h_wrist, side)

    def get(self, side: str) -> InputSample:
        with self._lock:
            s = self._state[side]
            angles = dict(s["angles"])
            wrist = s["wrist"]
            wrist = None if wrist is None else wrist.copy()
            ts = s["timestamp"]
            conn = self._connected[side]
        age = time.monotonic() - ts
        tracked = bool(angles) and conn and not (
            self._stale_timeout > 0 and age > self._stale_timeout)
        hand = HandPose(wrist=None if wrist is None else Pose(quat=wrist),
                        joint_angles=angles, tracked=tracked, timestamp=ts)
        return InputSample(hand=hand, tracked=tracked, timestamp=ts)

    def _h_angles(self, address, *args):
        side, rest = self._split(args)
        angles = parse_joint_angle_pairs(rest[1:])
        if angles is None:
            return
        self._commit(side, "angles", angles)

    def _h_wrist(self, address, *args):
        side, rest = self._split(args)
        quat = rest[1:]
        if len(quat) < 4:
            return
        try:
            raw = np.array([float(v) for v in quat[:4]])
            raw[1]*=-1
            raw[3]*=-1
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
