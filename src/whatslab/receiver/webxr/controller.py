from __future__ import annotations

import time

import numpy as np
from scipy.spatial.transform import Rotation

from whatslab.core.types import InputSample, Pose
from ..base import norm_quat
from .base import WEBXR_PORT, WebXRReceiverBase

POS_FRAMES = ("hmd_yaw", "hmd", "room")


def _yaw_rz(hmd_quat: np.ndarray) -> np.ndarray:
    H = Rotation.from_quat(np.asarray(hmd_quat, dtype=float)).as_matrix()
    h = float(np.arctan2(H[1, 0], H[0, 0]))
    c, s = np.cos(-h), np.sin(-h)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


class WebXRControllerReceiver(WebXRReceiverBase):

    def __init__(self, port: int = WEBXR_PORT, listen_ip: str = "0.0.0.0",
                 stale_timeout: float = 0.0,
                 pos_offset=(0.0, 0.0, 0.0), pos_frame: str = "hmd_yaw",
                 tls: bool = True, certfile: str = None, keyfile: str = None):
        if pos_frame not in POS_FRAMES:
            raise ValueError(
                f"pos_frame 은 {list(POS_FRAMES)} 중 하나 — 받은 값 {pos_frame!r}")
        super().__init__(port, listen_ip, tls, certfile, keyfile)
        self._stale_timeout = stale_timeout
        self._pos_offset = np.asarray(pos_offset, dtype=float)
        self._pos_frame = pos_frame
        for side in ("left", "right"):
            s = self._state[side]
            s["pos"] = np.zeros(3)
            s["quat"] = np.array([0.0, 0.0, 0.0, 1.0])
            s["valid"] = False
            s["timestamp"] = 0.0

    @property
    def pos_frame(self) -> str:
        return self._pos_frame

    def _set_side(self, side: str, item: dict) -> None:
        pos = item.get("pos")
        quat = item.get("quat")
        if pos is None or quat is None:
            return
        with self._lock:
            s = self._state[side]
            s["pos"] = np.asarray(pos[:3], dtype=float)
            s["quat"] = norm_quat(quat[:4])
            s["valid"] = True
            s["timestamp"] = time.monotonic()

    def _relative(self, pos: np.ndarray, hmd_pos: np.ndarray,
                  hmd_quat: np.ndarray, hmd_valid: bool) -> np.ndarray:
        if self._pos_frame == "room" or not hmd_valid:
            return pos
        rel = pos - hmd_pos
        if self._pos_frame == "hmd":
            return rel
        return _yaw_rz(hmd_quat) @ rel

    def get(self, side: str) -> InputSample:
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
            pos, quat = self.to_canonical(pos, quat)
            pos = self._relative(pos, hmd_pos, hmd_quat, hmd_valid)
        controller = Pose(pos + self._pos_offset, quat) if valid else None
        hmd = Pose(hmd_pos, hmd_quat) if hmd_valid else None
        return InputSample(controller=controller, hmd=hmd, tracked=tracked, timestamp=ts)

    def connected(self, side: str) -> bool:
        return self.get(side).tracked
