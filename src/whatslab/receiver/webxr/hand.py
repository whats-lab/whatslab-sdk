from __future__ import annotations

import threading
import time
from typing import Optional

import numpy as np
from scipy.spatial.transform import Rotation

from whatslab.core.constants import SIDES, XR_JOINT_COUNT, XR_JOINT_FIELDS
from whatslab.core.types import HandPose, InputSample

from .tls import preferred_ip
from .ws_server import WebXRServer

WEBXR_PORT = 8443
WEBXR_HAND_JOINT_COUNT = 25
WEBXR_WRIST = 0
WEBXR_MIDDLE_METACARPAL = 10
WEBXR_MIDDLE_PROXIMAL = 11


def webxr_hand_to_openxr26(joints25: np.ndarray) -> Optional[np.ndarray]:
    a = np.asarray(joints25, dtype=float).reshape(WEBXR_HAND_JOINT_COUNT, XR_JOINT_FIELDS)
    if not np.all(np.isfinite(a)):
        return None
    quats = a[:, :4]
    norms = np.linalg.norm(quats, axis=1, keepdims=True)
    if np.any(norms < 1e-6):
        return None
    rot = Rotation.from_quat(quats / norms)
    wrist_rot = rot[WEBXR_WRIST].inv()
    rel_pos = wrist_rot.apply(a[:, 4:7] - a[WEBXR_WRIST, 4:7])
    rel_quat = (wrist_rot * rot).as_quat()

    out = np.zeros((XR_JOINT_COUNT, XR_JOINT_FIELDS))
    out[1:, 0:4] = rel_quat
    out[1:, 4:7] = rel_pos
    out[1:, 7] = a[:, 7]
    out[0, 0:4] = rel_quat[WEBXR_MIDDLE_METACARPAL]
    out[0, 4:7] = 0.5 * (rel_pos[WEBXR_MIDDLE_METACARPAL] + rel_pos[WEBXR_MIDDLE_PROXIMAL])
    out[0, 7] = a[WEBXR_MIDDLE_METACARPAL, 7]
    out[1, 0:4] = (0.0, 0.0, 0.0, 1.0)
    out[1, 4:7] = 0.0
    return out


class WebXRHandReceiver:

    def __init__(
        self,
        port: int = WEBXR_PORT,
        listen_ip: str = "0.0.0.0",
        stale_timeout: float = 0.2,
        tls: bool = True,
        certfile: str = None,
        keyfile: str = None,
    ):
        self._srv = WebXRServer(self._on_message, port, listen_ip, tls, certfile, keyfile)
        self._stale_timeout = float(stale_timeout)
        self._lock = threading.Lock()
        self._joints = {s: None for s in SIDES}
        self._timestamp = {s: 0.0 for s in SIDES}

    @property
    def port(self) -> int:
        return self._srv.port

    @property
    def url(self) -> str:
        host = preferred_ip() if self._srv.scheme == "https" else "localhost"
        return f"{self._srv.scheme}://{host}:{self._srv.port}/"

    @property
    def running(self) -> bool:
        return self._srv.is_running

    def start(self) -> None:
        self._srv.start()

    def stop(self) -> None:
        self._srv.stop()

    def _on_message(self, msg: dict) -> None:
        hands = msg.get("hands")
        if not isinstance(hands, dict):
            return
        for side in SIDES:
            raw = hands.get(side)
            if not isinstance(raw, list) or len(raw) != WEBXR_HAND_JOINT_COUNT * XR_JOINT_FIELDS:
                continue
            joints = webxr_hand_to_openxr26(np.asarray(raw, dtype=float))
            if joints is None:
                continue
            with self._lock:
                self._joints[side] = joints
                self._timestamp[side] = time.monotonic()

    def get(self, side: str) -> InputSample:
        with self._lock:
            joints = self._joints[side]
            ts = self._timestamp[side]
        if joints is None:
            return InputSample(tracked=False, timestamp=ts)
        age = time.monotonic() - ts
        tracked = not (self._stale_timeout > 0 and age > self._stale_timeout)
        hand = HandPose(
            xr_joints=joints.copy(),
            xr_valid=np.ones(XR_JOINT_COUNT, dtype=bool),
            tracked=tracked,
            timestamp=ts,
        )
        return InputSample(hand=hand, tracked=tracked, timestamp=ts)

    def connected(self, side: str) -> bool:
        return self.get(side).tracked
