from __future__ import annotations

import threading
from typing import Dict

import numpy as np
from scipy.spatial.transform import Rotation

from ..base import norm_quat
from .tls import preferred_ip
from .ws_server import WebXRServer

WEBXR_PORT = 8443

_CANONICAL_M = np.array([[0.0, 0.0, -1.0],
                         [-1.0, 0.0, 0.0],
                         [0.0, 1.0, 0.0]])


class WebXRReceiverBase:

    _M = _CANONICAL_M

    def __init__(self, port: int = WEBXR_PORT, listen_ip: str = "0.0.0.0",
                 tls: bool = True, certfile: str = None, keyfile: str = None):
        self._srv = WebXRServer.get(port, listen_ip, tls, certfile, keyfile)
        self._lock = threading.Lock()
        self._state: Dict[str, dict] = {"left": {}, "right": {}}
        self._hmd_pos = np.zeros(3)
        self._hmd_quat = np.array([0.0, 0.0, 0.0, 1.0])
        self._hmd_valid = False
        self._srv.set_handler(self._on_message)

    @property
    def port(self) -> int:
        return self._srv.port

    @property
    def url(self) -> str:
        host = preferred_ip() if self._srv.scheme == "https" else "localhost"
        return f"{self._srv.scheme}://{host}:{self._srv.port}/"

    def start(self) -> None:
        self._srv.start()

    def stop(self) -> None:
        self._srv.stop()

    def get_hmd(self):
        with self._lock:
            pos, quat, valid = self._hmd_pos.copy(), self._hmd_quat.copy(), self._hmd_valid
        if not valid:
            return pos, quat, valid
        pos, quat = self.to_canonical(pos, quat)
        return pos, quat, valid

    def to_canonical(self, pos, quat):
        pos2 = self._M @ np.asarray(pos, dtype=float)
        R = Rotation.from_quat(np.asarray(quat, dtype=float)).as_matrix()
        return pos2, Rotation.from_matrix(self._M @ R @ self._M.T).as_quat()

    def to_canonical_quat(self, quat):
        R = Rotation.from_quat(np.asarray(quat, dtype=float)).as_matrix()
        return Rotation.from_matrix(self._M @ R @ self._M.T).as_quat()

    def _on_message(self, msg: dict) -> None:
        hmd = msg.get("hmd")
        if isinstance(hmd, dict):
            self._set_hmd(hmd)
        for side in ("left", "right"):
            item = msg.get(side)
            if isinstance(item, dict):
                self._set_side(side, item)

    def _set_hmd(self, item: dict) -> None:
        pos = item.get("pos")
        quat = item.get("quat")
        if pos is None or quat is None:
            return
        with self._lock:
            self._hmd_pos = np.asarray(pos[:3], dtype=float)
            self._hmd_quat = norm_quat(quat[:4])
            self._hmd_valid = True

    def _set_side(self, side: str, item: dict) -> None:
        raise NotImplementedError
