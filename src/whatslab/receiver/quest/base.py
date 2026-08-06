from __future__ import annotations

import threading
from typing import Dict

import numpy as np
from scipy.spatial.transform import Rotation

from ..base import norm_quat
from ..osc_transport import SharedOscServer

QUEST_OSC_PORT = 9000

_CANONICAL_M = np.array([[0.0, 0.0, 1.0],
                         [-1.0, 0.0, 0.0],
                         [0.0, 1.0, 0.0]])


class QuestReceiverBase:

    _M = _CANONICAL_M

    def __init__(self, quest_port: int = QUEST_OSC_PORT, listen_ip: str = "0.0.0.0"):
        self._srv = SharedOscServer.get(quest_port, listen_ip)
        self._lock = threading.Lock()
        self._state: Dict[str, dict] = {"left": {}, "right": {}}
        self._hmd_quat = np.array([0.0, 0.0, 0.0, 1.0])
        self._hmd_valid = False
        self._srv.add_handler("/hmd/rot", self._on_hmd_rot)

    def start(self) -> None:
        self._srv.start()

    def stop(self) -> None:
        self._srv.stop()

    def get_hmd(self):
        with self._lock:
            q, valid = self._hmd_quat.copy(), self._hmd_valid
        return (self.to_canonical_quat(q) if valid else q), valid

    def to_canonical(self, pos, quat):
        pos2 = self._M @ np.asarray(pos, dtype=float)
        R = Rotation.from_quat(np.asarray(quat, dtype=float)).as_matrix()
        return pos2, Rotation.from_matrix(self._M @ R @ self._M.T).as_quat()

    def to_canonical_quat(self, quat):
        R = Rotation.from_quat(np.asarray(quat, dtype=float)).as_matrix()
        return Rotation.from_matrix(self._M @ R @ self._M.T).as_quat()

    def _on_hmd_rot(self, address, *args):
        with self._lock:
            self._hmd_quat = norm_quat(args[:4])
            self._hmd_valid = True

    @staticmethod
    def _split(args):
        side = args[0]
        if isinstance(side, (list, tuple)):
            side = side[0]
        return side, args[1:]
