from __future__ import annotations

import threading
from typing import Dict

import numpy as np
from scipy.spatial.transform import Rotation

from ..base import norm_quat
from ..osc_transport import SharedOscServer

QUEST_OSC_PORT = 9000    # Unity → PC OSC(데이터) 포트. meta.QUEST_OSC_PORT 와 동일 값.

_CANONICAL_M = np.array([[0.0, 0.0, 1.0],
                         [-1.0, 0.0, 0.0],
                         [0.0, 1.0, 0.0]])


class QuestReceiverBase:

    _M = _CANONICAL_M     # Unity → 정준 변환(고정 상수, 유저 입력 없음)

    def __init__(self, quest_port: int = QUEST_OSC_PORT, listen_ip: str = "0.0.0.0"):
        self._srv = SharedOscServer.get(quest_port, listen_ip)
        self._lock = threading.Lock()
        # 서브클래스가 자유롭게 필드를 채우는 side 별 상태 컨테이너.
        self._state: Dict[str, dict] = {"left": {}, "right": {}}
        self._hmd_quat = np.array([0.0, 0.0, 0.0, 1.0])
        self._hmd_valid = False
        self._srv.add_handler("/hmd/rot", self._on_hmd_rot)

    # ---------------------------------------------------------------- public
    def start(self) -> None:
        self._srv.start()

    def stop(self) -> None:
        self._srv.stop()

    def get_hmd(self):
        with self._lock:
            q, valid = self._hmd_quat.copy(), self._hmd_valid
        return (self.to_canonical_quat(q) if valid else q), valid

    # -------------------------------------------------------- 정준 좌표 변환
    def to_canonical(self, pos, quat):
        pos2 = self._M @ np.asarray(pos, dtype=float)
        R = Rotation.from_quat(np.asarray(quat, dtype=float)).as_matrix()
        return pos2, Rotation.from_matrix(self._M @ R @ self._M.T).as_quat()

    def to_canonical_quat(self, quat):
        R = Rotation.from_quat(np.asarray(quat, dtype=float)).as_matrix()
        return Rotation.from_matrix(self._M @ R @ self._M.T).as_quat()

    # ----------------------------------------------------------- OSC handlers
    def _on_hmd_rot(self, address, *args):
        # /hmd/rot 는 side 인자 없음 — 첫 4개 float(x,y,z,w), 좌/우 공통.
        with self._lock:
            self._hmd_quat = norm_quat(args[:4])
            self._hmd_valid = True

    @staticmethod
    def _split(args):
        side = args[0]
        if isinstance(side, (list, tuple)):
            side = side[0]
        return side, args[1:]
