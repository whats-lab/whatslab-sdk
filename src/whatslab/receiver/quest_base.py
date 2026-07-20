"""QuestReceiverBase — SharedOscServer 위에 얹는 Quest modality receiver 공통 기반.

같은 Quest 기기가 보내는 여러 modality(컨트롤러/손가락 등)를 각각 별도 receiver 로
분리하되, 포트 하나(OSC 서버 소켓)는 공유한다. 이 베이스가 그 공유를 담당한다:

  - `SharedOscServer.get(quest_port)` 로 서버를 획득(직접 소켓을 열지 않음).
  - `/hmd/rot` (좌/우 공통 머리 자세)는 여기서 한 번만 등록해 모든 서브클래스가
    `get_hmd()` 로 재사용한다.
  - `start()/stop()` 은 SharedOscServer 의 refcount 에 위임 — 여러 receiver 가
    동시에 start 해도 마지막 stop() 에서만 실제로 서버가 닫힌다.

서브클래스(예: QuestControllerReceiver)는 자신의 modality OSC 주소를
`self._srv.add_handler(...)` 로 등록하고, `self._state[side]` 에 원하는 필드를
채운 뒤 `get(side) -> InputSample` 을 구현한다.

side 는 물리적 기기(컨트롤러/손)의 좌/우 — 재해석 금지. 마운트 조합은
Model 계층(TeleopModel 의 arm_side/hand_side)에서 선언한다.
"""
from __future__ import annotations

import threading
from typing import Dict

import numpy as np
from scipy.spatial.transform import Rotation

from .base import norm_quat
from .osc_transport import SharedOscServer

QUEST_OSC_PORT = 9000    # Unity → PC OSC(데이터) 포트. meta.QUEST_OSC_PORT 와 동일 값.

_CANONICAL_M = np.array([[0.0, 0.0, 1.0],
                         [-1.0, 0.0, 0.0],
                         [0.0, 1.0, 0.0]])


class QuestReceiverBase:
    """Quest 계열 modality receiver 공통 기반 (core.Receiver 프로토콜: start/stop/get).

    직접 인스턴스화하지 않는다 — modality 별 서브클래스(QuestControllerReceiver 등)를
    통해 사용한다. `get(side)` 는 서브클래스가 구현해야 한다.
    """

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
        """(정준 quat_xyzw, valid) 스냅샷 — 정준 변환 적용. 서브클래스 get() 에서 사용."""
        with self._lock:
            q, valid = self._hmd_quat.copy(), self._hmd_valid
        return (self.to_canonical_quat(q) if valid else q), valid

    # -------------------------------------------------------- 정준 좌표 변환
    def to_canonical(self, pos, quat):
        """Unity(HMD 로컬) pose → 정준(x=앞,z=위,오른손): 위치 M·p, 회전 M·R·Mᵀ."""
        pos2 = self._M @ np.asarray(pos, dtype=float)
        R = Rotation.from_quat(np.asarray(quat, dtype=float)).as_matrix()
        return pos2, Rotation.from_matrix(self._M @ R @ self._M.T).as_quat()

    def to_canonical_quat(self, quat):
        """회전만 정준 변환 (위치 없는 hmd 등)."""
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
        """`(side, x, y, z, ...)` 형태 OSC 인자에서 side 를 분리."""
        side = args[0]
        if isinstance(side, (list, tuple)):
            side = side[0]
        return side, args[1:]
