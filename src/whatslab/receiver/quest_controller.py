"""QuestControllerReceiver — /controller/{left,right}/{pos,rot} 만 다루는 modality receiver.

컨트롤러 처리(quat 정규화, 마운트 오프셋 가산)만 담당하고, 손가락 트래킹은
다루지 않는다 — QuestReceiverBase 를 통해 다른 modality receiver(손가락 등)와
OSC 포트(SharedOscServer)를 공유한다.

프로토콜(Unity QuestOscSender 합의, 좌표는 HMD 로컬):
  /controller/<left|right>/pos    float x,y,z
  /controller/<left|right>/rot    float x,y,z,w
"""
from __future__ import annotations

import time
from typing import Optional

import numpy as np

from whatslab.core.types import InputSample, Pose
from .base import norm_quat
from .quest_base import QUEST_OSC_PORT, QuestReceiverBase

# 컨트롤러 위치 오프셋 (정준좌표 x=앞,z=위) — 컨트롤러 마운트 편차 보정. 불변 상수로
# 리시버가 controller.pos 에 가산한다(외부 설정 불가). meta.CONTROLLER_POS_OFFSET 와 동일 값.
CONTROLLER_POS_OFFSET = np.array([0.02, -0.04, 0.08])


class QuestControllerReceiver(QuestReceiverBase):
    """컨트롤러 pos/rot 만 수신 (손가락/손목 트래킹 없음). side 별 상태 독립."""

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

    # ----------------------------------------------------------- OSC handlers
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

    # -------------------------------------------------------------------- get
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
        # Unity → 정준 좌표 변환(QuestReceiverBase). 그 뒤 마운트 오프셋(정준) 가산.
        if valid:
            pos, quat = self.to_canonical(pos, quat)
        controller = Pose(pos + CONTROLLER_POS_OFFSET, quat) if valid else None
        hmd = Pose(np.zeros(3), hmd_quat) if hmd_valid else None
        return InputSample(controller=controller, hmd=hmd, tracked=tracked, timestamp=ts)

    def connected(self, side: str) -> bool:
        return self.get(side).tracked
