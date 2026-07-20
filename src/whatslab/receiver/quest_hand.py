"""QuestHandReceiver — /hand/{left,right}/{pos,rot,joints/pos,joints/rot} modality receiver.

손 트래킹 파싱(root=finger_quats[0], joints/rot → FINGER_JOINT_ORDER 16관절,
joints/pos 미사용)만 담당하고, 컨트롤러는 다루지 않는다 — QuestReceiverBase 를
통해 다른 modality receiver(컨트롤러 등)와 OSC 포트(SharedOscServer)를 공유한다.

프로토콜(Unity QuestOscSender 합의, 좌표는 HMD 로컬):
  /hand/<left|right>/pos          float x,y,z       (손목 root)
  /hand/<left|right>/rot          float x,y,z,w
  /hand/<left|right>/joints/pos   float[16*3]       (미사용 — 회전만 리타게팅에 쓴다)
  /hand/<left|right>/joints/rot   float[16*4]       (FINGER_JOINT_ORDER)
"""
from __future__ import annotations

import time
from typing import Callable, Optional

import numpy as np

from whatslab.core.types import HandPose, InputSample, Pose
from .base import NUM_FINGER_JOINTS, neutral_finger_quats, norm_quat
from .quest_base import QUEST_OSC_PORT, QuestReceiverBase


class QuestHandReceiver(QuestReceiverBase):
    """손목 pos/rot + 손가락 joints 만 수신 (컨트롤러 없음). side 별 상태 독립.

    `on_update(side)` 콜백은 새 joints/rot 프레임(=한 손 프레임 완성) 수신 시
    호출된다 (이벤트 구동 소비자용).
    """

    def __init__(self, quest_port: int = QUEST_OSC_PORT, listen_ip: str = "0.0.0.0",
                 stale_timeout: float = 0.0,
                 on_update: Optional[Callable[[str], None]] = None):
        super().__init__(quest_port, listen_ip)
        self._stale_timeout = stale_timeout
        self._on_update = on_update
        for side in ("left", "right"):
            s = self._state[side]
            s["wrist_pos"] = np.zeros(3)
            s["finger_quats"] = neutral_finger_quats()  # [0]=wrist 회전, [1:17]=관절
            s["tracked"] = False
            s["timestamp"] = 0.0
            self._srv.add_handler(f"/hand/{side}/pos", self._on_hand_pos, side)
            self._srv.add_handler(f"/hand/{side}/rot", self._on_hand_rot, side)
            self._srv.add_handler(f"/hand/{side}/joints/pos", self._on_joints_pos, side)
            self._srv.add_handler(f"/hand/{side}/joints/rot", self._on_joints_rot, side)

    # ----------------------------------------------------------- OSC handlers
    def _on_hand_pos(self, address, *args):
        side, v = self._split(args)
        with self._lock:
            self._state[side]["wrist_pos"] = np.array(v[:3], dtype=float)

    def _on_hand_rot(self, address, *args):
        side, v = self._split(args)
        with self._lock:
            self._state[side]["finger_quats"][0] = norm_quat(v[:4])  # root = finger_quats[0]

    def _on_joints_pos(self, address, *args):
        # 위치는 현재 리타게팅에 미사용 (회전만 사용). meta.py 와 동일.
        pass

    def _on_joints_rot(self, address, *args):
        side, v = self._split(args)
        arr = np.asarray(v, dtype=float)
        if arr.size < NUM_FINGER_JOINTS * 4:
            return
        rots = arr[: NUM_FINGER_JOINTS * 4].reshape(NUM_FINGER_JOINTS, 4)
        rots = rots / (np.linalg.norm(rots, axis=1, keepdims=True) + 1e-9)
        with self._lock:
            s = self._state[side]
            s["finger_quats"][1:1 + NUM_FINGER_JOINTS] = rots
            s["tracked"] = True
            s["timestamp"] = time.monotonic()
        if self._on_update is not None:
            self._on_update(side)

    # -------------------------------------------------------------------- get
    def get(self, side: str) -> InputSample:
        with self._lock:
            s = self._state[side]
            wrist_pos = s["wrist_pos"].copy()
            finger = s["finger_quats"].copy()
            tracked = s["tracked"]
            ts = s["timestamp"]
        hmd_quat, hmd_valid = self.get_hmd()
        age = time.monotonic() - ts
        tracked = tracked and not (self._stale_timeout > 0 and age > self._stale_timeout)
        # 손목 pose(위치+root 회전)만 Unity → 정준 변환. 손가락 관절 회전(finger[1:])은
        # 리타게팅용 상대값이라 변환하지 않는다.
        wrist_pos, finger[0] = self.to_canonical(wrist_pos, finger[0])
        hand = HandPose.from_sensor_array(finger, wrist_pos=wrist_pos,
                                          tracked=tracked, timestamp=ts)
        hmd = Pose(np.zeros(3), hmd_quat) if hmd_valid else None
        return InputSample(hand=hand, hmd=hmd, tracked=tracked, timestamp=ts)

    def connected(self, side: str) -> bool:
        return self.get(side).tracked
