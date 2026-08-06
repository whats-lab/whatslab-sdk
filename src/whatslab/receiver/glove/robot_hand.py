from __future__ import annotations

import logging
import time
from typing import Dict, Optional

import numpy as np

from whatslab.core.types import HandPose, InputSample, Pose
from .base import GLOVE_CLIENT_PORT, GLOVE_OSC_PORT, GLOVE_TARGET_IP, GloveReceiverBase
from .human_hand import wrist_to_canonical

logger = logging.getLogger(__name__)


def unpack_wrist(args) -> np.ndarray:
    # Spine 은 손등 raw 쿼터니언 (w,x,y,z) 를 [y, x, z, -w] 로 재배열해 보낸다
    # (OSC_Protocol.md `/side/wrist/get`). HandCoordinateConvention 을 타지 않는
    # 유일한 발행값이라 여기서 raw 로 되돌린다.
    y, x, z, nw = (float(v) for v in args[:4])
    return np.array([-nw, x, y, z])


def spine_lh_xyzw(q_wxyz) -> np.ndarray:
    # Spine 의 wire 규약: HandCoordinateConvention(LeftHanded, Xyzw)
    # = (w,-x,y,-z) 후 xyzw 재배열. `/side/quat/get` 이 이미 적용해 보내는 변환이라,
    # raw 로 오는 손목을 quat 계열과 같은 프레임에 올리려면 여기서 직접 태운다.
    w, x, y, z = (float(v) for v in q_wxyz)
    return np.array([-x, y, -z, w])


# Spine 의 URDF 경로 발행(joint_angles + wrist)을 받는다.
#
# /side/joint_angles/get 은 (이름, rad) 쌍이라 순서를 하드코딩하지 않는다 — 배열
# 순서가 pinocchio 내부 순서이고 프로파일(orca/robotis/allegro)마다 개수·순서가
# 달라 위치 기반 소비는 프로파일 교체에 깨진다. joint_map 으로 Spine 이름
# (side 접두사 제거) → 로봇 관절명을 옮긴다. None 이면 Spine 이름 그대로 낸다.
#
# 손목은 joint_angles 에 없다(앞 7슬롯 free-flyer 루트를 빼고 보냄) — 별개 주소
# /side/wrist/get 에서 받아 정준으로 올려 hand.wrist 에 싣는다(팔 EE 목표).
# 두 주소는 별개 데이터그램이라 동일 프레임 보장이 없다.
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
        # 새 프레임 수신 시 호출되는 콜백(side) — 이벤트 구동 소비자용(폴링 불필요)
        self._on_update = on_update

        for side in ("left", "right"):
            s = self._state[side]
            s["q"] = None            # 첫 패킷 전에는 0 을 채우지 않는다(생략 규칙)
            s["wrist"] = None
            s["timestamp"] = 0.0
            self._srv.add_handler(f"/{side}/joint_angles/get", self._h_joint_angles, side)
            self._srv.add_handler(f"/{side}/wrist/get", self._h_wrist, side)

    # ---------------------------------------------------------------- public
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
            # tracked=False 고정 — 이 경로는 손가락 회전(joint_rot)을 주지 않으므로
            # 리타게팅 대상이 아니다. 손목만 팔 목표로 쓰인다.
            hand = HandPose(wrist=Pose(quat=wrist), tracked=False, timestamp=ts)
        return InputSample(hand=hand, joint_q=q, tracked=tracked, timestamp=ts)

    # ----------------------------------------------------------- OSC handlers
    def _h_joint_angles(self, address, *args):
        # add_handler(address, cb, side) 로 등록 — args[0]=side(주입값),
        # args[1]=messageType 헤더('16'/'17'), args[2:]=(이름, rad) 쌍 반복.
        side, rest = self._split(args)
        pairs = rest[1:]
        if len(pairs) < 2:
            return
        q: Dict[str, float] = {}
        prefix = f"{side}_"
        for i in range(0, len(pairs) - 1, 2):
            name, val = pairs[i], pairs[i + 1]
            if not isinstance(name, str):
                return                       # 쌍 정렬이 깨진 패킷 — 통째로 버린다
            try:
                v = float(val)
            except (TypeError, ValueError):
                return
            if name.startswith(prefix):
                name = name[len(prefix):]
            if self.joint_map is not None:
                name = self.joint_map.get(name)
                if name is None:
                    continue                 # 이 로봇이 쓰지 않는 관절 — 조용히 생략
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
            raw = unpack_wrist(quat)
        except (TypeError, ValueError):
            return
        if not np.all(np.isfinite(raw)) or np.linalg.norm(raw) < 1e-9:
            return
        self._commit(side, "wrist", wrist_to_canonical(spine_lh_xyzw(raw)))

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
