"""GloveHumanHandReceiver — AGA 글러브 OSC 입력 소스 (GloveReceiverBase modality).

글러브는 손가락 회전(17×4, [0]=손목 root)만 제공한다. 팔 pose(controller)는
주지 않으므로 InputSample.controller/hand_root 는 None 이다.
구 `AirGloveReceiver`(atlas_hand_core/sources/atlas_glove.py 이식본)를
GloveReceiverBase 위 modality receiver 로 재배치한 것 — 파싱/출력은 동일하다.

RECEIVER 는 입력 전용 계약이다: 리타게팅/IK/캘리브레이션은 여기 두지 않는다
(Model 계층 소관, 별도 작업).
"""
from __future__ import annotations

import time
from typing import Optional

import numpy as np
from scipy.spatial.transform import Rotation

from whatslab.core.types import HandPose, InputSample
from .glove_base import GLOVE_OSC_PORT, GLOVE_TARGET_IP, GLOVE_CLIENT_PORT, GloveReceiverBase


# ── AGA OSC 프로토콜 상수 (atlas_hand_core/config.py 계승) ──
OSC_ADDR_LEFT_HAPT      = "/left/hapt/set"
OSC_ADDR_RIGHT_HAPT     = "/right/hapt/set"
OSC_MSG_TYPE_LEFT_HAPT  = "9"
OSC_MSG_TYPE_RIGHT_HAPT = "10"
AGA_FINGER_COUNT    = 5
AGA_RAW_FLOAT_COUNT = 72
AGA_SKIP_JOINT      = 14   # pinky_0 — FK 에서 제외

# 글러브(y-up) 손목 프레임 → 정준(x=앞, z=위, 오른손) 변환. y↔z 스왑.
# Quest 의 QuestReceiverBase._M 과 같은 역할이나 글러브 기기 프레임에 맞는 별도 상수다.
_CANONICAL_M = np.array([[1.0, 0.0, 0.0],
                         [0.0, 0.0, 1.0],
                         [0.0, 1.0, 0.0]])


def parse_aga_raw(raw_floats) -> np.ndarray:
    """AGA 원시 데이터(72 floats / 18×4) → 17×4 쿼터니언 ([0]=손목). pinky_0 제거."""
    arr = np.asarray(raw_floats, dtype=np.float32).reshape(18, 4)
    return np.delete(arr, AGA_SKIP_JOINT, axis=0)


def wrist_to_canonical(quat) -> np.ndarray:
    """글러브 손목 회전(글러브 프레임) → 정준 프레임: M·R·Mᵀ.

    손목 회전 q[0] 만 팔 EE 방향(GloveModel._arm_pose_raw)으로 쓰이므로 컨트롤러 pos
    (이미 정준)와 같은 프레임이어야 한다. 손가락 관절 회전([1:])은 리타게팅용 상대값
    이라 변환하지 않는다(Quest 손 receiver 와 동일한 원칙)."""
    R = Rotation.from_quat(np.asarray(quat, dtype=float)).as_matrix()
    return Rotation.from_matrix(_CANONICAL_M @ R @ _CANONICAL_M.T).as_quat()


def _neutral() -> np.ndarray:
    q = np.zeros((17, 4), dtype=np.float32)
    q[:, 3] = 1.0
    return q


class GloveHumanHandReceiver(GloveReceiverBase):
    """AGA 글러브 손가락 회전 수신 + 최신 샘플 pull (core.Receiver 프로토콜).

    side 는 물리적 글러브의 좌/우 — 재해석 금지 (크로스핸드 조합은 Model 의 arm_side/hand_side).

    - 손가락 17×4 회전을 손별 버퍼에 누적, get(side) 로 pull
    - send_haptic(side, values) 로 햅틱 명령 송신
    """

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
        # 새 프레임 수신 시 호출되는 콜백(side) — 이벤트 구동 소비자용(폴링 불필요)
        self._on_update = on_update

        for side in ("left", "right"):
            s = self._state[side]
            s["quats"] = _neutral()
            s["timestamp"] = 0.0
            self._srv.add_handler(f"/{side}/quat/get", self._h_quat, side)

    # ---------------------------------------------------------------- public
    def get(self, side: str) -> InputSample:
        with self._lock:
            s = self._state[side]
            q = s["quats"].copy()
            ts = s["timestamp"]
            conn = self._connected[side]
        age = time.monotonic() - ts
        tracked = conn and not (self._stale_timeout > 0 and age > self._stale_timeout)
        # 손목 회전(q[0])만 글러브 → 정준 변환(팔 EE 방향으로 쓰임). 손가락 관절은 raw.
        q[0] = wrist_to_canonical(q[0])
        # 글러브는 손가락 회전만: wrist.pos 없음(회전 q[0]만), controller 없음
        hand = HandPose.from_sensor_array(q, wrist_pos=None, tracked=tracked, timestamp=ts)
        return InputSample(controller=None, hand=hand, tracked=tracked, timestamp=ts)

    def send_haptic(self, side: str, values: list) -> bool:
        if self._udp_client is None or not self.connected(side):
            return False
        address = OSC_ADDR_LEFT_HAPT if side == "left" else OSC_ADDR_RIGHT_HAPT
        msg_type = OSC_MSG_TYPE_LEFT_HAPT if side == "left" else OSC_MSG_TYPE_RIGHT_HAPT
        packet: list = [msg_type]
        for i, v in enumerate(values[:AGA_FINGER_COUNT]):
            packet.extend([i, int(v)])
        try:
            self._udp_client.send_message(address, packet)
            return True
        except Exception:
            return False

    # ----------------------------------------------------------- OSC handlers
    def _h_quat(self, address, *args):
        # dispatcher.map(address, self._h_quat, side) 로 등록 — args[0]=side(주입값),
        # args[1:]=실제 OSC 메시지 인자([msg_type, *72 floats]).
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
        # OSC 메시지 첫 값 args[0] 은 메시지 타입 헤더(예: '1') → 건너뛰고 count 개.
        # (atlas AtlasGloveSource._parse_floats 와 동일: args[1:count+1])
        if len(args) < count + 1:
            return None
        try:
            return np.array(args[1:count + 1], dtype=np.float32)
        except (TypeError, ValueError):
            return None
