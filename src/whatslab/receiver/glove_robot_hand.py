"""GloveRobotHandReceiver — 글러브(펌웨어)가 리타게팅을 거치지 않고 로봇 관절각(q)을
직접 보내는 모드의 receiver.

TODO(실제 프로토콜 미확인): 글러브 펌웨어가 "이미 리타게팅된 로봇 q"를 직접 보내는
경우의 실제 OSC 주소/페이로드/단위는 제조사 문서 확인 전이라 아래는 합리적 가정 하의
파서 골격이다:
  · 주소   : `/glove/{side}/q` (GloveReceiverBase 와 동일 GLOVE_OSC_PORT 를 공유)
  · 페이로드: float 배열(라디안), 생성자에 주입한 `joint_names` 순서와 1:1 대응
  · 관절명↔인덱스 매핑은 여기서 하드코드하지 않고 소비자(Model/설정)가 주입한다.
실기 프로토콜이 확정되면 `_h_q` 파서 본문만 교체하면 된다 — 캐리어 계약
(`InputSample.joint_q`)은 core/types.py 에 이미 고정되어 있어 변하지 않는다.

RECEIVER 는 입력 전용 계약이다: 여기서 IK/리타게팅을 하지 않는다 — 이 모드 자체가
"이미 계산된 q" 를 그대로 전달하는 바이패스 경로이며, 실제 우회는
`TeleopModel.get_q()` 가 `InputSample.joint_q is not None` 을 감지해 수행한다.
"""
from __future__ import annotations

import time
from typing import List, Optional

import numpy as np

from whatslab.core.types import InputSample
from .glove_base import GLOVE_CLIENT_PORT, GLOVE_OSC_PORT, GLOVE_TARGET_IP, GloveReceiverBase


class GloveRobotHandReceiver(GloveReceiverBase):
    """글러브 → 로봇 q 직접 송신 모드 (core.Receiver 프로토콜: start/stop/get).

    side 는 물리적 글러브의 좌/우 — 재해석 금지(크로스핸드 조합은 Model 의 arm_side/hand_side).
    """

    def __init__(
        self,
        joint_names: List[str],
        glove_port: int = GLOVE_OSC_PORT,
        listen_ip: str = "0.0.0.0",
        target_ip: str = GLOVE_TARGET_IP,
        client_port: int = GLOVE_CLIENT_PORT,
        stale_timeout: float = 0.0,
        on_update: Optional[callable] = None,
    ):
        super().__init__(glove_port, listen_ip, target_ip, client_port)
        if not joint_names:
            raise ValueError("joint_names 는 비어있을 수 없다(로봇 q 순서 매핑 필수)")
        self.joint_names = list(joint_names)
        self._stale_timeout = stale_timeout
        # 새 프레임 수신 시 호출되는 콜백(side) — 이벤트 구동 소비자용(폴링 불필요)
        self._on_update = on_update

        for side in ("left", "right"):
            s = self._state[side]
            s["q"] = np.zeros(len(self.joint_names))
            s["timestamp"] = 0.0
            self._srv.add_handler(f"/glove/{side}/q", self._h_q, side)

    # ---------------------------------------------------------------- public
    def get(self, side: str) -> InputSample:
        with self._lock:
            s = self._state[side]
            q = s["q"].copy()
            ts = s["timestamp"]
            conn = self._connected[side]
        age = time.monotonic() - ts
        tracked = conn and not (self._stale_timeout > 0 and age > self._stale_timeout)
        joint_q = dict(zip(self.joint_names, (float(v) for v in q)))
        return InputSample(joint_q=joint_q, tracked=tracked, timestamp=ts)

    # ----------------------------------------------------------- OSC handlers
    def _h_q(self, address, *args):
        # dispatcher.map(address, self._h_q, side) 로 등록 — args[0]=side(주입값),
        # args[1:]=실제 OSC 메시지 인자(float q 배열, joint_names 순서).
        side = args[0]
        if isinstance(side, (list, tuple)):
            side = side[0]
        n = len(self.joint_names)
        raw = args[1:]
        if len(raw) < n:
            return
        try:
            q = np.array(raw[:n], dtype=float)
        except (TypeError, ValueError):
            return
        with self._lock:
            s = self._state[side]
            s["q"] = q
            s["timestamp"] = time.monotonic()
            self._connected[side] = True
        if self._on_update is not None:
            self._on_update(side)
