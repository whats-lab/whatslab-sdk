from __future__ import annotations

import time
from typing import Dict, Optional

from whatslab.solvers.hand import HandRetargetController


class HandModel:
    """한 손의 입력 수신기 + 손 리타게팅. ``get_q()`` 가 ``{side: {관절: rad}}`` 를 낸다.

    ``hand_source`` 를 안 주면 ``AirGloveReceiver(**hand_kwargs)``. ``safety`` 에 ``SafetyFilter``
    를 넣으면 매 틱 측정 dt 로 통과시킨다. 추적이 끊긴 틱에는 손 관절이 결과에서 빠진다.
    """

    def __init__(self, hand_config: str = "dex:orca", side: str = "right",
                 urdf_root: Optional[str] = None, hand_source=None,
                 hand_kwargs: Optional[dict] = None, **kwargs):
        if hand_source is not None and hand_kwargs:
            raise ValueError("hand_source 인스턴스와 hand_kwargs 는 함께 줄 수 없다")
        if hand_source is None:
            # airglove-client 는 receiver extra — 다른 수신기를 넘기는 쪽은 없어도 쓸 수 있게.
            from whatslab.receiver.glove.airglove import AirGloveReceiver

            hand_source = AirGloveReceiver(**(hand_kwargs or {}))
        self.side = side
        self.hand_source = hand_source
        self.retarget = HandRetargetController(side, hand_config, urdf_root=urdf_root, **kwargs)
        self.safety = None
        self.q: Dict[str, float] = {}
        self._tracked = False
        self._t_prev: Optional[float] = None

    def start(self) -> None:
        self.hand_source.start()

    def stop(self) -> None:
        self.hand_source.stop()

    def get_q(self) -> Dict[str, Dict[str, float]]:
        sample = self.hand_source.get(self.side)
        q: Dict[str, float] = {}
        if sample is not None and sample.hand is not None and sample.hand.tracked:
            cmd = self.retarget.compute(sample)
            q = {n: float(v) for n, v in zip(cmd.joint_names, cmd.joint_angles)}
            self._tracked = True
        elif self._tracked:
            # 끊긴 틱에는 compute 를 안 부르므로 엔진 상태(low-pass·warm start)를 여기서 비운다.
            self._tracked = False
            self.retarget.reset()
        if self.safety is not None:
            now = time.monotonic()
            dt = None if self._t_prev is None else now - self._t_prev
            self._t_prev = now
            q = self.safety.step(q, dt)
        self.q = q
        return {self.side: q}
