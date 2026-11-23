from __future__ import annotations

import time
from typing import Dict, Optional

from whatslab.core.interfaces import Receiver
from whatslab.retarget import HandRetargeter


class HandModel:

    def __init__(self, hand_config: str = "dex:orca", side: str = "right",
                 urdf_root: Optional[str] = None, hand_source: Optional[Receiver] = None,
                 hand_kwargs: Optional[dict] = None, **kwargs):
        if hand_source is not None and hand_kwargs:
            raise ValueError("hand_source 인스턴스와 hand_kwargs 는 함께 줄 수 없다")
        if hand_source is None:
            from whatslab.receiver.glove.airglove import AirGloveReceiver

            hand_source = AirGloveReceiver(**(hand_kwargs or {}))
        self.side = side
        self.hand_source = hand_source
        self.retarget = HandRetargeter(hand_config, side=side, urdf_root=urdf_root, **kwargs)
        self.safety = None
        self.q: Dict[str, float] = {}
        self._t_prev: Optional[float] = None

    def start(self) -> None:
        self.hand_source.start()

    def stop(self) -> None:
        self.hand_source.stop()

    def get_q(self) -> Dict[str, Dict[str, float]]:
        sample = self.hand_source.get(self.side)
        cmd = self.retarget.compute(sample)
        tracked = sample.hand is not None and sample.hand.tracked
        q = dict(zip(cmd.joint_names, map(float, cmd.joint_angles))) if tracked else {}
        if self.safety is not None:
            now = time.monotonic()
            dt = None if self._t_prev is None else now - self._t_prev
            self._t_prev = now
            q = self.safety.step(q, dt)
        self.q = q
        return {self.side: q}
