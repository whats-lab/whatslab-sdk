from __future__ import annotations

from typing import Optional

from whatslab.receiver.glove.openxr import GloveOpenXRReceiver

from whatslab.solvers.hand import HandRetargetController

from ..base import TeleopModel
from ..side import SideModel


class HandModel(TeleopModel):

    def __init__(self, hand_config: str = "dex:orca", side: str = "right",
                 urdf_root: str | None = None, hand_source=None,
                 hand_kwargs: Optional[dict] = None, **kwargs):
        if hand_source is not None and hand_kwargs:
            raise ValueError("hand_source 인스턴스와 hand_kwargs 는 함께 줄 수 없다")
        self._side = side
        self.hand_source = (hand_source if hand_source is not None
                            else GloveOpenXRReceiver(**(hand_kwargs or {})))
        super().__init__(robot=None)
        self.sides[side] = SideModel(
            side=side, robot=None,
            retarget=HandRetargetController(side, hand_config, urdf_root=urdf_root,
                                            **kwargs))

    def _get_raw_target(self):
        return {s: None for s in self.SIDES}
