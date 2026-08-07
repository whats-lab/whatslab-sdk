from __future__ import annotations

from whatslab.receiver.glove.human_hand import GloveHumanHandReceiver

from whatslab.solvers.hand import HandRetargetController

from ..base import TeleopModel
from ..side import SideModel


class HandModel(TeleopModel):

    def __init__(self, hand_config: str = "orca_hand", side: str = "right",
                 urdf_root: str | None = None, hand_source=None):
        self._side = side
        self.hand_source = hand_source if hand_source is not None else GloveHumanHandReceiver()
        super().__init__(robot=None)
        self.sides[side] = SideModel(
            side=side, robot=None,
            retarget=HandRetargetController(side, hand_config, urdf_root=urdf_root))

    def _get_raw_target(self):
        return {s: None for s in self.SIDES}
