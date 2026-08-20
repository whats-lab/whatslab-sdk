from typing import ClassVar, Dict

from ._base import HandConfig


class RobotisHX5Config(HandConfig):

    _MODEL_SUBDIR = "robotis_hx5_d20"
    _URDF_FILENAME = "urdf/hx5_d20_{hand_type}.urdf"

    _CHAIN_LEN: ClassVar[Dict[str, int]] = {
        "thumb": 6, "index": 6, "middle": 6, "ring": 6, "pinky": 6,
    }
