from typing import ClassVar, Dict

from ._base import HandConfig


class AbilityHandConfig(HandConfig):

    _MODEL_SUBDIR = "ability_hand"

    _CHAIN_LEN: ClassVar[Dict[str, int]] = {
        "thumb": 5, "index": 4, "middle": 4, "ring": 4, "pinky": 4,
    }
