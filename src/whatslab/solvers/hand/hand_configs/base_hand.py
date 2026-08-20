from typing import ClassVar, Dict

from ._base import HandConfig


class BaseHandConfig(HandConfig):

    _MODEL_SUBDIR = "base_hand"

    _CHAIN_LEN: ClassVar[Dict[str, int]] = {
        "thumb": 7, "index": 6, "middle": 6, "ring": 6, "pinky": 6,
    }
