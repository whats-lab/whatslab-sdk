from typing import ClassVar, Dict

from ._base import HandConfig


class TesolloDG5FConfig(HandConfig):

    _MODEL_SUBDIR = "tesollo_dg5f"

    _CHAIN_LEN: ClassVar[Dict[str, int]] = {
        "thumb": 6, "index": 5, "middle": 5, "ring": 5, "pinky": 5,
    }
