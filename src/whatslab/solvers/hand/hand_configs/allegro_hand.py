from typing import ClassVar, Dict

from ._base import HandConfig


class AllegroHandConfig(HandConfig):

    _MODEL_SUBDIR = "allegro_hand"

    _CHAIN_LEN: ClassVar[Dict[str, int]] = {
        "thumb": 6, "index": 6, "middle": 6, "ring": 6,
    }
