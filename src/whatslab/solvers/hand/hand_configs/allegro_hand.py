from typing import ClassVar, Dict, List

import numpy as np

from ._base import HandConfig


class AllegroHandConfig(HandConfig):

    _MODEL_SUBDIR = "allegro_hand"
    _COORD_TRANSFORM: ClassVar[np.ndarray] = np.array(
        [[0, 0, -1], [-1, 0, 0], [0, 1, 0]], dtype=np.float32
    )

    _HUMAN_CHAIN: ClassVar[Dict[str, List[str]]] = {
        "thumb": ["wrist", "thumb_cmc0", "thumb_cmc1", "thumb_mcp", "thumb_ip", "thumb_tip"],
        "index": ["wrist", "index_mcp", "index_pip", "index_dip", "index_tip", "index_tip"],
        "middle": ["wrist", "middle_mcp", "middle_pip", "middle_dip", "middle_tip", "middle_tip"],
        "ring": ["wrist", "ring_mcp", "ring_pip", "ring_dip", "ring_tip", "ring_tip"],
    }
