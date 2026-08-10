from typing import ClassVar, Dict, List

from ._base import HandConfig


class BaseHandConfig(HandConfig):

    _MODEL_SUBDIR = "base_hand"

    _HUMAN_CHAIN: ClassVar[Dict[str, List[str]]] = {
        "thumb": ["wrist", "thumb_cmc0", "thumb_cmc1", "thumb_mcp", "thumb_mcp",
                  "thumb_ip", "thumb_tip"],
        "index": ["wrist", "index_mcp", "index_mcp", "index_pip", "index_dip",
                  "index_tip"],
        "middle": ["wrist", "middle_mcp", "middle_mcp", "middle_pip", "middle_dip",
                   "middle_tip"],
        "ring": ["wrist", "ring_mcp", "ring_mcp", "ring_pip", "ring_dip", "ring_tip"],
        "pinky": ["wrist", "pinky_mcp", "pinky_mcp", "pinky_pip", "pinky_dip",
                  "pinky_tip"],
    }
