from typing import ClassVar, Dict, List

from ._base import HandConfig


class AbilityHandConfig(HandConfig):

    _MODEL_SUBDIR = "ability_hand"
    _TARGET_JOINT_NAMES = ["thumb_q1", "thumb_q2", "index_q1", "middle_q1",
                           "ring_q1", "pinky_q1"]

    _HUMAN_CHAIN: ClassVar[Dict[str, List[str]]] = {
        "thumb": ["wrist", "thumb_cmc0", "thumb_cmc1", "thumb_mcp", "thumb_ip"],
        "index": ["wrist", "index_mcp", "index_pip", "index_dip"],
        "middle": ["wrist", "middle_mcp", "middle_pip", "middle_dip"],
        "ring": ["wrist", "ring_mcp", "ring_pip", "ring_dip"],
        "pinky": ["wrist", "pinky_mcp", "pinky_pip", "pinky_dip"],
    }
