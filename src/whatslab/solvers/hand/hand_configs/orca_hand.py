from typing import ClassVar, Dict, List

from ._base import HandConfig


class OrcaHandConfig(HandConfig):

    _MODEL_SUBDIR = "orca_hand"
    _SCALE_FACTOR = [0.95, 1.03, 1.06, 1.06, 1.05]
    _KP_SHAPE_WEIGHT = 2.0
    _KP_COLD_SHAPE = True
    _FIXED_JOINTS = {
        "right": "R-Carpals_8d1f1041_to_TopTower-Model_4a80d30e",
        "left": "L-Carpals_719fff8c_to_TopTower-Model_4a80d30e",
    }

    _HUMAN_CHAIN: ClassVar[Dict[str, List[str]]] = {
        "thumb": ["wrist", "thumb_cmc1", "thumb_cmc1", "thumb_mcp", "thumb_ip", "thumb_tip"],
        "index": ["wrist", "index_mcp", "index_mcp", "index_pip", "index_tip"],
        "middle": ["wrist", "middle_mcp", "middle_mcp", "middle_pip", "middle_tip"],
        "ring": ["wrist", "ring_mcp", "ring_mcp", "ring_pip", "ring_tip"],
        "pinky": ["wrist", "pinky_mcp", "pinky_mcp", "pinky_pip", "pinky_tip"],
    }
