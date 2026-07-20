from typing import ClassVar, Dict, List

from ._base import FingerChain, HandConfig


class BaseHandConfig(HandConfig):
    _MODEL_SUBDIR  = 'base_hand'
    _RVIZ_FILENAME = {'left': 'base_left.rviz', 'right': 'base_right.rviz'}
    _WRIST_LINK    = {'left': 'left_wrist', 'right': 'right_wrist'}

    _chains = [
        FingerChain(  # Thumb
            links=["{side}_wrist", "{side}_thumb_cmc0", "{side}_thumb_cmc1",
                   "{side}_thumb_mcp", "{side}_thumb_ip", "{side}_thumb_tip"],
            human=["wrist", "thumb_cmc0", "thumb_cmc1", "thumb_mcp", "thumb_ip", "thumb_tip"],
        ),
        FingerChain(  # Index
            links=["{side}_wrist", "{side}_index_mcp", "{side}_index_pip",
                   "{side}_index_dip", "{side}_index_tip"],
            human=["wrist", "index_mcp", "index_pip", "index_dip", "index_tip"],
        ),
        FingerChain(  # Middle
            links=["{side}_wrist", "{side}_middle_mcp", "{side}_middle_pip",
                   "{side}_middle_dip", "{side}_middle_tip"],
            human=["wrist", "middle_mcp", "middle_pip", "middle_dip", "middle_tip"],
        ),
        FingerChain(  # Ring
            links=["{side}_wrist", "{side}_ring_mcp", "{side}_ring_pip",
                   "{side}_ring_dip", "{side}_ring_tip"],
            human=["wrist", "ring_mcp", "ring_pip", "ring_dip", "ring_tip"],
        ),
        FingerChain(  # Pinky
            links=["{side}_wrist", "{side}_pinky_cmc", "{side}_pinky_mcp",
                   "{side}_pinky_pip", "{side}_pinky_dip", "{side}_pinky_tip"],
            human=["wrist", "pinky0", "pinky_mcp", "pinky_pip", "pinky_dip", "pinky_tip"],
        ),
    ]
    _FINGERS: ClassVar[Dict[str, List[FingerChain]]] = {'left': _chains, 'right': _chains}
