from typing import ClassVar, Dict, List

from ._base import FingerChain, HandConfig

import numpy as np
class TesolloDG5FConfig(HandConfig):
    """Tesollo DG5F — 5-finger, 20-DOF.

    링크명 접두어: ll_ (left) / rl_ (right).
    """

    _MODEL_SUBDIR  = 'tesollo_dg5f'
    _URDF_FILENAME = 'dg5f_{hand_type}.urdf'
    _SIDE_MAP      = {'left': 'll', 'right': 'rl'}
    _WRIST_LINK    = {'left': 'll_dg_palm', 'right': 'rl_dg_palm'}
    
    _COORD_TRANSFORM: ClassVar[np.ndarray] = np.array(
        [[0, 1, 0], [0, 0, 1], [1, 0,0]], dtype=np.float32
    )
    _SCALE_FACTOR = [1, 1.2, 1.2, 1.2, 1.2]    
      
      
    _FINGERS: ClassVar[Dict[str, List[FingerChain]]] = {
        "right":  [
            FingerChain(  # Thumb 
                links=[
                    "{wrist}",
                    "rl_dg_1_1",
                    "rl_dg_1_2",
                    "rl_dg_1_3",
                    "rl_dg_1_4",
                    "rl_dg_1_tip",
                ],
                human=["wrist", "thumb_cmc0", "thumb_cmc1", "thumb_mcp", "thumb_ip", "thumb_tip"],
            ),
            FingerChain(  # Index
                links=[
                   "{wrist}",
                    "rl_dg_2_2",
                    "rl_dg_2_3",
                    "rl_dg_2_4",
                    "rl_dg_2_tip",
                ],
                human=["wrist", "index_mcp", "index_pip", "index_dip", "index_tip"],
            ),
               FingerChain(  # Index
                links=[
                   "{wrist}",
                    "rl_dg_3_2",
                    "rl_dg_3_3",
                    "rl_dg_3_4",
                    "rl_dg_3_tip",
                ],
                human=["wrist", "middle_mcp", "middle_pip", "middle_dip", "middle_tip"],
            ),   FingerChain(  # Index
                links=[
                   "{wrist}",
                    "rl_dg_4_2",
                    "rl_dg_4_3",
                    "rl_dg_4_4",
                    "rl_dg_4_tip",
                ],
                human=["wrist", "ring_mcp", "ring_pip", "ring_dip", "ring_tip"],
            ),   FingerChain(  # Index
                links=[
                   "{wrist}",
                    "rl_dg_5_2",
                    "rl_dg_5_3",
                    "rl_dg_5_4",
                    "rl_dg_5_tip",
                ],
                human=["wrist", "pinky_mcp", "pinky_pip", "pinky_dip", "pinky_tip"],
            ),
        ],
        "left": [
            FingerChain(  # Thumb 
                links=[
                    "{wrist}",
                    "ll_dg_1_1",
                    "ll_dg_1_2",
                    "ll_dg_1_3",
                    "ll_dg_1_4",
                    "ll_dg_1_tip",
                ],
                human=["wrist", "thumb_cmc0", "thumb_cmc1", "thumb_mcp", "thumb_ip", "thumb_tip"],
            ),
            FingerChain(  # Index
                links=[
                   "{wrist}",
                    "ll_dg_2_2",
                    "ll_dg_2_3",
                    "ll_dg_2_4",
                    "ll_dg_2_tip",
                ],
                human=["wrist", "index_mcp", "index_pip", "index_dip", "index_tip"],
            ),
               FingerChain(  # Index
                links=[
                   "{wrist}",
                    "ll_dg_3_2",
                    "ll_dg_3_3",
                    "ll_dg_3_4",
                    "ll_dg_3_tip",
                ],
                human=["wrist", "middle_mcp", "middle_pip", "middle_dip", "middle_tip"],
            ),   FingerChain(  # Index
                links=[
                   "{wrist}",
                    "ll_dg_4_2",
                    "ll_dg_4_3",
                    "ll_dg_4_4",
                    "ll_dg_4_tip",
                ],
                human=["wrist", "ring_mcp", "ring_pip", "ring_dip", "ring_tip"],
            ),   FingerChain(  # Index
                links=[
                   "{wrist}",
                    "ll_dg_5_2",
                    "ll_dg_5_3",
                    "ll_dg_5_4",
                    "ll_dg_5_tip",
                ],
                human=["wrist", "pinky_mcp", "pinky_pip", "pinky_dip", "pinky_tip"],
            ),
        ],
    }
