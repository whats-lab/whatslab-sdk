from typing import ClassVar, Dict, List

import numpy as np

from ._base import FingerChain, HandConfig


class RobotisHX5Config(HandConfig):
    """Robotis HX5 핸드 설정.

    - {side} : 'l' / 'r' 약어
    - {wrist} : wrist base 링크명
    """

    _MODEL_SUBDIR = "robotis_hx5_d20"
    _URDF_FILENAME = "urdf/hx5_d20_{hand_type}.urdf"
    _RVIZ_FILENAME = {
        "left": "robotis_hx5_d20_left.rviz",
        "right": "robotis_hx5_d20_right.rviz",
    }
    _SIDE_MAP = {"left": "l", "right": "r"}
    _WRIST_LINK = {"left": "robotis_hx5_d20_left", "right": "hx5_d20_right_base"}
    _COORD_TRANSFORM: ClassVar[np.ndarray] = np.array(
        [[0, 1, 0], [0, 0, 1], [1, 0, 0]], dtype=np.float32
    )
    _SCALE_FACTOR = [1.2, 1.25, 1.25, 1.3, 1.4]

    _chains = [
        FingerChain(  # Thumb
            links=[
                "{wrist}",
                "finger_{side}_link1",
                "finger_{side}_link2",
                "finger_{side}_link3",
                "finger_{side}_link4",
                "finger_end_{side}_link1",
            ],
            human=["wrist", "thumb_cmc0", "thumb_cmc1", "thumb_mcp", "thumb_ip", "thumb_tip"],
        ),
        FingerChain(  # Index
            links=[
                "{wrist}",
                "finger_{side}_link5",
                "finger_{side}_link7",
                "finger_{side}_link8",
                "finger_end_{side}_link2",
            ],
            human=["wrist", "index_mcp", "index_pip", "index_dip", "index_tip"],
        ),
        FingerChain(  # Middle
            links=[
                "{wrist}",
                "finger_{side}_link9",
                "finger_{side}_link11",
                "finger_{side}_link12",
                "finger_end_{side}_link3",
            ],
            human=["wrist", "middle_mcp", "middle_pip", "middle_dip", "middle_tip"],
        ),
        FingerChain(  # Ring
            links=[
                "{wrist}",
                "finger_{side}_link13",
                "finger_{side}_link15",
                "finger_{side}_link16",
                "finger_end_{side}_link4",
            ],
            human=["wrist", "ring_mcp", "ring_pip", "ring_dip", "ring_tip"],
        ),
        FingerChain(  # Pinky
            links=[
                "{wrist}",
                "finger_{side}_link17",
                "finger_{side}_link19",
                "finger_{side}_link20",
                "finger_end_{side}_link5",
            ],
            human=["wrist", "pinky_mcp", "pinky_pip", "pinky_dip", "pinky_tip"],
        ),
    ]
    _FINGERS: ClassVar[Dict[str, List[FingerChain]]] = {
        "left": _chains,
        "right": _chains,
    }
