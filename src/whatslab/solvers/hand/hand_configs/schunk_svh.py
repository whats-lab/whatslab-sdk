from typing import ClassVar, Dict, List
import numpy as np

from ._base import FingerChain, HandConfig


class SchunkSVHConfig(HandConfig):

    _MODEL_SUBDIR  = "schunk_hand"
    _URDF_FILENAME = "schunk_svh_hand_{hand_type}.urdf"
    _RVIZ_FILENAME = {"left": "schunk_hand.rviz", "right": "schunk_hand.rviz"}
    _WRIST_LINK    = {"left": "left_hand_e1", "right": "right_hand_e1"}
    _COORD_TRANSFORM: ClassVar[np.ndarray] = np.array(
        [[0, 0, -1], [0, 1,0], [1, 0, 0]], dtype=np.float32
    )
    _TARGET_JOINT_NAMES = {
        side: [
            f"{side}_hand_Thumb_Flexion",
            f"{side}_hand_Thumb_Opposition",
            f"{side}_hand_Index_Finger_Proximal",
            f"{side}_hand_Index_Finger_Distal",
            f"{side}_hand_Middle_Finger_Proximal",
            f"{side}_hand_Middle_Finger_Distal",
            f"{side}_hand_Ring_Finger",
            f"{side}_hand_Pinky",
            f"{side}_hand_Finger_Spread",
        ]
        for side in ("left", "right")
    }

    _chains = [
        FingerChain(
            links=["{wrist}", "{side}_hand_z", "{side}_hand_a",
                   "{side}_hand_b", "{side}_hand_c", "thtip"],
            human=["wrist", "thumb_cmc0", "thumb_cmc1", "thumb_mcp", "thumb_ip", "thumb_tip"],
        ),
        FingerChain(
            links=["{wrist}", "{side}_hand_virtual_l", "{side}_hand_l",
                   "{side}_hand_p", "{side}_hand_t", "fftip"],
            human=["wrist", "index_mcp", "index_pip", "index_dip", "index_tip", "index_tip"],
        ),
        FingerChain(
            links=["{wrist}", "{side}_hand_virtual_k", "{side}_hand_k",
                   "{side}_hand_o", "{side}_hand_s", "mftip"],
            human=["wrist", "middle_mcp", "middle_pip", "middle_dip", "middle_tip", "middle_tip"],
        ),
        FingerChain(
            links=["{wrist}", "{side}_hand_e2", "{side}_hand_virtual_j",
                   "{side}_hand_j", "{side}_hand_n", "{side}_hand_r", "rftip"],
            human=["wrist", "ring_mcp", "ring_mcp", "ring_pip", "ring_dip", "ring_tip", "ring_tip"],
        ),
        FingerChain(
            links=["{wrist}", "{side}_hand_e2", "{side}_hand_virtual_i",
                   "{side}_hand_i", "{side}_hand_m", "{side}_hand_q", "lftip"],
            human=["wrist", "pinky_mcp", "pinky_mcp", "pinky_pip", "pinky_dip", "pinky_tip", "pinky_tip"],
        ),
    ]
    _FINGERS: ClassVar[Dict[str, List[FingerChain]]] = {"left": _chains, "right": _chains}
