import logging
import os
from typing import List

import numpy as np

logger = logging.getLogger(__name__)

try:
    from dex_retargeting.retargeting_config import RetargetingConfig
except ImportError as e:  # pragma: no cover
    raise ImportError(
        "dex_retargeting 필요: pip install 'whatslab-sdk[hand]'"
    ) from e

from whatslab.teleop.core.constants import (
    HUBER_DELTA,
    IK_MAX_EVAL,
    NORM_DELTA,
    POSITION_WEIGHT,
    VECTOR_WEIGHT,
)

from .hand_configs import CONFIG_REGISTRY, HandConfig
from .spherical_fk import HandSphericalFK


class HandRetargeter:

    def __init__(
        self,
        hand_type: str,
        config_name: str = 'base_hand',
        vector_weight: float = VECTOR_WEIGHT,
        position_weight: float = POSITION_WEIGHT,
        urdf_root=None,
    ):
        if config_name not in CONFIG_REGISTRY:
            raise ValueError(
                f"Unknown robot_config '{config_name}'. Available: {list(CONFIG_REGISTRY.keys())}"
            )
        config = CONFIG_REGISTRY[config_name](urdf_root=urdf_root)

        self.hand_type = hand_type.lower()
        models_root = getattr(config, '_models_root', None)
        fk_urdf = (os.path.join(models_root, 'base_hand', 'urdf', f'{self.hand_type}.urdf')
                   if models_root else None)
        self.fk        = HandSphericalFK(self.hand_type, urdf_path=fk_urdf)

        self._coord_transform = config.get_coord_transform(self.hand_type)

        sf = config.get_scale_factor()
        if isinstance(sf, list):
            self._scale_array  = self._build_scale_array(config, self.hand_type, sf)
            self._scale_factor = 1.0
        else:
            self._scale_array  = None
            self._scale_factor = float(sf)

        _ = (vector_weight, position_weight)

        s1_dict, s2_dict = config.get_two_stage_config(self.hand_type)
        s1_dict.update({'normal_delta': NORM_DELTA, 'huber_delta': HUBER_DELTA})
        s2_dict.update({'normal_delta': NORM_DELTA, 'huber_delta': HUBER_DELTA})

        cfg1 = RetargetingConfig.from_dict(s1_dict)
        self._seq_stage1 = cfg1.build()
        cfg2 = RetargetingConfig.from_dict(s2_dict)
        self._seq_stage2 = cfg2.build()
        for seq in (self._seq_stage1, self._seq_stage2):
            seq.optimizer.opt.set_maxtime(0.0)
            seq.optimizer.opt.set_maxeval(IK_MAX_EVAL)

        s1_human            = np.array(cfg1.target_link_human_indices)
        self._s1_origin_idx = s1_human[0].astype(np.int32)
        self._s1_task_idx   = s1_human[1].astype(np.int32)
        self._s2_tip_idx    = np.array(cfg2.target_link_human_indices, dtype=np.int32)
        self.tip_human_indices = self._s2_tip_idx.tolist()

        robot               = self._seq_stage1.optimizer.robot
        all_names           = list(robot.dof_joint_names)
        fixed_names         = set(config.get_fixed_joint_names(self.hand_type))
        self._keep_indices  = [i for i, n in enumerate(all_names)
                               if n not in fixed_names]
        self.joint_names    = [n for n in all_names if n not in fixed_names]

        try:
            widx = robot.get_link_index(config.get_wrist_link_name(self.hand_type))
            robot.compute_forward_kinematics(robot.q0)
            self._wrist_offset = robot.get_link_pose(widx)[:3, 3].astype(np.float64).copy()
        except Exception as e:
            logger.warning("손목 링크(%s) 오프셋 계산 실패 → 0 사용, 리타게팅이 어긋날 수 있음: %s",
                           config.get_wrist_link_name(self.hand_type), e)
            self._wrist_offset = np.zeros(3, dtype=np.float64)


    def compute(self, sensor_quats_17: np.ndarray) -> np.ndarray:
        positions          = self.fk.compute_positions(sensor_quats_17)
        positions_centered = positions - positions[0]
        positions_robot    = (self._coord_transform @ positions_centered.T).T

        if self._scale_array is not None:
            positions_robot *= self._scale_array[:, None]
        elif self._scale_factor != 1.0:
            positions_robot *= self._scale_factor

        self.last_human_positions = positions_robot

        positions_ik = positions_robot + self._wrist_offset
        robot_qpos = self._two_stage_retarget(positions_ik)

        return robot_qpos[self._keep_indices]


    def _two_stage_retarget(self, positions_robot: np.ndarray) -> np.ndarray:
        ref_vec     = positions_robot[self._s1_task_idx] - positions_robot[self._s1_origin_idx]
        stage1_qpos = self._seq_stage1.retarget(ref_vec)
        self._seq_stage2.set_qpos(stage1_qpos)
        return self._seq_stage2.retarget(positions_robot[self._s2_tip_idx])

    @staticmethod
    def _build_scale_array(config: HandConfig, hand_type: str, sf_list: List[float]) -> np.ndarray:
        fingers = config._get_fingers(hand_type)
        arr = np.ones(23, dtype=np.float32)
        for i, f in enumerate(fingers):
            scale = float(sf_list[i]) if i < len(sf_list) else 1.0
            for idx in f.human[1:]:
                arr[idx] = scale
        return arr
