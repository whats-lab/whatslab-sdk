"""
retargeter.py
Hand 리타겟팅 파이프라인 코어 (ROS2 비의존)

사용법:
    from whatslab.teleop.hand import HandRetargeter
    retargeter = HandRetargeter('right', 'orca_hand', urdf_root='/path/to/models')
    joint_angles = retargeter.compute(sensor_quats_17)  # shape (17, 4)
"""

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
    """AGA 센서 쿼터니언 → 로봇 조인트 각도 변환기.

    FK → 좌표 변환 → 스케일링 → 2단계 IK (vector + position)
    (손목 자세는 여기서 다루지 않는다 — 팔 IK 소관)
    """

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
        # 사람 손 FK 는 base_hand URDF 를 쓴다. config 가 해석한 models root
        # (urdf_root 인자 > WHATSLAB_MODELS_ROOT > 내장 fallback)를 그대로 사용.
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

        # vector_weight/position_weight: 과거 maxtime 예산을 두 스테이지에 배분하던
        # 값. maxeval(반복) 종료로 바뀌며 무의미해졌으나, 레거시 위치인자 호출
        # (예: ROS2 retargeting_node 의 HandRetargeter(..., vw, pw)) 호환 위해 유지.
        _ = (vector_weight, position_weight)

        s1_dict, s2_dict = config.get_two_stage_config(self.hand_type)
        s1_dict.update({'normal_delta': NORM_DELTA, 'huber_delta': HUBER_DELTA})
        s2_dict.update({'normal_delta': NORM_DELTA, 'huber_delta': HUBER_DELTA})

        cfg1 = RetargetingConfig.from_dict(s1_dict)
        self._seq_stage1 = cfg1.build()
        cfg2 = RetargetingConfig.from_dict(s2_dict)
        self._seq_stage2 = cfg2.build()
        # 반복(iteration) 기반 종료 → 결정적 + 수렴. 시간 종료는 비결정적이라 해제.
        # (기존 set_maxtime 은 벽시계 의존 → 실행/부하마다 다른 해로 종료되어 떨림)
        for seq in (self._seq_stage1, self._seq_stage2):
            seq.optimizer.opt.set_maxtime(0.0)
            seq.optimizer.opt.set_maxeval(IK_MAX_EVAL)

        s1_human            = np.array(cfg1.target_link_human_indices)
        self._s1_origin_idx = s1_human[0].astype(np.int32)
        self._s1_task_idx   = s1_human[1].astype(np.int32)
        self._s2_tip_idx    = np.array(cfg2.target_link_human_indices, dtype=np.int32)
        # IK가 실제로 위치를 맞추는 손가락 팁의 human 인덱스 (TF 시각화용)
        self.tip_human_indices = self._s2_tip_idx.tolist()

        robot               = self._seq_stage1.optimizer.robot
        all_names           = list(robot.dof_joint_names)
        fixed_names         = set(config.get_fixed_joint_names(self.hand_type))
        # fixed 관절(예: orca 카펄 = 손목 회전)은 손 리타게팅이 구동하지 않는다.
        # 손목 관절은 팔 IK 소관이므로 joint_names/출력에서 제외한다(팔이 소유).
        self._keep_indices  = [i for i, n in enumerate(all_names)
                               if n not in fixed_names]
        self.joint_names    = [n for n in all_names if n not in fixed_names]

        # wrist 링크의 URDF root 프레임 기준 위치 (root에 fixed → 상수).
        # PositionOptimizer는 로봇 팁을 root 프레임에서 비교하므로, wrist 기준인
        # human 타깃을 이 오프셋만큼 이동시켜 같은 space에서 IK를 풀게 한다.
        try:
            widx = robot.get_link_index(config.get_wrist_link_name(self.hand_type))
            robot.compute_forward_kinematics(robot.q0)
            self._wrist_offset = robot.get_link_pose(widx)[:3, 3].astype(np.float64).copy()
        except Exception:
            self._wrist_offset = np.zeros(3, dtype=np.float64)

    # ─────────────────────────────────────────

    def compute(self, sensor_quats_17: np.ndarray) -> np.ndarray:
        """17×4 센서 쿼터니언 → 로봇 조인트 각도 (rad).

        Returns:
            np.ndarray: len(joint_names) 크기의 조인트 각도 배열
        Side-effect:
            self.last_human_positions 에 변환 후 human 포지션 저장 (TF 발행용)
        """
        positions          = self.fk.compute_positions(sensor_quats_17)
        positions_centered = positions - positions[0]
        positions_robot    = (self._coord_transform @ positions_centered.T).T

        if self._scale_array is not None:
            positions_robot *= self._scale_array[:, None]
        elif self._scale_factor != 1.0:
            positions_robot *= self._scale_factor

        self.last_human_positions = positions_robot  # wrist 기준 (TF 발행용)

        # IK는 URDF root 프레임에서 계산 (로봇 팁 위치와 같은 space).
        # vector stage는 상대벡터라 오프셋이 상쇄되고, position stage만 정합됨.
        positions_ik = positions_robot + self._wrist_offset
        robot_qpos = self._two_stage_retarget(positions_ik)

        # fixed 관절(손목 카펄 등)은 손 출력에서 제외 — 팔 IK 소관
        return robot_qpos[self._keep_indices]

    # ─────────────────────────────────────────

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
