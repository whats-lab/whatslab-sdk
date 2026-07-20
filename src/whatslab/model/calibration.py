"""캘리브 모델 — yaw 정렬 + reach 스케일.

팔 자세는 이미 **머리-상대(head-relative)** 로 들어온다(TeleopModel.get_data 가
Rz(-머리yaw)·G 로 변환). 그래서 이 모델은 hmd 를 알 필요가 없고, 캘리브는 단지
**그 상대 손목 rot 의 yaw 를 0 으로 만드는 보정 W 를 캡처**해 두었다가 `W·G_rel`
로 적용한다(캡처 순간 = yaw 0, 이후 손목이 돌면 그만큼 반영).

인스턴스 하나는 **한 side(rig)** 에 대응한다(TeleopModel 이 side 별로 하나씩
만든다). 따라서 상태(W/input_reach)는 side 로 키잉하지 않고 인스턴스당 하나만 둔다
— 단일 rig 를 좌우 공용(_pick sole-fallback)으로 쓸 때도 동일 인스턴스라 캘리브가
양쪽에 그대로 반영된다.

  · apply(data) -> data       : arm_pose(=Pose, head-relative quat) → reach 스케일 +
                               W 적용 → 정준 목표 `arm_target`(4x4).
  · capture(data) -> bool     : 현재 상대 손목 rot 의 yaw 를 0 으로 만드는 W 저장.
  · set_reach(input_reach)    : reach 스케일용 사람 도달반경 등록.
"""
from __future__ import annotations

from typing import Optional

import numpy as np
from scipy.spatial.transform import Rotation


def _yaw(R: np.ndarray) -> float:
    return float(np.arctan2(R[1, 0], R[0, 0]))


def _Rz(a: float) -> np.ndarray:
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


class ArmCalibration:
    """yaw-0 보정(W) + reach 스케일. reach_max=로봇 도달반경(스케일용)."""

    def __init__(self, reach_max: Optional[float] = None,
                 input_reach: Optional[float] = None):
        self.reach_max = reach_max
        self._W: Optional[np.ndarray] = None       # yaw-0 보정(Rz), 미캡처면 None
        # 사람 도달반경(reach 스케일). rig 의 calibration.input_reach 로 초기화되고
        # calibrate_reach 가 갱신한다(persist 하면 rig yaml 에도 저장).
        self._input_reach: Optional[float] = input_reach

    def apply(self, data: dict) -> dict:
        """arm_pose(head-relative) → reach 스케일 + W 적용 → data["arm_target"]."""
        pose = data.get("arm_pose")
        if pose is None:                        # 팔 목표 없음(=미추적/미사용 side) → 스킵
            data["arm_target"] = None
            return data
        pos = np.asarray(pose.pos, dtype=float)
        if self._input_reach and self.reach_max:           # reach 스케일(사람→로봇)
            pos = pos * (self.reach_max / self._input_reach)
        G = Rotation.from_quat(np.asarray(pose.quat, dtype=float)).as_matrix()
        T = np.eye(4)
        T[:3, 3] = pos
        T[:3, :3] = (self._W @ G) if self._W is not None else G  # 캡처 전엔 상대 rot 그대로
        data["arm_target"] = T
        return data

    def capture(self, data: dict) -> bool:
        """현재 상대 손목 rot 의 yaw 를 0 으로 만드는 보정 W = Rz(-yaw(G)) 저장."""
        pose = data.get("arm_pose")
        if pose is None:
            return False
        G = Rotation.from_quat(np.asarray(pose.quat, dtype=float)).as_matrix()
        self._W = _Rz(-_yaw(G))
        return True

    def set_reach(self, input_reach: float) -> None:
        self._input_reach = float(input_reach)

    @property
    def ready(self) -> bool:
        return self._W is not None
