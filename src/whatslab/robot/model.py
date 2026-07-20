"""RobotModel — "로봇이 무엇인가"의 단일 출처 (무상태, 정준 샌드위치).

  · rig config 로 arm/hand 를 부분 조립 (둘 다 optional)
  · 데카르트 입력(목표 pose)은 정준 프레임 — 내부에서 [정준→베이스] 후 IK
  · q 출력은 무변환(관절공간은 프레임 무관)
  · 데카르트 출력(ee_pose 등)은 요청 시에만 [베이스→정준] (lazy)

무상태 원칙: 이 클래스는 정의 + 기하 함수만 가진다. 현재 q·목표·평활 같은
상태는 드라이버/파이프라인 소유. (IK 백엔드 내부의 warm-start history 는
솔버 구현 세부 — 모델 밖에서 공유 금지)
"""
from __future__ import annotations

from typing import List

import numpy as np
import pinocchio as pin
from scipy.spatial.transform import Rotation

from whatslab.teleop.arm.builders import backend_cls

from .config import RigConfig, load_rig


class RobotModel:
    """rig 조립 결과. 공개 데카르트 API 는 전부 정준 프레임."""

    def __init__(self, rig: RigConfig):
        self.rig = rig
        self.has_arm = rig.arm is not None
        self.has_hand = rig.hand is not None

        # 정준 → 루트 로봇 베이스 (URDF 좌표). mount ∘ axis_align 합성.
        root = rig.arm if self.has_arm else rig.hand
        self._M = rig.mount.T @ root.axis_align.T          # 4x4
        self._M_inv = np.linalg.inv(self._M)

        self.solver = None           # 팔 IK (arm 있을 때만)
        self.arm_joint_names: List[str] = []
        if self.has_arm:
            self.solver = self._build_arm_solver()
            self.arm_joint_names = list(self.solver.active_joint_names())

    # ---------------------------------------------------------------- build
    def _build_arm_solver(self):
        rig = self.rig
        arm = rig.arm
        cls = backend_cls(rig.solver.backend)
        common = dict(w_pos=rig.solver.w_pos, w_ori=rig.solver.w_ori)

        if self.has_hand:
            # 결합 모델: arm ee.parent 프레임에 [ee.origin ∘ attach] 로 손 부착.
            # 방향 정렬은 전부 config(URDF origin 표기) — 하드코딩 회전 없음.
            # 부착 체인 = ee.origin ∘ attach ∘ hand.axis_align
            # (1단계에서 손을 원점 정합해 두면 attach ≈ 항등)
            # 활성 조인트는 from_appended 가 지지 체인(universe→target_ee)에서
            # 산출 — 손목 구동 관절(orca 카펄 등)도 잠금이 아니면 팔 IK 대상.
            aMb_T = arm.ee_origin.T @ rig.attach.T @ rig.hand.axis_align.T
            rpy = Rotation.from_matrix(aMb_T[:3, :3]).as_euler("xyz")
            return cls.from_appended(
                arm_urdf=arm.urdf_abspath(),
                hand_urdf=rig.hand.urdf_abspath(),
                attach_frame=arm.ee_parent,
                ee_link=rig.resolve_target_ee(),
                mount_xyz=aMb_T[:3, 3].tolist(),
                mount_rpy=rpy.tolist(),
                locked_joints=rig.lock_joints,
                # target_ee 프레임 정렬은 hand config(ee_align)가 소유 —
                # 메쉬 불변, IK 제어 프레임 축만 회전 (align_frames ee 모드로 튜닝)
                ee_local_rpy=list(rig.hand.ee_align.rpy),
                **common,
            )
        # arm 단독: TCP 프레임("ee")을 ee.origin(URDF origin 관례)으로 등록.
        # ee.parent 의 지지 조인트(fixed 프레임이면 그 프레임을 지탱하는 조인트)가
        # 잠기면 TCP 를 구동할 수 없다 → 에러.
        m_arm = pin.buildModelFromUrdf(arm.urdf_abspath())
        if m_arm.existJointName(arm.ee_parent):
            support_joint = arm.ee_parent
        else:
            fr = m_arm.frames[m_arm.getFrameId(arm.ee_parent)]
            jp = getattr(fr, "parentJoint", None)
            support_joint = m_arm.names[int(jp if jp is not None else fr.parent)]
        if support_joint in set(rig.lock_joints):
            raise ValueError(
                f"lock_joints 가 ee.parent({arm.ee_parent})의 지지 조인트"
                f"({support_joint})를 잠금 — TCP 를 구동할 수 없습니다")
        # ArmIK 의 tool 인자는 (회전 후 회전축 기준 이동) 관례라 변환해 전달:
        #   T(R,p) = Rot(rpy) · Trans(Rᵀp)
        R = arm.ee_origin.T[:3, :3]
        p = np.asarray(arm.ee_origin.xyz, dtype=float)
        return cls(
            urdf_path=arm.urdf_abspath(),
            package_dirs=[],
            locked_joints=list(rig.lock_joints),
            ee_parent_joint=arm.ee_parent,
            ee_frame_name="ee",
            tool_pre_rot_rpy=Rotation.from_matrix(R).as_euler("xyz").tolist(),
            tool_translation_xyz=(R.T @ p).tolist(),
            **common,
        )

    # ------------------------------------------------------------ factories
    @classmethod
    def from_yaml(cls, path: str) -> "RobotModel":
        return cls(load_rig(path))

    def make_hand_controller(self, config_name: str, side: str):
        """리타게팅 드라이버용 컨트롤러 (CONFIG_REGISTRY 이름 참조 — 모델과 분리)."""
        from whatslab.teleop.hand import HandRetargetController
        return HandRetargetController(side, config_name)

    # ------------------------------------------------------ 정준 데카르트 API
    def to_base(self, T_canonical: np.ndarray) -> np.ndarray:
        """정준 pose → 베이스 pose (in-leg)."""
        return self._M_inv @ np.asarray(T_canonical, dtype=float)

    def to_canonical(self, T_base: np.ndarray) -> np.ndarray:
        """베이스 pose → 정준 pose (out-leg, lazy 용)."""
        return self._M @ np.asarray(T_base, dtype=float)

    def solve(self, T_canonical: np.ndarray) -> np.ndarray:
        """정준 목표 pose → q_arm. 내부: uniform 스케일 → 정준→베이스 → reach 클램프 → IK.

        위치는 (캘리브 시) `s = reach_max / input_reach` 단일 스칼라로 등방 스케일한
        뒤(원점 0 기준) 베이스로 옮기고, reach_max 구로 클램프한다(안전망).
        (workspace 박스 매핑은 폐기 — reach_max 기준)
        """
        assert self.has_arm, "arm 없는 rig — solve 불가"
        sol = self.rig.solver
        T_c = np.asarray(T_canonical, dtype=float).copy()

        # uniform reach 스케일 (사람 도달반경 → 로봇 reach). 원점 0 기준 등방.
        cal = self.rig.calibration
        if cal.enabled and cal.input_reach and sol.reach_max:
            T_c[:3, 3] *= sol.reach_max / cal.input_reach

        T_b = self.to_base(T_c)
        if sol.reach_max:
            n = float(np.linalg.norm(T_b[:3, 3]))
            if n > sol.reach_max:
                T_b[:3, 3] *= sol.reach_max / n

        return self.solver.solve(T_b)

    def ee_pose(self, q_arm: np.ndarray) -> np.ndarray:
        """q → target_ee 의 정준 4x4 pose (lazy out-leg)."""
        assert self.has_arm
        return self.to_canonical(self.solver.fk(np.asarray(q_arm, dtype=float)))

    def sync_state(self, q_arm) -> None:
        """IK warm-start 를 현재 관절각으로 동기화 (솔버 위임)."""
        if self.solver is not None:
            self.solver.sync_state(q_arm)
