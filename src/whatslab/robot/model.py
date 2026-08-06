from __future__ import annotations

from typing import List, Optional

import numpy as np
import pinocchio as pin
from scipy.spatial.transform import Rotation

from whatslab.teleop.arm.builders import backend_cls

from .config import RigConfig, load_rig


def clamp_reach(T_base: np.ndarray, reach_max: Optional[float]) -> np.ndarray:
    T_b = np.asarray(T_base, dtype=float)
    if not reach_max:
        return T_b
    n = float(np.linalg.norm(T_b[:3, 3]))
    if n <= reach_max:
        return T_b
    T_b = T_b.copy()
    T_b[:3, 3] *= reach_max / n
    return T_b


class RobotModel:

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
            self._apply_solver_tuning(self.solver)
            self.arm_joint_names = list(self.solver.active_joint_names())

    def _apply_solver_tuning(self, solver) -> None:
        sol = self.rig.solver
        for attr, val in (("max_iter", sol.max_iter),
                          ("iters_per_call", sol.iters_per_call),
                          ("tol", sol.tol),
                          ("sugihara_bias", sol.sugihara_bias)):
            if val is not None and hasattr(solver, attr):
                setattr(solver, attr, val)

    # ---------------------------------------------------------------- build
    def _build_arm_solver(self):
        rig = self.rig
        arm = rig.arm
        cls = backend_cls(rig.solver.backend)
        common = dict(w_pos=rig.solver.w_pos, w_ori=rig.solver.w_ori)

        if self.has_hand:

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
        from whatslab.teleop.hand import HandRetargetController
        return HandRetargetController(side, config_name)

    # ------------------------------------------------------ 정준 데카르트 API
    def to_base(self, T_canonical: np.ndarray) -> np.ndarray:
        return self._M_inv @ np.asarray(T_canonical, dtype=float)

    def to_canonical(self, T_base: np.ndarray) -> np.ndarray:
        return self._M @ np.asarray(T_base, dtype=float)

    def solve(self, T_canonical: np.ndarray) -> np.ndarray:
        assert self.has_arm, "arm 없는 rig — solve 불가"
        sol = self.rig.solver
        T_c = np.asarray(T_canonical, dtype=float).copy()

        # uniform reach 스케일 (사람 도달반경 → 로봇 reach). 원점 0 기준 등방.
        cal = self.rig.calibration
        if cal.enabled and cal.input_reach and sol.reach_max:
            T_c[:3, 3] *= sol.reach_max / cal.input_reach

        return self.solver.solve(self.clamp_reach(self.to_base(T_c)))

    def clamp_reach(self, T_base: np.ndarray) -> np.ndarray:
        return clamp_reach(T_base, self.rig.solver.reach_max)

    def ee_pose(self, q_arm: np.ndarray) -> np.ndarray:
        assert self.has_arm
        return self.to_canonical(self.solver.fk(np.asarray(q_arm, dtype=float)))

    def sync_state(self, q_arm) -> None:
        if self.solver is not None:
            self.solver.sync_state(q_arm)
