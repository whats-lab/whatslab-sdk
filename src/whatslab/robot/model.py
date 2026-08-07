from __future__ import annotations

import os
from dataclasses import fields
from typing import List, Optional, Union

import numpy as np
import pinocchio as pin
from scipy.spatial.transform import Rotation

from whatslab.solvers.arm.builders import backend_cls
from whatslab.solvers.hand import HandRetargetController

from .config import RigConfig, load_rig

_RIG_ONLY_SOLVER_KEYS = frozenset({
    "backend", "w_pos", "w_ori", "max_joint_velocity", "reach_max",
    "joint_weights", "orientation_joints",
})


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

    def __init__(self, rig: Union[str, "os.PathLike[str]", RigConfig]):
        if not isinstance(rig, RigConfig):
            rig = load_rig(os.fspath(rig))
        self.rig = rig
        self.has_arm = rig.arm is not None
        self.has_hand = rig.hand is not None

        root = rig.arm if self.has_arm else rig.hand
        self._M = rig.mount.T @ root.axis_align.T
        self._M_inv = np.linalg.inv(self._M)

        self.solver = None
        self.arm_joint_names: List[str] = []
        if self.has_arm:
            self.solver = self._build_arm_solver()
            self.solver.set_joint_weights(rig.solver.joint_weights)
            if hasattr(self.solver, "set_task_split"):
                self.solver.set_task_split(rig.solver.orientation_joints)
            self._apply_solver_tuning(self.solver)
            self.arm_joint_names = list(self.solver.active_joint_names())

    def _apply_solver_tuning(self, solver) -> None:
        sol = self.rig.solver
        for f in fields(sol):
            if f.name in _RIG_ONLY_SOLVER_KEYS:
                continue
            val = getattr(sol, f.name)
            if val is not None and hasattr(solver, f.name):
                setattr(solver, f.name, val)

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

    def make_hand_controller(self, config_name: str, side: str):
        return HandRetargetController(side, config_name)

    def to_base(self, T_canonical: np.ndarray) -> np.ndarray:
        return self._M_inv @ np.asarray(T_canonical, dtype=float)

    def to_canonical(self, T_base: np.ndarray) -> np.ndarray:
        return self._M @ np.asarray(T_base, dtype=float)

    def clamp_reach(self, T_base: np.ndarray) -> np.ndarray:
        return clamp_reach(T_base, self.rig.solver.reach_max)

    def ee_pose(self, q_arm: np.ndarray) -> np.ndarray:
        assert self.has_arm
        return self.to_canonical(self.solver.fk(np.asarray(q_arm, dtype=float)))

    def sync_state(self, q_arm) -> None:
        if self.solver is not None:
            self.solver.sync_state(q_arm)
