from __future__ import annotations

import os
from dataclasses import fields
from typing import Dict, List, Optional, Tuple, Union

import numpy as np
import pinocchio as pin
from scipy.spatial.transform import Rotation

from whatslab.solvers.arm.builders import backend_cls

from .config import RigConfig, load_rig

_RIG_ONLY_SOLVER_KEYS = frozenset({
    "backend", "w_pos", "w_ori", "max_joint_velocity", "reach_max",
    "joint_weights", "posture",
})


def clamp_reach(tf_base: np.ndarray, reach_max: Optional[float]) -> np.ndarray:
    """베이스 프레임 목표의 위치를 원점에서 reach_max 이내로 줄인다(None·0 이면 그대로)."""
    tf_b = np.asarray(tf_base, dtype=float)
    if not reach_max:
        return tf_b
    n = float(np.linalg.norm(tf_b[:3, 3]))
    if n <= reach_max:
        return tf_b
    tf_b = tf_b.copy()
    tf_b[:3, 3] *= reach_max / n
    return tf_b


class RobotModel:

    def __init__(self, rig: Union[str, "os.PathLike[str]", RigConfig]):
        if not isinstance(rig, RigConfig):
            rig = load_rig(os.fspath(rig))
        self.rig = rig
        self.has_arm = rig.arm is not None
        self.has_hand = rig.hand is not None

        root = rig.arm if self.has_arm else rig.hand
        self._tf_to_canonical = rig.mount.transform @ root.axis_align.transform
        self._tf_to_base = np.linalg.inv(self._tf_to_canonical)

        self.solver = None
        self.arm_joint_names: List[str] = []
        if self.has_arm:
            self.solver = self._build_arm_solver()
            self.solver.set_joint_weights(rig.solver.joint_weights)
            self._apply_solver_tuning(self.solver)
            self.arm_joint_names = list(self.solver.active_joint_names())
            if rig.solver.posture and hasattr(self.solver, "q_posture"):
                # 적지 않은 관절은 0 (범위 안으로) — 범위 가운데는 비대칭 한계에서
                # 엉뚱한 자세가 된다
                q = np.array(
                    [rig.solver.posture.get(n, 0.0) for n in self.arm_joint_names], dtype=float
                )
                self.solver.q_posture = np.clip(q, self.solver._lo, self.solver._hi)

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
            a_to_b_t = (
                arm.ee_origin.transform @ rig.attach.transform @ rig.hand.axis_align.transform
            )
            rpy = Rotation.from_matrix(a_to_b_t[:3, :3]).as_euler("xyz")
            return cls.from_appended(
                arm_urdf=arm.urdf_abspath(),
                hand_urdf=rig.hand.urdf_abspath(),
                attach_frame=arm.ee_parent,
                ee_link=rig.resolve_target_ee(),
                mount_xyz=a_to_b_t[:3, 3].tolist(),
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
                f"({support_joint})를 잠금 — TCP 를 구동할 수 없습니다"
            )
        rot = arm.ee_origin.transform[:3, :3]
        p = np.asarray(arm.ee_origin.xyz, dtype=float)
        return cls(
            urdf_path=arm.urdf_abspath(),
            locked_joints=list(rig.lock_joints),
            ee_parent_joint=arm.ee_parent,
            ee_frame_name="ee",
            tool_pre_rot_rpy=Rotation.from_matrix(rot).as_euler("xyz").tolist(),
            tool_translation_xyz=(rot.T @ p).tolist(),
            **common,
        )

    def hand_joint_limits(self) -> Dict[str, Tuple[float, float]]:
        """손 로봇 URDF 의 관절 범위 {관절: (lo, hi)}."""
        from whatslab.safety import load_limits_from_urdf

        with open(self.rig.hand.urdf_abspath(), encoding="utf-8") as f:
            limits = load_limits_from_urdf(f.read())
        return {name: (lim.lower, lim.upper) for name, lim in limits.items()}

    def make_hand_controller(self, config_name: str, side: str):
        """robot yaml 의 retarget(dex:<손> | geort:<설정>)으로 손 리타게팅 컨트롤러를 만든다.

        geort 는 학습 범위에 더해 rig 손 URDF 범위로도 자른다. dex 는 자기 설정의 URDF 범위를
        쓰므로 rig 손 URDF 를 읽지 않는다.
        """
        from whatslab.solvers.hand.controller import (
            GEORT_PREFIX,
            HandRetargetController,
            engine_of,
        )

        limits = self.hand_joint_limits() if engine_of(config_name) == GEORT_PREFIX else None
        return HandRetargetController(side, config_name, robot_limits=limits)

    def to_base(self, tf_canonical: np.ndarray) -> np.ndarray:
        """정준 프레임 4×4 를 로봇 베이스 프레임으로."""
        return self._tf_to_base @ np.asarray(tf_canonical, dtype=float)

    def to_canonical(self, tf_base: np.ndarray) -> np.ndarray:
        """로봇 베이스 프레임 4×4 를 정준 프레임으로."""
        return self._tf_to_canonical @ np.asarray(tf_base, dtype=float)

    def clamp_reach(self, tf_base: np.ndarray) -> np.ndarray:
        """rig 의 solver.reach_max 로 `clamp_reach` 한다."""
        return clamp_reach(tf_base, self.rig.solver.reach_max)

    def ee_pose(self, q_arm: np.ndarray) -> np.ndarray:
        assert self.has_arm
        return self.to_canonical(self.solver.fk(np.asarray(q_arm, dtype=float)))

    def sync_state(self, q_arm) -> None:
        if self.solver is not None:
            self.solver.sync_state(q_arm)
