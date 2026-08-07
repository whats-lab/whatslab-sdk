from __future__ import annotations

import logging

import numpy as np

from .model import clamp_reach

logger = logging.getLogger(__name__)


class RobotArmIK:

    cold_restarts = 40
    cold_pos_tol = 0.002
    cold_ori_tol = 0.02
    cold_max_tries = 5

    def __init__(self, robot):
        assert robot.has_arm, "arm 없는 rig 로 RobotArmIK 불가"
        self._robot = robot
        self.joint_names = list(robot.arm_joint_names)
        self._seeded = False
        self._warm = None
        self._cold_tries = 0

    def solve(self, T_canonical: np.ndarray) -> np.ndarray:
        r = self._robot
        T_b = clamp_reach(r.to_base(np.asarray(T_canonical, dtype=float)),
                          r.rig.solver.reach_max)
        solver = r.solver
        if self._warm is not None and hasattr(solver, "sync_state"):
            solver.sync_state(self._warm)
        if not self._seeded and hasattr(solver, "solve_robust"):
            q = self._cold_start(solver, T_b)
        else:
            q = np.asarray(solver.solve(T_b), dtype=float)
        if q.shape[0] != len(self.joint_names):
            raise ValueError(
                f"IK 해({q.shape[0]}) != arm_joint_names({len(self.joint_names)}) — "
                "rig/solver 관절 구성 불일치")
        self._warm = q.copy()
        return q

    def _cold_start(self, solver, T_b):
        q = np.asarray(solver.solve_robust(
            T_b, restarts=self.cold_restarts, pos_tol=self.cold_pos_tol,
            ori_tol=self.cold_ori_tol, seed=self._cold_tries), dtype=float)
        pe, oe = solver.pose_error(q, T_b)
        self._cold_tries += 1
        locked = (pe <= self.cold_pos_tol and oe <= self.cold_ori_tol)
        if locked or self._cold_tries >= self.cold_max_tries:
            self._seeded = True
            if not locked:
                p = T_b[:3, 3]
                logger.warning(
                    "cold start: 전역 탐색(%d회x%d)이 pos %.1fmm / ori %.1f° 까지만 "
                    "간다. base 좌표 [%+.3f %+.3f %+.3f] |p|=%.3f (reach_max %s)",
                    self._cold_tries, self.cold_restarts, pe * 1e3, np.degrees(oe),
                    p[0], p[1], p[2], float(np.linalg.norm(p)),
                    self._robot.rig.solver.reach_max)
        return q

    def reseed(self) -> None:
        self._seeded = False
        self._cold_tries = 0
        self._warm = None
        solver = getattr(self._robot, "solver", None)
        if solver is not None and hasattr(solver, "sync_state") \
                and hasattr(solver, "q_neutral"):
            solver.sync_state(solver.q_neutral)

    def sync_state(self, q_arm) -> None:
        self._warm = np.array(q_arm, dtype=float)
        self._robot.sync_state(q_arm)

    def ee_pose(self, q_arm) -> np.ndarray:
        return self._robot.ee_pose(q_arm)
