from __future__ import annotations

import numpy as np

from .model import clamp_reach


class RobotArmIK:

    stall_pos_tol = 0.02
    stall_ori_tol = 0.15
    stall_ticks = 5
    reseed_cooldown = 30
    reseed_min_gain = 0.01
    reseed_w_dist = 0.02
    reseed_dq_max = 0.3
    tick_dq_max = 0.5

    def __init__(self, robot):
        assert robot.has_arm, "arm 없는 rig 로 RobotArmIK 불가"
        self._robot = robot
        self.joint_names = list(robot.arm_joint_names)
        self._seeded = False
        self._stall = 0
        self._cooldown = 0
        self._q_prev = None

    def solve(self, T_canonical: np.ndarray) -> np.ndarray:
        r = self._robot
        T_b = clamp_reach(r.to_base(np.asarray(T_canonical, dtype=float)),
                          r.rig.solver.reach_max)
        solver = r.solver
        if not self._seeded and hasattr(solver, "solve_robust"):
            q = np.asarray(solver.solve_robust(T_b), dtype=float)
            self._seeded = True
        else:
            q = np.asarray(solver.solve(T_b), dtype=float)
            q = self._recover_if_stalled(solver, q, T_b)
        if q.shape[0] != len(self.joint_names):
            raise ValueError(
                f"IK 해({q.shape[0]}) != arm_joint_names({len(self.joint_names)}) — "
                "rig/solver 관절 구성 불일치")
        self._q_prev = q.copy()
        return q

    def _cap_tick_step(self, q, solver):
        if self._q_prev is None or not self.tick_dq_max:
            return q
        step = q - self._q_prev
        n = float(np.linalg.norm(step))
        if n <= self.tick_dq_max:
            return q
        q = self._q_prev + step * (self.tick_dq_max / n)
        if hasattr(solver, "sync_state"):
            solver.sync_state(q)
        return q

    def _recover_if_stalled(self, solver, q, T_b):
        if not (hasattr(solver, "solve_robust") and hasattr(solver, "pose_error")):
            return q
        if self._cooldown > 0:
            self._cooldown -= 1
            return q
        pe, oe = solver.pose_error(q, T_b)
        self._stall = self._stall + 1 \
            if (pe > self.stall_pos_tol or oe > self.stall_ori_tol) else 0
        if self._stall < self.stall_ticks:
            return q
        try:
            try:
                q_g = np.asarray(solver.solve_robust(T_b, q_ref=q,
                                                     w_dist=self.reseed_w_dist), dtype=float)
            except TypeError:
                q_g = np.asarray(solver.solve_robust(T_b), dtype=float)
            if self._score(solver, q_g, T_b) < self._score(solver, q, T_b) - self.reseed_min_gain:
                step = q_g - q
                n = float(np.linalg.norm(step))
                q = q + step * (self.reseed_dq_max / n) if n > self.reseed_dq_max else q_g
        except RuntimeError:
            pass
        finally:
            if hasattr(solver, "sync_state"):
                solver.sync_state(q)
        q = self._cap_tick_step(q, solver)
        self._stall = 0
        self._cooldown = self.reseed_cooldown
        return q

    @staticmethod
    def _score(solver, q, T_b) -> float:
        pe, oe = solver.pose_error(q, T_b)
        return pe + 0.1 * oe

    def reseed(self) -> None:
        self._seeded = False
        self._stall = 0
        self._cooldown = 0
        solver = getattr(self._robot, "solver", None)
        if solver is not None and hasattr(solver, "sync_state") \
                and hasattr(solver, "q_neutral"):
            solver.sync_state(solver.q_neutral)

    def sync_state(self, q_arm) -> None:
        self._robot.sync_state(q_arm)

    def ee_pose(self, q_arm) -> np.ndarray:
        return self._robot.ee_pose(q_arm)
