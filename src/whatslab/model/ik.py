from __future__ import annotations

import numpy as np


class RobotArmIK:

    # ── 전역 탐색(basin 선택) 정책 ───────────────────────────────────────────
    # 프레임 추종은 rig 가 고른 백엔드의 solve() 가 한다(연속성은 그쪽이 보장).
    # 전역 탐색(solve_robust)은 후보 하나가 full-convergence DLS(수 ms)라 매 프레임
    # 돌릴 수 없다 → **basin 을 새로 골라야 할 때만** 부른다:
    #   ① 첫 유효 타깃  ② reseed()(캘리브 등 목표 불연속)  ③ 확실한 스톨
    stall_pos_tol = 0.02     # [m]   이 이상 남으면 스톨 후보
    stall_ori_tol = 0.15     # [rad] 이 이상 남으면 스톨 후보 (~8.6°)
    stall_ticks = 5          # 연속 이만큼 스톨이어야 발동 (일시 lag 오탐 방지)
    reseed_cooldown = 30     # 발동 후 이만큼 틱은 재발동 금지
    reseed_min_gain = 0.01   # [m] score(pos+0.1·ori)가 이만큼 개선될 때만 채택
    reseed_w_dist = 0.02     # [m/rad] 탈출 해 선택 시 관절거리 가중(가까운 분기 우선)
    reseed_dq_max = 0.3      # [rad] 채택 시 틱당 이동 상한(여러 틱에 걸쳐 이행)
    tick_dq_max = 0.5        # [rad] 틱당 **총** 이동 상한 (백엔드 스텝 + 탈출 이행분 합)

    def __init__(self, robot):
        assert robot.has_arm, "arm 없는 rig 로 RobotArmIK 불가"
        self._robot = robot
        self.joint_names = list(robot.arm_joint_names)
        self._seeded = False       # 첫 유효 타깃에서 전역 탐색으로 basin 을 한 번 잡는다
        self._stall = 0            # 연속 스톨 틱 카운터
        self._cooldown = 0         # 재발동 쿨다운 잔여 틱
        self._q_prev = None        # 직전 틱 해 — 틱당 총 이동 상한 계산용

    def solve(self, T_canonical: np.ndarray) -> np.ndarray:
        r = self._robot
        from whatslab.robot.model import clamp_reach   # lazy: 이 모듈을 numpy 전용으로 유지
        T_b = clamp_reach(r.to_base(np.asarray(T_canonical, dtype=float)),
                          r.rig.solver.reach_max)
        solver = r.solver
        if not self._seeded and hasattr(solver, "solve_robust"):
            q = np.asarray(solver.solve_robust(T_b), dtype=float)   # cold-start: 균등 전역
            self._seeded = True
        else:
            q = np.asarray(solver.solve(T_b), dtype=float)          # 추종(연속)
            q = self._recover_if_stalled(solver, q, T_b)
        if q.shape[0] != len(self.joint_names):     # 관절 수 불일치 → 조용한 절단 방지
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
            solver.sync_state(q)                    # warm-start 도 잘라낸 해로
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
        # solve_robust 는 후보를 평가하며 warm-start 를 덮어쓴다. 성공/실패/예외 어느
        # 경로로 나가든 이 틱의 해로 되돌려야 다음 틱이 랜덤 시드에서 이어지지 않는다.
        try:
            try:
                q_g = np.asarray(solver.solve_robust(T_b, q_ref=q,
                                                     w_dist=self.reseed_w_dist), dtype=float)
            except TypeError:                # 거리 가중 미지원 커스텀 솔버
                q_g = np.asarray(solver.solve_robust(T_b), dtype=float)
            if self._score(solver, q_g, T_b) < self._score(solver, q, T_b) - self.reseed_min_gain:
                step = q_g - q               # 더 나은 분기 → 상한만큼만 이행
                n = float(np.linalg.norm(step))
                q = q + step * (self.reseed_dq_max / n) if n > self.reseed_dq_max else q_g
        except RuntimeError:                 # 전역 탐색 실패 → 연속 추종 유지
            pass
        finally:
            if hasattr(solver, "sync_state"):
                solver.sync_state(q)
        q = self._cap_tick_step(q, solver)   # 백엔드 스텝 + 이행분 합을 상한으로 묶는다
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
            solver.sync_state(solver.q_neutral)      # warm-start(이전 맥락) → 중립

    def sync_state(self, q_arm) -> None:
        self._robot.sync_state(q_arm)

    def ee_pose(self, q_arm) -> np.ndarray:
        return self._robot.ee_pose(q_arm)
