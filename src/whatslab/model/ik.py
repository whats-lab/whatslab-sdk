"""팔 IK 컴포넌트 — 순수 solve(정준 EE 목표 → q_arm).

TeleopModel 에 주입하는 IK 모델의 기본 구현. 계약(덕타이핑)은 단 둘:
  · `solve(T_canonical) -> q_arm`  (4x4 정준 목표 → 관절각 배열)
  · `joint_names`                   (그 q 의 관절명 순서)
유저가 자기 IK 모델을 넣으려면 이 둘만 만족하면 된다.

`RobotArmIK` 는 reach **스케일을 하지 않는다** — 사람→로봇 도달반경 매핑은
전처리(ArmCalibration)의 몫이고, 여기 들어오는 T 는 이미 정준·스케일 완료
상태다. IK 는 정준→베이스 변환 + reach_max 클램프(로봇 안전망) + 로봇 솔버만.
"""
from __future__ import annotations

import numpy as np


class RobotArmIK:
    """RobotModel 기반 기본 IK 컴포넌트 (정준 T → q_arm, reach 스케일 없음)."""

    # 스톨 자동 reseed — 연속(local) solve 가 다른 분기에 갇혀 EE 오차를 못 닫을 때,
    # 전역 solve_robust 로 탈출시켜 "무조건 수렴"에 가깝게 만든다.
    stall_pos_tol = 0.02     # [m]   이 이상 남으면 스톨 후보
    stall_ori_tol = 0.15     # [rad] 이 이상 남으면 스톨 후보 (~8.6°)
    stall_ticks = 3          # 연속 이만큼 스톨이어야 발동 (일시적 lag 오탐 방지)
    reseed_cooldown = 10     # 발동 후 이만큼 틱은 재발동 금지 (도달불가서 매틱 낭비 방지)

    def __init__(self, robot):
        assert robot.has_arm, "arm 없는 rig 로 RobotArmIK 불가"
        self._robot = robot
        self.joint_names = list(robot.arm_joint_names)
        self._seeded = False       # 첫 유효 타깃에서 solve_robust 로 좋은 basin 을 한 번 잡는다
        self._stall = 0            # 연속 스톨 틱 카운터
        self._cooldown = 0         # 재발동 쿨다운 잔여 틱

    def solve(self, T_canonical: np.ndarray) -> np.ndarray:
        r = self._robot
        T_b = r.to_base(np.asarray(T_canonical, dtype=float))
        reach_max = r.rig.solver.reach_max
        if reach_max:                                  # 베이스 프레임 도달 클램프(안전망)
            n = float(np.linalg.norm(T_b[:3, 3]))
            if n > reach_max:
                T_b[:3, 3] *= reach_max / n
        solver = r.solver
        # cold-start basin 을 시작점 운(첫 목표)에 맡기지 않도록, 첫 유효 타깃에서만
        # 다중 재시작(solve_robust)으로 실제 오차 최소해를 잡고 warm-start 로 넘긴다.
        # 이후 프레임은 연속(solve) 추종. (solve_robust 없는 커스텀 솔버면 그냥 solve.)
        if not self._seeded and hasattr(solver, "solve_robust"):
            q = np.asarray(solver.solve_robust(T_b), dtype=float)
            self._seeded = True
        else:
            q = np.asarray(solver.solve(T_b), dtype=float)
            q = self._recover_if_stalled(solver, q, T_b)   # 스톨 → 전역 탈출
        if q.shape[0] != len(self.joint_names):            # 관절 수 불일치 → 조용한 절단 방지
            raise ValueError(
                f"IK 해({q.shape[0]}) != arm_joint_names({len(self.joint_names)}) — "
                "rig/solver 관절 구성 불일치")
        return q

    def _recover_if_stalled(self, solver, q, T_b):
        """연속 solve 가 스톨(다른 분기 필요)이면 solve_robust 로 전역 탈출.
        stall_ticks 연속 스톨에만 발동(일시 lag 무시), 발동 후 reseed_cooldown 틱 대기."""
        if not (hasattr(solver, "solve_robust") and hasattr(solver, "pose_error")):
            return q
        if self._cooldown > 0:
            self._cooldown -= 1
            return q
        pe, oe = solver.pose_error(q, T_b)
        if pe > self.stall_pos_tol or oe > self.stall_ori_tol:
            self._stall += 1
        else:
            self._stall = 0
        if self._stall >= self.stall_ticks:
            q = np.asarray(solver.solve_robust(T_b), dtype=float)
            self._stall = 0
            self._cooldown = self.reseed_cooldown
        return q

    def reseed(self) -> None:
        """캘리브 등으로 목표 프레임이 불연속으로 바뀔 때 호출 — IK 맥락을 초기화한다.
        ① `_seeded=False` → 다음 solve 가 solve_robust(다중 재시작)로 basin 재탐색.
        ② solver warm-start(history/init)를 **중립으로 리셋** → 이전 포즈(맥락)를
           solve_robust 후보에서 아예 배제해, 이전 basin(elbow/어깨 branch)에 갇히는
           것을 확실히 방지. (캘리 시점의 의도된 리셋이라 포즈 점프는 허용.)"""
        self._seeded = False
        self._stall = 0
        self._cooldown = 0
        solver = getattr(self._robot, "solver", None)
        if solver is not None and hasattr(solver, "sync_state") \
                and hasattr(solver, "_q_neutral"):
            solver.sync_state(solver._q_neutral)     # warm-start(이전 맥락) → 중립

    def sync_state(self, q_arm) -> None:
        self._robot.sync_state(q_arm)

    def ee_pose(self, q_arm) -> np.ndarray:
        return self._robot.ee_pose(q_arm)
