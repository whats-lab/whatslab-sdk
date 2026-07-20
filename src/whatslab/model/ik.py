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

    def __init__(self, robot):
        assert robot.has_arm, "arm 없는 rig 로 RobotArmIK 불가"
        self._robot = robot
        self.joint_names = list(robot.arm_joint_names)
        self._seeded = False       # 첫 유효 타깃에서 solve_robust 로 좋은 basin 을 한 번 잡는다

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
        if q.shape[0] != len(self.joint_names):            # 관절 수 불일치 → 조용한 절단 방지
            raise ValueError(
                f"IK 해({q.shape[0]}) != arm_joint_names({len(self.joint_names)}) — "
                "rig/solver 관절 구성 불일치")
        return q

    def sync_state(self, q_arm) -> None:
        self._robot.sync_state(q_arm)

    def ee_pose(self, q_arm) -> np.ndarray:
        return self._robot.ee_pose(q_arm)
