from __future__ import annotations

from typing import List, Protocol, Sequence, runtime_checkable

import numpy as np

from .types import HandCommand, InputSample


@runtime_checkable
class Receiver(Protocol):

    def start(self) -> None: ...
    def stop(self) -> None: ...
    def get(self, side: str) -> InputSample: ...


@runtime_checkable
class HandController(Protocol):

    @property
    def joint_names(self) -> List[str]: ...
    def compute(self, sample: InputSample) -> HandCommand: ...


@runtime_checkable
class ArmSolver(Protocol):
    # 최소 계약 — 프레임 추종만 하는 커스텀 솔버는 이것만 만족하면 된다.
    @property
    def nq(self) -> int: ...
    def solve(self, target_pose: np.ndarray) -> np.ndarray: ...
    def fk(self, q: np.ndarray) -> np.ndarray: ...
    def active_joint_names(self) -> List[str]: ...


@runtime_checkable
class GlobalArmSolver(ArmSolver, Protocol):
    # RobotArmIK 가 basin 재선택(첫 타깃 / reseed / 스톨 탈출)에 추가로 요구하는 것.
    # 이걸 만족하지 않으면 그 경로들이 조용히 비활성화되므로, 최소 계약과 분리해 둔다.
    def solve_robust(self, target_pose: np.ndarray, **kw) -> np.ndarray: ...
    def pose_error(self, q: np.ndarray, target_pose: np.ndarray) -> tuple: ...
    def sync_state(self, q_current: Sequence[float]) -> None: ...
    @property
    def q_neutral(self) -> np.ndarray: ...
