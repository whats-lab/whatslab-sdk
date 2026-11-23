from __future__ import annotations

from typing import List, Protocol, Sequence, runtime_checkable

import numpy as np

from .types import HandCommand, InputSample


@runtime_checkable
class Receiver(Protocol):
    """입력 수신기.

    start()·stop() 은 여러 번 불러도 한 번만 동작한다(TeleopModel 이 기댄다). 둘은 스레드 안전을
    약속하지 않는다 — 한 스레드(보통 메인)에서 부른다. get() 은 다른 스레드에서 불러도 된다.
    """

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
    @property
    def nq(self) -> int: ...
    def solve(self, target_pose: np.ndarray) -> np.ndarray: ...
    def fk(self, q: np.ndarray) -> np.ndarray: ...
    def active_joint_names(self) -> List[str]: ...


@runtime_checkable
class GlobalArmSolver(ArmSolver, Protocol):
    def solve_robust(self, target_pose: np.ndarray, **kw) -> np.ndarray: ...
    def pose_error(self, q: np.ndarray, target_pose: np.ndarray) -> tuple: ...
    def sync_state(self, q_current: Sequence[float]) -> None: ...
    @property
    def q_neutral(self) -> np.ndarray: ...
