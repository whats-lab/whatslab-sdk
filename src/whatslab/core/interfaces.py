"""레이어 인터페이스 (Protocol). 각 인터페이스 = 한 가지 책임(SRP).

구조적 타이핑(Protocol)이라 기존 클래스가 메서드 시그니처만 맞으면 자동으로 부합한다.
조립 측(ROS2 노드·standalone 러너)은 이 인터페이스에만 의존하고 구체 구현을 모른다.

의존성 규칙: receiver / hand / arm 구현체는 서로를 import 하지 않고, 오직
core(types·interfaces)만 공유한다. 이들을 엮는 것은 소비자의 몫이다.

명명: 저수준 엔진(예: HandRetargeter 구체 클래스)과 구분하기 위해, 파이프라인
계약은 역할 이름(HandController/ArmSolver/Receiver)을 쓴다.
"""
from __future__ import annotations

from typing import List, Protocol, runtime_checkable

import numpy as np

from .types import HandCommand, InputSample


@runtime_checkable
class Receiver(Protocol):
    """입력을 어디서 받나(Quest/AirGlove). side 별 최신 샘플 제공.

    텔레옵과 무관하게 위치추적 등 단독 용도로도 쓸 수 있다.
    """

    def start(self) -> None: ...
    def stop(self) -> None: ...
    def get(self, side: str) -> InputSample: ...


@runtime_checkable
class HandController(Protocol):
    """손 입력(InputSample) → 손 명령. 다관절 리타게팅 / 그리퍼 공통."""

    @property
    def joint_names(self) -> List[str]: ...
    def compute(self, sample: InputSample) -> HandCommand: ...


@runtime_checkable
class ArmSolver(Protocol):
    """EE 목표 pose(4x4) → 팔 관절각."""

    @property
    def nq(self) -> int: ...
    def solve(self, target_pose: np.ndarray) -> np.ndarray: ...
    def fk(self, q: np.ndarray) -> np.ndarray: ...
    def active_joint_names(self) -> List[str]: ...
