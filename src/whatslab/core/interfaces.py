from __future__ import annotations

from typing import List, Protocol, runtime_checkable

from .types import HandCommand, InputSample


@runtime_checkable
class Receiver(Protocol):

    def start(self) -> None: ...
    def stop(self) -> None: ...
    def get(self, side: str) -> InputSample: ...


@runtime_checkable
class Retargeter(Protocol):

    @property
    def joint_names(self) -> List[str]: ...
    def compute(self, sample: InputSample) -> HandCommand: ...
