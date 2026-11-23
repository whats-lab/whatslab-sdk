from __future__ import annotations

from typing import Protocol, runtime_checkable

from .types import InputSample


@runtime_checkable
class Receiver(Protocol):

    def start(self) -> None: ...
    def stop(self) -> None: ...
    def get(self, side: str) -> InputSample: ...
