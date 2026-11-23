from __future__ import annotations

from ..xr_input import HandInputError


class GeortError(ValueError):

    def __init__(self, code: str, cause: str, remedy: str = "") -> None:
        super().__init__(f"[{code}] {cause}" + (f" — {remedy}" if remedy else ""))
        self.code = code
        self.cause = cause
        self.remedy = remedy


class GeortConfigError(GeortError):
    pass


class GeortInputError(GeortError, HandInputError):
    pass
