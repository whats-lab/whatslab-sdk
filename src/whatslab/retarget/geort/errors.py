from __future__ import annotations


def describe(code: str, cause: str, remedy: str = "") -> str:
    return f"[{code}] {cause}" + (f" — {remedy}" if remedy else "")


class GeortError(ValueError):

    def __init__(self, code: str, cause: str, remedy: str = "") -> None:
        super().__init__(describe(code, cause, remedy))
        self.code = code
        self.cause = cause
        self.remedy = remedy


class GeortConfigError(GeortError):
    pass
