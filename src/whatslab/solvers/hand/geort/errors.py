"""GeoRT 오류 — 기계용 code 와 사람용 cause·remedy 를 담는다."""
from __future__ import annotations


class GeortError(ValueError):
    """GeoRT 소프트 프롬프트 리타게팅 오류. `code` 는 기계용, `cause`·`remedy` 는 사람용."""

    def __init__(self, code: str, cause: str, remedy: str = "") -> None:
        super().__init__(f"[{code}] {cause}" + (f" — {remedy}" if remedy else ""))
        self.code = code
        self.cause = cause
        self.remedy = remedy


class GeortConfigError(GeortError):
    """설정 파일·모델 파일이 없거나 잘못됐다."""


class GeortInputError(GeortError):
    """이번 프레임의 손 입력으로는 풀 수 없다 (호출자는 직전 값을 유지하면 된다)."""
