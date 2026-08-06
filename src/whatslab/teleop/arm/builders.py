from __future__ import annotations

from .arm_ik import ArmIK, DiffArmIK


def backend_cls(backend: str):
    if backend == "dls":
        return ArmIK
    if backend == "diff":
        return DiffArmIK
    raise ValueError(f"unknown IK backend {backend!r} (dls|diff)")


# 하위 명칭 호환 (RobotModel 등 내부 사용)
_backend_cls = backend_cls
