"""IK 백엔드 선택 — 로봇 불가지론.

로봇별 조립(URDF/TCP/attach/lock)은 rig config + RobotModel 소관
(whatslab.robot). 여기는 "어떤 수치 방법으로 풀 것인가"만 담당한다.

    from whatslab.teleop.arm import backend_cls
    solver = backend_cls("diff")(urdf_path=..., ...)   # 보통은 RobotModel 이 호출
"""
from __future__ import annotations

from .arm_ik import ArmIK, DiffArmIK


def backend_cls(backend: str):
    """IK 백엔드 클래스 선택.

    dls  : 매 프레임 수렴까지 반복 (cold-start 정밀해, solve_robust 용)
    diff : 미분 IK — 틱당 소수 스텝 + 목표 rate-limit + Sugihara 감쇠
           + null-space 자세 (텔레옵 권장)
    """
    if backend == "dls":
        return ArmIK
    if backend == "diff":
        return DiffArmIK
    raise ValueError(f"unknown IK backend {backend!r} (dls|diff)")


# 하위 명칭 호환 (RobotModel 등 내부 사용)
_backend_cls = backend_cls
