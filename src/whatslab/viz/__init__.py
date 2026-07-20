"""시각화 (viser 기반, ROS 비의존).

`whatslab-sdk[viz]` (viser, trimesh) 필요. viser 서버는 포트당 하나를 공유(get_server)해
팔/손/사람손/캘리브가 한 화면에 공존한다.

    from whatslab.viz import RobotArmViz, HandSkeletonViz
"""
from .scene import (
    HandSkeletonViz,
    RobotArmViz,
    RobotHandViz,
    URDFScene,
    get_server,
)

__all__ = ["URDFScene", "get_server", "HandSkeletonViz", "RobotHandViz",
           "RobotArmViz"]
