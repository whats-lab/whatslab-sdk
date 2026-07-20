"""손 리타게팅 — dex_retargeting 2단계 IK + pinocchio spherical FK.

atlas_hand_core 이식본. URDF/모델 경로는 생성 시 urdf_root 로 주입한다:

    from whatslab.teleop.hand import HandRetargeter
    r = HandRetargeter('right', 'allegro_hand', urdf_root='/path/to/models')

`HandRetargeter` import 는 dex_retargeting/pinocchio 를 요구한다(`whatslab-sdk[hand]`).
설정 레지스트리만 필요하면 `from whatslab.teleop.hand.hand_configs import CONFIG_REGISTRY`.
"""
from .controller import HandRetargetController
from .hand_configs import CONFIG_REGISTRY
from .retargeter import HandRetargeter

__all__ = ["HandRetargeter", "HandRetargetController", "CONFIG_REGISTRY"]
