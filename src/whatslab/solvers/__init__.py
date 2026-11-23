# 손 dex(dex-retargeting, torch)는 무겁다 — 하위 패키지를 쓸 때만
# 불러오도록 지연 import 한다. (`whatslab.solvers.hand.geort` 만 쓰는 쪽은 numpy 만 있으면
# 된다.)
_HAND = (
    "HandRetargetController",
    "engine_of",
    "XRHandRetargeter",
    "DexHandConfig",
    "GeortHandSolver",
    "hand_config_names",
    "xr_to_mano",
    "mirror_xr_joints",
)

# star import 가 dex 를 끌어오지 않게 numpy 만 쓰는 이름만 __all__ 에 둔다.
__all__ = [
    "HandRetargetController", "engine_of", "GeortHandSolver", "xr_to_mano", "mirror_xr_joints",
]


def __getattr__(name):
    if name in _HAND:
        from . import hand

        return getattr(hand, name)
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
