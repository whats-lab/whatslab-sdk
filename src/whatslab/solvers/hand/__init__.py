# dex(DexPilot)는 pinocchio·dex-retargeting(torch)을 import 한다. GeoRT 는 numpy 만 쓴다.
# 패키지 import 만으로 무거운 의존을 요구하지 않게 지연 import 한다.
_LAZY = {
    "HandRetargetController": ".controller",
    "engine_of": ".controller",
    "XRHandRetargeter": ".dex_retargeter",
    "DexHandConfig": ".dex_retargeter",
    "hand_config_names": ".dex_retargeter",
    "mirror_xr_joints": ".xr_input",
    "xr_to_mano": ".xr_input",
    "GeortHandSolver": ".geort",
}

# star import 가 dex(pinocchio·torch)를 끌어오지 않게 그 이름은 __all__ 에서 뺀다.
__all__ = sorted(n for n, m in _LAZY.items() if m != ".dex_retargeter")


def __getattr__(name):
    mod = _LAZY.get(name)
    if mod is None:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
    import importlib
    return getattr(importlib.import_module(mod, __name__), name)
