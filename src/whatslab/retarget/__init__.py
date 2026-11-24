import importlib

_LAZY = {
    "HandRetargeter": ".retargeter",
    "Engine": ".retargeter",
    "engine_of": ".retargeter",
    "masked_keypoints": ".retargeter",
    "DexEngine": ".dex",
    "DexHandConfig": ".dex",
    "hand_config_names": ".dex",
    "mirror_xr_joints": ".dex",
    "xr_to_mano": ".dex",
    "GeortEngine": ".geort",
}

__all__ = sorted(n for n, m in _LAZY.items() if m != ".dex")


def __getattr__(name):
    mod = _LAZY.get(name)
    if mod is None:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
    return getattr(importlib.import_module(mod, __name__), name)
