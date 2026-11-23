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

__all__ = sorted(n for n, m in _LAZY.items() if m != ".dex_retargeter")


def __getattr__(name):
    mod = _LAZY.get(name)
    if mod is None:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
    import importlib
    return getattr(importlib.import_module(mod, __name__), name)
