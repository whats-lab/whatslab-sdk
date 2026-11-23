_LAZY = {
    "HandRetargeter": ".retargeter",
    "engine_of": ".retargeter",
    "DexEngine": ".dex",
    "DexHandConfig": ".dex",
    "hand_config_names": ".dex",
    "mirror_xr_joints": ".xr_input",
    "xr_to_mano": ".xr_input",
    "GeortEngine": ".geort",
}

__all__ = sorted(n for n, m in _LAZY.items() if m != ".dex")


def __getattr__(name):
    mod = _LAZY.get(name)
    if mod is None:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
    import importlib
    return getattr(importlib.import_module(mod, __name__), name)
