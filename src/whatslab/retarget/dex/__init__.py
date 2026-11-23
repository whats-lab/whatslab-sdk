import importlib

_LAZY = {
    "DexEngine": ".engine",
    "DexHandConfig": ".engine",
    "hand_config_names": ".engine",
    "resolve_hand_config": ".engine",
    "mano_palm_frame": ".preprocess",
    "mirror_xr_joints": ".preprocess",
    "palm_frame": ".preprocess",
    "xr_to_mano": ".preprocess",
    "xr_wrist_positions": ".preprocess",
}

__all__ = sorted(_LAZY)


def __getattr__(name):
    mod = _LAZY.get(name)
    if mod is None:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
    return getattr(importlib.import_module(mod, __name__), name)
