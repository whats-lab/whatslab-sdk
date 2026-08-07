_LAZY = {
    "ArmIK": ".arm_ik",
    "DiffArmIK": ".arm_ik",
    "xyzrpy_to_mat": ".arm_ik",
    "xyzquat_to_mat": ".arm_ik",
    "backend_cls": ".builders",
    "HandRetargeter": ".hand",
    "HandRetargetController": ".hand",
    "CONFIG_REGISTRY": ".hand",
}

__all__ = list(_LAZY)


def __getattr__(name):
    mod = _LAZY.get(name)
    if mod is None:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
    from importlib import import_module
    return getattr(import_module(mod, __name__), name)


def __dir__():
    return sorted(__all__)
