import importlib

_EXPORTS = {
    "AirGloveReceiver": ".glove",
    "GloveFrame": ".glove",
    "WebXRHandReceiver": ".webxr",
}

__all__ = [n for n, m in _EXPORTS.items() if m != ".glove"]


def __getattr__(name):
    module = _EXPORTS.get(name)
    if module is None:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
    return getattr(importlib.import_module(module, __name__), name)
