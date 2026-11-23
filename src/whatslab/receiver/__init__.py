# glove 는 airglove-client(receiver extra)가 있어야 import 된다 — WebXR 만 쓰는 쪽이 그 의존성
# 없이 쓸 수 있게 하위 패키지를 쓸 때만 불러온다.
import importlib

_EXPORTS = {
    "AirGloveReceiver": ".glove",
    "GloveFrame": ".glove",
    "GloveWrist": ".glove",
    "WebXRControllerReceiver": ".webxr",
    "WebXRHandReceiver": ".webxr",
}

# star import 가 glove 를 끌어와 airglove-client 를 요구하지 않게 __all__ 에서 뺀다.
__all__ = [n for n, m in _EXPORTS.items() if m != ".glove"]


def __getattr__(name):
    module = _EXPORTS.get(name)
    if module is None:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
    return getattr(importlib.import_module(module, __name__), name)
