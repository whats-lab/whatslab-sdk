# GloveModel 은 airglove-client(receiver extra)가 있어야 import 된다 — QuestModel(WebXR)만
# 쓰는 쪽이 그 의존성 없이 쓸 수 있게 쓸 때만 불러온다.
import importlib

from .hand import HandModel
from .quest import QuestModel

_LAZY = {"GloveModel": ".glove"}

# star import 가 지연 이름을 끌어와 airglove-client 를 요구하지 않게 __all__ 에서 뺀다.
__all__ = ["QuestModel", "HandModel"]


def __getattr__(name):
    module = _LAZY.get(name)
    if module is None:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
    return getattr(importlib.import_module(module, __name__), name)
