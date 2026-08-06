import os

_PKG_DIR = os.path.dirname(os.path.abspath(__file__))


def models_root() -> str:
    root = os.environ.get("WHATSLAB_MODELS_ROOT")
    if root:
        return root
    import dexhand_description          # lazy — 실제 자산 해석 시에만 필요
    return dexhand_description.get_share()


def configs_root() -> str:
    return os.environ.get("WHATSLAB_CONFIGS_ROOT") or os.path.join(_PKG_DIR, "configs")
