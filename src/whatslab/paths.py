import os

_PKG_DIR = os.path.dirname(os.path.abspath(__file__))


def models_root() -> str:
    root = os.environ.get("WHATSLAB_MODELS_ROOT")
    if root:
        return root
    import dexhand_description
    return dexhand_description.get_share()


def mesh_urdf(urdf: str) -> str:
    if os.environ.get("WHATSLAB_MODELS_ROOT"):
        return urdf
    import dexhand_description
    rel = os.path.relpath(os.path.abspath(urdf), dexhand_description.get_share())
    if rel.startswith(os.pardir):
        return urdf
    hand, _, inner = rel.partition(os.sep)
    folder = dexhand_description.hand_dir(hand)
    return urdf if folder is None else os.path.join(folder, inner)


def configs_root() -> str:
    return os.environ.get("WHATSLAB_CONFIGS_ROOT") or os.path.join(_PKG_DIR, "configs")
