"""패키지 자산 경로 해석 — configs(whatslab 내장), models(dexhand_description 패키지).

configs(rig/robot yaml)는 whatslab 고유라 패키지에 동봉된다. models(URDF/메쉬)는 별도
설치 패키지 `dexhand_description`(단일 소스)이 제공한다. 환경변수로 override 가능.

dexhand_description 은 models_root() 안에서 **lazy import** 한다 — paths(및 이를
쓰는 viz/robot) 를 import 하는 것만으로 dexhand 자산 패키지 설치를 강제하지 않기
위함(예: viz 순수 로직, WHATSLAB_MODELS_ROOT 로 자산을 직접 준 경우).
"""
import os

_PKG_DIR = os.path.dirname(os.path.abspath(__file__))


def models_root() -> str:
    """URDF/메쉬 루트. WHATSLAB_MODELS_ROOT > dexhand_description 패키지 share."""
    root = os.environ.get("WHATSLAB_MODELS_ROOT")
    if root:
        return root
    import dexhand_description          # lazy — 실제 자산 해석 시에만 필요
    return dexhand_description.get_share()


def configs_root() -> str:
    """robot/rig config 루트. WHATSLAB_CONFIGS_ROOT > 동봉 whatslab/configs."""
    return os.environ.get("WHATSLAB_CONFIGS_ROOT") or os.path.join(_PKG_DIR, "configs")
