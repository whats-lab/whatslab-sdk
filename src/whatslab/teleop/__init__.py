from .base import TeleopModel
from .calibration import ArmCalibration
from .models import HandModel, QuestModel
from .side import SideModel

# GloveModel 은 airglove-client 가 있어야 하므로 models 처럼 쓸 때만 불러온다(star import 제외).
__all__ = ["TeleopModel", "SideModel", "QuestModel", "HandModel", "ArmCalibration"]


def __getattr__(name):
    if name == "GloveModel":
        from .models.glove import GloveModel

        return GloveModel
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
