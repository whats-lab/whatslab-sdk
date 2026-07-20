"""로봇 정의 계층 — rig config 로드 + RobotModel (docs/DESIGN_robot_rig.md).

    from whatslab.robot import RobotModel, load_rig
    model = RobotModel.from_yaml("rigs/nero_orca_right.yaml")
"""
from whatslab.paths import configs_root

from .config import (
    CalibrationCfg,
    Origin,
    RigConfig,
    RobotSpec,
    SolverCfg,
    load_rig,
    load_robot,
    save_calibration,
    save_reach_max,
)
from .model import RobotModel

__all__ = [
    "RobotModel", "RigConfig", "RobotSpec", "Origin", "SolverCfg",
    "CalibrationCfg", "load_rig", "load_robot", "save_calibration",
    "save_reach_max", "configs_root",
]
