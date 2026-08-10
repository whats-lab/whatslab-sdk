import os
from abc import ABC
from dataclasses import dataclass
from typing import ClassVar, Dict, List, Optional, Union

import numpy as np
import pinocchio as pin

from whatslab.core.types import JOINT_INDEX
from whatslab.paths import models_root as _models_root

FINGER_ORDER = ('thumb', 'index', 'middle', 'ring', 'pinky')


def _resolve_human(human: List) -> List[int]:
    out = []
    for h in human:
        if isinstance(h, str):
            if h not in JOINT_INDEX:
                raise KeyError(f"알 수 없는 골격 관절명: {h!r}. 유효: {list(JOINT_INDEX)}")
            out.append(JOINT_INDEX[h])
        else:
            out.append(int(h))
    return out


def _default_models_root() -> str:
    return _models_root()


@dataclass
class FingerChain:
    links: List[str]
    human: List


class HandConfig(ABC):

    _MODEL_SUBDIR:         ClassVar[str]                               = ''
    _HUMAN_CHAIN:          ClassVar[Dict[str, List[str]]]              = {}
    _COORD_TRANSFORM:      ClassVar[np.ndarray]                        = np.eye(3, dtype=np.float32)
    _SCALE_FACTOR:         ClassVar[Union[float, List[float]]]         = 1.0
    _URDF_FILENAME:        ClassVar[str]                               = 'urdf/{hand_type}.urdf'
    _FIXED_JOINTS:         ClassVar[Dict[str, str]]                    = {}
    _TARGET_JOINT_NAMES:   ClassVar[Union[List[str], Dict[str, List[str]]]] = []
    _KP_SHAPE_WEIGHT:      ClassVar[float]                             = 1.0
    _KP_COLD_SHAPE:        ClassVar[bool]                              = False

    def __init__(self, urdf_root=None):
        root = urdf_root or _default_models_root()
        self._models_root = root
        self._urdf_dir = os.path.join(root, self._MODEL_SUBDIR)
        self._cache: Dict[str, tuple] = {}

    def _get_urdf_path(self, hand_type: str) -> str:
        unified = os.path.join(self._urdf_dir, 'urdf', f'{hand_type}.urdf')
        legacy  = os.path.join(self._urdf_dir, self._URDF_FILENAME.format(hand_type=hand_type))
        for path in (unified, legacy):
            if os.path.exists(path):
                return path
        return unified

    def _derive(self, hand_type: str) -> tuple:
        if hand_type in self._cache:
            return self._cache[hand_type]
        path = self._get_urdf_path(hand_type)
        if not os.path.exists(path):
            raise FileNotFoundError(f"{type(self).__name__}: URDF 없음 {path}")
        m = pin.buildModelFromUrdf(path)

        def body(joint: int) -> Optional[str]:
            for fr in m.frames:
                if (fr.type == pin.FrameType.BODY and int(fr.parent) == joint
                        and '_sensor_' not in fr.name):
                    return fr.name
            return None

        tips, chains = {}, {}
        for f in FINGER_ORDER:
            if f not in self._HUMAN_CHAIN:
                continue
            tip = f'{hand_type}_sensor_{f}_distal'
            if not m.existFrame(tip, pin.FrameType.BODY):
                raise ValueError(
                    f"{type(self).__name__}/{hand_type}: 센서 프레임 {tip} 이 URDF 에"
                    f" 없다 — 손가락 사슬은 센서 프레임에서 유도한다 ({path})")
            jid = int(m.frames[m.getFrameId(tip, pin.FrameType.BODY)].parent)
            tips[f] = tip
            chains[f] = [int(j) for j in m.supports[jid] if j > 0]
        if not chains:
            raise ValueError(f"{type(self).__name__}: _HUMAN_CHAIN 이 비어 있다")

        shared = set.intersection(*[set(v) for v in chains.values()])
        palm = body(max(shared) if shared else 0)
        if palm is None:
            raise ValueError(f"{type(self).__name__}/{hand_type}: 팜 링크를 못 찾았다")

        fingers = []
        for f in chains:
            links = [n for n in (body(j) for j in chains[f] if j not in shared)
                     if n is not None]
            robot = [palm] + links + [tips[f]]
            human = list(self._HUMAN_CHAIN[f])
            if len(human) != len(robot):
                raise ValueError(
                    f"{type(self).__name__}/{hand_type}/{f}: _HUMAN_CHAIN 길이"
                    f" {len(human)} != URDF 사슬 {len(robot)} — 사슬 {robot}")
            fingers.append(FingerChain(robot, _resolve_human(human)))
        self._cache[hand_type] = (fingers, palm)
        return self._cache[hand_type]

    def _get_fingers(self, hand_type: str) -> List[FingerChain]:
        return self._derive(hand_type)[0]

    def get_two_stage_config(self, hand_type: str):
        urdf_path = self._get_urdf_path(hand_type)
        fingers   = self._get_fingers(hand_type)

        stage1 = {
            'type': 'vector',
            'urdf_path': urdf_path,
            'target_origin_link_names':  [l for f in fingers for l in f.links[:-1]],
            'target_task_link_names':    [l for f in fingers for l in f.links[1:]],
            'target_link_human_indices': [
                [h for f in fingers for h in f.human[:-1]],
                [h for f in fingers for h in f.human[1:]],
            ],
            'low_pass_alpha': -1.0,
        }
        stage2 = {
            'type': 'position',
            'urdf_path': urdf_path,
            'target_link_names':         [f.links[-1] for f in fingers],
            'target_link_human_indices': [f.human[-1] for f in fingers],
            'low_pass_alpha': -1.0,
        }
        target_joints = self._TARGET_JOINT_NAMES
        if isinstance(target_joints, dict):
            target_joints = target_joints.get(hand_type, [])
        if target_joints:
            stage1['target_joint_names'] = target_joints
            stage2['target_joint_names'] = target_joints
        return stage1, stage2

    def get_coord_transform(self, _hand_type: str) -> np.ndarray:
        return self._COORD_TRANSFORM

    def get_scale_factor(self) -> Union[float, List[float]]:
        return self._SCALE_FACTOR

    def get_wrist_link_name(self, hand_type: str) -> str:
        return self._derive(hand_type)[1]

    def get_fixed_joint_names(self, hand_type: str) -> List[str]:
        joint = self._FIXED_JOINTS.get(hand_type, '')
        return [joint] if joint else []

    def get_tf_coord_transform(self, hand_type: str) -> np.ndarray:
        return self.get_coord_transform(hand_type)
