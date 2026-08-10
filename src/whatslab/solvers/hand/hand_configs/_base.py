import os
from abc import ABC
from dataclasses import dataclass
from typing import ClassVar, Dict, List, Union

import numpy as np

from whatslab.core.types import JOINT_INDEX
from whatslab.paths import models_root as _models_root


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
    _FINGERS:              ClassVar[Dict[str, List[FingerChain]]]      = {'left': [], 'right': []}
    _WRIST_LINK:           ClassVar[Dict[str, str]]                    = {'left': 'world', 'right': 'world'}
    _COORD_TRANSFORM:      ClassVar[np.ndarray]                        = np.eye(3, dtype=np.float32)
    _SCALE_FACTOR:         ClassVar[Union[float, List[float]]]         = 1.0
    _URDF_FILENAME:        ClassVar[str]                               = 'urdf/{hand_type}.urdf'
    _SIDE_MAP:             ClassVar[Dict[str, str]]                    = {'left': 'left', 'right': 'right'}
    _FIXED_JOINTS:         ClassVar[Dict[str, str]]                    = {}
    _RVIZ_FILENAME:        ClassVar[Dict[str, str]]                    = {}
    _TARGET_JOINT_NAMES:   ClassVar[Union[List[str], Dict[str, List[str]]]] = []
    _KP_SHAPE_WEIGHT:      ClassVar[float]                             = 1.0
    _KP_COLD_SHAPE:        ClassVar[bool]                              = False
    _LINK_FALLBACK:        ClassVar[Dict[str, str]]                    = {}

    def __init__(self, urdf_root=None):
        root = urdf_root or _default_models_root()
        self._models_root = root
        self._urdf_dir = os.path.join(root, self._MODEL_SUBDIR)

    def _get_urdf_path(self, hand_type: str) -> str:
        unified = os.path.join(self._urdf_dir, 'urdf', f'{hand_type}.urdf')
        legacy  = os.path.join(self._urdf_dir, self._URDF_FILENAME.format(hand_type=hand_type))
        for path in (unified, legacy):
            if os.path.exists(path):
                return path
        return unified

    def _urdf_links(self, hand_type: str) -> set:
        path = self._get_urdf_path(hand_type)
        if not os.path.exists(path):
            return set()
        import xml.etree.ElementTree as ET
        return {l.get('name') for l in ET.parse(path).getroot().iter('link')}

    def _get_fingers(self, hand_type: str) -> List[FingerChain]:
        fmt = {'side': self._SIDE_MAP[hand_type], 'wrist': self._WRIST_LINK[hand_type],
               'hand': hand_type}
        present = self._urdf_links(hand_type)

        def resolve(name: str) -> str:
            first = name.format(**fmt)
            if not present or first in present:
                return first
            alt = self._LINK_FALLBACK.get(name)
            if alt is not None:
                alt = alt.format(**fmt)
                if alt in present:
                    return alt
            return first

        return [
            FingerChain([resolve(l) for l in f.links], _resolve_human(f.human))
            for f in self._FINGERS[hand_type]
        ]

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
        return self._WRIST_LINK[hand_type]

    def get_fixed_joint_names(self, hand_type: str) -> List[str]:
        joint = self._FIXED_JOINTS.get(hand_type, '')
        return [joint] if joint else []

    def get_tf_coord_transform(self, hand_type: str) -> np.ndarray:
        return self.get_coord_transform(hand_type)
