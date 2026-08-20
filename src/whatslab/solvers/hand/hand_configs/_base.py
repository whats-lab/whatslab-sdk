import os
from typing import ClassVar, Dict, List, Optional

import pinocchio as pin

from whatslab.paths import models_root

FINGER_ORDER = ('thumb', 'index', 'middle', 'ring', 'pinky')


class HandConfig:

    _MODEL_SUBDIR:  ClassVar[str] = ''
    _CHAIN_LEN:     ClassVar[Dict[str, int]] = {}
    _URDF_FILENAME: ClassVar[str] = 'urdf/{hand_type}.urdf'

    def __init__(self, urdf_root=None):
        root = urdf_root or models_root()
        self._models_root = root
        self._urdf_dir = os.path.join(root, self._MODEL_SUBDIR)
        self._cache: Dict[str, tuple] = {}

    def _get_urdf_path(self, hand_type: str) -> str:
        unified = os.path.join(self._urdf_dir, 'urdf', f'{hand_type}.urdf')
        legacy = os.path.join(self._urdf_dir,
                              self._URDF_FILENAME.format(hand_type=hand_type))
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
            if f not in self._CHAIN_LEN:
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
            raise ValueError(f"{type(self).__name__}: _CHAIN_LEN 이 비어 있다")

        shared = set.intersection(*[set(v) for v in chains.values()])
        palm = body(max(shared) if shared else 0)
        if palm is None:
            raise ValueError(f"{type(self).__name__}/{hand_type}: 팜 링크를 못 찾았다")

        fingers = []
        for f in chains:
            links = [n for n in (body(j) for j in chains[f] if j not in shared)
                     if n is not None]
            robot = [palm] + links + [tips[f]]
            want = self._CHAIN_LEN[f]
            if want != len(robot):
                raise ValueError(
                    f"{type(self).__name__}/{hand_type}/{f}: _CHAIN_LEN"
                    f" {want} != URDF 사슬 {len(robot)} — 사슬 {robot}")
            fingers.append(robot)
        self._cache[hand_type] = (fingers, palm)
        return self._cache[hand_type]

    def _get_fingers(self, hand_type: str) -> List[List[str]]:
        return self._derive(hand_type)[0]

    def get_wrist_link_name(self, hand_type: str) -> str:
        return self._derive(hand_type)[1]
