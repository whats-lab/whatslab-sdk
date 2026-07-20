"""
핸드 리타겟팅 설정 기반 클래스 및 공통 유틸리티

새로운 로봇 핸드를 추가하려면:
  1. HandConfig를 상속하는 클래스를 구현 — 클래스 변수 선언만으로 완성
     - _MODEL_SUBDIR    : models/{subdir} 경로
     - _FINGERS         : {'left': [...], 'right': [...]}  FingerChain 리스트
     - _WRIST_LINK      : {'left': '<link>', 'right': '<link>'}
     - _COORD_TRANSFORM : 3×3 변환 행렬 (기본: 단위행렬)
     - _SCALE_FACTOR    : Human 포지션 스케일 (기본: 1.0)
     - _URDF_FILENAME   : (선택) 외부 트리 호환용 레거시 파일명. 내장은 'urdf/{side}.urdf' 통일
     - _SIDE_MAP        : side 포맷 키 (기본: {'left':'left', 'right':'right'})
     - _FIXED_JOINTS    : {'left': '<joint>', 'right': '<joint>'} (기본: 없음)
  2. hand_configs/__init__.py 의 CONFIG_REGISTRY에 등록

Human 손 인덱스 레이아웃 (23 포인트, 0~22)
──────────────────────────────────────────
 0 : wrist
 1 : thumb_cmc0   2 : thumb_cmc1   3 : thumb_mcp    4 : thumb_ip     5 : thumb_tip
 6 : index_mcp    7 : index_pip    8 : index_dip    9 : index_tip
10 : middle_mcp  11 : middle_pip  12 : middle_dip  13 : middle_tip
14 : ring_mcp    15 : ring_pip    16 : ring_dip    17 : ring_tip
18 : pinky0      19 : pinky_mcp   20 : pinky_pip   21 : pinky_dip   22 : pinky_tip
"""

import os
from abc import ABC
from dataclasses import dataclass
from typing import ClassVar, Dict, List, Union

import numpy as np

from whatslab.core.types import JOINT_INDEX
from whatslab.paths import models_root as _models_root


def _resolve_human(human: List) -> List[int]:
    """human 항목을 정규 인덱스로 해석. 골격 관절명(str) 또는 인덱스(int) 모두 허용.

    이름 → core.types.JOINT_INDEX (사람 손 골격 단일 출처)로 변환.
    """
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
    """models 경로 해석: WHATSLAB_MODELS_ROOT > 레포 models/ (whatslab.paths 위임)."""
    return _models_root()


@dataclass
class FingerChain:
    """Single finger: URDF link chain + 대응 사람 손 골격 관절.

    links  — root→tip link names (may contain {side} / {wrist} placeholders)
    human  — 골격 관절명(str, 권장) 또는 정규 인덱스(int). links 와 같은 길이.
             예: ["wrist", "index_mcp", "index_pip", "index_dip", "index_tip"]
    """
    links: List[str]
    human: List  # List[str | int] — _resolve_human 으로 정규 인덱스화


class HandConfig(ABC):
    """로봇 핸드 리타겟팅 설정 기반 클래스.

    서브클래스는 클래스 변수 선언만으로 동작을 완전히 정의할 수 있습니다.
    메서드 오버라이드는 필요하지 않습니다.

    필수
    ----
    _MODEL_SUBDIR  : models/{subdir} 경로
    _FINGERS       : {'left': [...], 'right': [...]}
    _WRIST_LINK    : {'left': '<link>', 'right': '<link>'}

    선택 (기본값 있음)
    ------------------
    _COORD_TRANSFORM : 3×3 변환 행렬     (기본: 단위행렬)
    _SCALE_FACTOR    : 스케일 팩터       (기본: 1.0)
    _URDF_FILENAME   : 외부 트리 호환 레거시 파일명 (기본: 'urdf/{hand_type}.urdf')
    _SIDE_MAP        : side 포맷 값      (기본: {'left':'left', 'right':'right'})
    _FIXED_JOINTS    : 고정 조인트 맵    (기본: {})
    _RVIZ_FILENAME   : RViz 설정 파일명  (기본: {}, hand_view.launch.py에서 사용)
    _TARGET_JOINT_NAMES : mimic joint 포함 URDF용 제어 joint 명시 (기본: [] = 자동)
    """

    _MODEL_SUBDIR:         ClassVar[str]                               = ''
    _FINGERS:              ClassVar[Dict[str, List[FingerChain]]]      = {'left': [], 'right': []}
    _WRIST_LINK:           ClassVar[Dict[str, str]]                    = {'left': 'world', 'right': 'world'}
    _COORD_TRANSFORM:      ClassVar[np.ndarray]                        = np.eye(3, dtype=np.float32)
    _SCALE_FACTOR:         ClassVar[Union[float, List[float]]]         = 1.0
    # 외부 상류 트리(ROS2 등) 호환용 레거시 파일명. 내장 models 는 통일 규칙
    # 'urdf/{hand_type}.urdf' 을 우선 사용하므로 대개 이 기본값이면 충분하다.
    _URDF_FILENAME:        ClassVar[str]                               = 'urdf/{hand_type}.urdf'
    _SIDE_MAP:             ClassVar[Dict[str, str]]                    = {'left': 'left', 'right': 'right'}
    _FIXED_JOINTS:         ClassVar[Dict[str, str]]                    = {}
    _RVIZ_FILENAME:        ClassVar[Dict[str, str]]                    = {}
    # mimic joint가 있는 URDF에서 실제 제어 가능한 joint만 명시. 비어있으면 자동 탐색.
    # 좌우 joint명이 다르면(예: {side}_hand_ 접두어) dict로 지정 가능.
    _TARGET_JOINT_NAMES:   ClassVar[Union[List[str], Dict[str, List[str]]]] = []

    def __init__(self, urdf_root=None):
        """urdf_root: models 디렉토리 경로 (하위에 {_MODEL_SUBDIR}/ 존재).
        미지정 시 WHATSLAB_MODELS_ROOT 환경변수/개발 fallback 사용.
        """
        root = urdf_root or _default_models_root()
        self._models_root = root
        self._urdf_dir = os.path.join(root, self._MODEL_SUBDIR)

    def _get_urdf_path(self, hand_type: str) -> str:
        """URDF 경로 해석.

        내장 models 는 통일 규칙 `{subdir}/urdf/{side}.urdf` 를 따른다. 외부
        상류 트리(예: ROS2 models, 메쉬 포함)는 벤더별 파일명이 달라 config 의
        `_URDF_FILENAME` 을 레거시 fallback 으로 사용한다. 존재하는 첫 후보 반환.
        """
        unified = os.path.join(self._urdf_dir, 'urdf', f'{hand_type}.urdf')
        legacy  = os.path.join(self._urdf_dir, self._URDF_FILENAME.format(hand_type=hand_type))
        for path in (unified, legacy):
            if os.path.exists(path):
                return path
        return unified

    def _get_fingers(self, hand_type: str) -> List[FingerChain]:
        fmt = {'side': self._SIDE_MAP[hand_type], 'wrist': self._WRIST_LINK[hand_type]}
        return [
            FingerChain([l.format(**fmt) for l in f.links], _resolve_human(f.human))
            for f in self._FINGERS[hand_type]
        ]

    def get_two_stage_config(self, hand_type: str):
        """2단계 최적화 설정 (stage1_dict, stage2_dict) 반환."""
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
