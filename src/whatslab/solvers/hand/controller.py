from __future__ import annotations

import logging
import os
from typing import List, Mapping, Optional, Tuple

import numpy as np

from whatslab.core.types import HandCommand, HandPose, InputSample

_log = logging.getLogger(__name__)

DEX_PREFIX = "dex:"
GEORT_PREFIX = "geort:"
ENGINES = (DEX_PREFIX, GEORT_PREFIX)


def engine_of(config_name) -> str:
    """retarget 이름의 엔진 접두사(`dex:` / `geort:`). 아니면 ValueError."""
    if isinstance(config_name, str):
        for prefix in ENGINES:
            if config_name.startswith(prefix) and len(config_name) > len(prefix):
                return prefix
    raise ValueError(
        "손 리타게팅 이름은 dex:<손> (DexPilot) 또는 geort:<설정> (GeoRT) — "
        f"받은 값 {config_name!r}"
    )


def geort_config_path(name: str) -> str:
    """`geort:<이름>` 의 설정 경로.

    이름만 주면 configs 루트(``$WHATSLAB_CONFIGS_ROOT``, 없으면 패키지 동봉)의 geort/ 를 쓴다 —
    현재 디렉토리의 같은 이름 파일이 아니라. dex 의 configs/hands 와 같은 규칙이다.
    `.yaml` 로 끝나면 사용자 경로로 본다.
    """
    if name.endswith((".yaml", ".yml")):
        return name
    from whatslab.paths import configs_root

    return os.path.join(configs_root(), "geort", f"{name}.yaml")


class HandRetargetController:
    """`HandPose.xr_joints`(OpenXR 26×8) → 로봇 손 관절각. `HandController` 구현.

    엔진은 이름 접두사로 고르고 계산만 맡긴다:
    - ``dex:<손>``  → `XRHandRetargeter` (dex-retargeting DexPilot,
      configs/hands/<손>_<side>.yaml)
    - ``geort:<설정>`` → `GeortHandSolver` (GeoRT 소프트 프롬프트,
      configs/geort/<설정>.yaml 또는 경로)

    공통 정책은 여기 한 곳에 있다: 추적이 없거나 이번 프레임을 풀 수 없으면(dex 의
    `HandInputError`, geort 의 `GeortInputError` — 그 밖의 예외는 올린다) 직전 명령을
    유지하고(못 푼 이유는 `last_error`, 연속 실패의 첫 번에 경고 로그), 추적이 끊기는 순간 엔진
    상태(low-pass·warm start)를 한 번 비운다. 첫 입력 전 명령은 0 을 관절범위로 자른 값이다.
    `HandModel` 은 추적이 없는 틱에 compute 를 부르지 않고 손 관절을 q 에서 뺀다 — 직전 명령
    유지는 직접 부르는 쪽에 적용된다.

    ``robot_limits`` ({관절: (lo, hi)}) 는 geort 출력을 실제 로봇 범위로 더 좁힌다(dex 는 URDF
    범위를 직접 쓰므로 무시). ``urdf_root`` 는 dex 만 쓰고 geort 에서는 무시한다. 두 인자 모두
    엔진과 관계없이 넘겨도 된다.
    """

    def __init__(
        self,
        hand_type: str,
        config_name: str = "dex:orca",
        urdf_root=None,
        robot_limits: Optional[Mapping[str, Tuple[float, float]]] = None,
        **kwargs,
    ):
        self.engine_name = engine_of(config_name)
        name = config_name[len(self.engine_name):]
        if self.engine_name == DEX_PREFIX:
            from .dex_retargeter import XRHandRetargeter

            if robot_limits is not None:
                _log.debug("dex 엔진은 손 URDF 범위를 직접 쓴다 — robot_limits 무시")
            from .xr_input import HandInputError

            self._engine = XRHandRetargeter(name, side=hand_type, urdf_root=urdf_root, **kwargs)
            self._frame_errors: Tuple[type, ...] = (HandInputError,)
        else:
            from .geort import GeortHandSolver, GeortInputError

            if urdf_root is not None:
                _log.debug("geort 엔진은 urdf_root 를 쓰지 않는다 — 무시 (%s)", urdf_root)
            self._engine = GeortHandSolver.from_config(
                geort_config_path(name), human_side=hand_type, robot_limits=robot_limits, **kwargs
            )
            # 이번 프레임만 못 푸는 입력 오류 — 그 밖의 예외(모양 오류 등 버그)는 그대로 올린다.
            self._frame_errors = (GeortInputError,)
        lim = self._engine.joint_limits
        self._last = np.clip(np.zeros(len(self._engine.joint_names)), lim[:, 0], lim[:, 1])
        self._tracking = False
        self.last_error: Optional[ValueError] = None

    @property
    def joint_names(self) -> List[str]:
        return list(self._engine.joint_names)

    @property
    def engine(self):
        """`XRHandRetargeter`(dex) 또는 `GeortHandSolver`(geort)."""
        return self._engine

    def reset(self) -> None:
        """엔진 상태(low-pass·warm start)·추적 플래그·last_error 를 비운다."""
        self._engine.reset()
        self._tracking = False
        self.last_error = None

    @property
    def robot_side(self) -> str:
        """엔진이 내는 관절각의 로봇 손 side(사람 손 side 와 다를 수 있다).

        dex 는 설정의 side, geort 는 모델의 side 다 — 오른손 geort 모델은 왼손 사람 입력을
        반전해 풀어도 오른손 로봇 관절을 낸다.
        """
        if self.engine_name == DEX_PREFIX:
            return self._engine.config.side
        return self._engine.model.side

    def _solve(self, hand: HandPose) -> np.ndarray:
        from .geort import masked_keypoints

        # 무효 관절(원점·항등으로 남은 값)은 두 엔진 모두 NaN 으로 가려 넣는다 — 쓰는 관절이
        # 무효면 엔진이 이번 프레임 입력 오류(HandInputError·GeortInputError)를 낸다.
        joints = masked_keypoints(hand.xr_joints, hand.xr_valid)
        if self.engine_name == DEX_PREFIX:
            return self._engine.compute(joints)
        return self._engine.solve(joints)

    def compute(self, sample: InputSample) -> HandCommand:
        hand = sample.hand
        if hand is None or not hand.tracked or hand.xr_joints is None:
            if self._tracking:
                self.reset()
            self.last_error = None  # 지금 프레임 기준 — 다시 잡힌 뒤의 첫 실패도 경고하게
        else:
            try:
                self._last = np.asarray(self._solve(hand), dtype=float)
                self._tracking = True
                self.last_error = None
            except self._frame_errors as e:
                first = self.last_error is None
                if first:
                    _log.warning("손 리타게팅 입력을 풀 수 없어 직전 명령을 유지한다: %s", e)
                self.reset()
                self.last_error = e
        return HandCommand(joint_names=self.joint_names, joint_angles=self._last.copy())
