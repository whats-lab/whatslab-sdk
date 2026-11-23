"""WebXR 핸드트래킹 — 브라우저(Quest 등)가 보내는 손 관절 25개를 OpenXR 26관절로 받는다.

WebXR Hand Input 의 25관절(wrist, thumb-metacarpal … pinky-finger-tip)은 OpenXR
`XrHandJointEXT` 에서 palm 하나만 빠진 같은 순서·같은 관절 축(−Z 손끝 방향, +Y 손등)이다.
palm 은 Spine·AirGlove-Client 와 같은 규칙(중지 중수골·근위 관절 위치의 중점,
방향 = 중지 중수골)으로 만든다.

메시지: ``{"hands": {"left": [25×8 float], "right": …}}`` — 관절마다
``[qx qy qz qw px py pz radius]``, 세션 바닥 좌표(local-floor, 오른손계 y-up).
`get(side)` 는 손목 기준 26×8(`HandPose.xr_joints`)과 정준 프레임의 손목 자세
(`HandPose.wrist`)를 준다. 손목 위치와 방위는 둘 다 pos_frame 기준이다(hmd_yaw 면 둘 다
머리 yaw 를 뺀다).
"""

from __future__ import annotations

import time
from typing import Optional

import numpy as np
from scipy.spatial.transform import Rotation

from whatslab.core.constants import SIDES, XR_JOINT_COUNT, XR_JOINT_FIELDS
from whatslab.core.types import HandPose, InputSample, Pose

from ..base import Y_UP_RH_TO_CANONICAL, norm_quat, to_canonical_pose
from .base import WEBXR_PORT, WebXRReceiverBase

WEBXR_HAND_JOINT_COUNT = 25  # WebXR XRHandJoint (OpenXR 26 에서 palm 제외)
WEBXR_WRIST = 0
WEBXR_MIDDLE_METACARPAL = 10
WEBXR_MIDDLE_PROXIMAL = 11


def webxr_hand_to_openxr26(joints25: np.ndarray) -> Optional[np.ndarray]:
    """WebXR 25관절(세션 좌표) → OpenXR 26관절(손목 기준). 값이 유한하지 않으면 None."""
    a = np.asarray(joints25, dtype=float).reshape(WEBXR_HAND_JOINT_COUNT, XR_JOINT_FIELDS)
    if not np.all(np.isfinite(a)):
        return None
    quats = a[:, :4]
    norms = np.linalg.norm(quats, axis=1, keepdims=True)
    if np.any(norms < 1e-6):
        return None
    rot = Rotation.from_quat(quats / norms)
    wrist_rot = rot[WEBXR_WRIST].inv()
    rel_pos = wrist_rot.apply(a[:, 4:7] - a[WEBXR_WRIST, 4:7])
    rel_quat = (wrist_rot * rot).as_quat()

    out = np.zeros((XR_JOINT_COUNT, XR_JOINT_FIELDS))
    out[1:, 0:4] = rel_quat
    out[1:, 4:7] = rel_pos
    out[1:, 7] = a[:, 7]
    out[0, 0:4] = rel_quat[WEBXR_MIDDLE_METACARPAL]
    out[0, 4:7] = 0.5 * (rel_pos[WEBXR_MIDDLE_METACARPAL] + rel_pos[WEBXR_MIDDLE_PROXIMAL])
    out[0, 7] = a[WEBXR_MIDDLE_METACARPAL, 7]
    out[1, 0:4] = (0.0, 0.0, 0.0, 1.0)  # 손목은 항등 (수치 오차 제거)
    out[1, 4:7] = 0.0
    return out


class WebXRHandReceiver(WebXRReceiverBase):
    """WebXR 핸드트래킹 손목 6D + OpenXR 26관절.

    다른 수신기와 같은 고정 포트를 주면 서버를 함께 쓴다(port=0 은 수신기마다 따로인 임시
    포트라 공유하지 않는다). 메시지는 ``start()`` 한 수신기만 받는다 — 서버를 공유해도 각 수신기가
    ``start()`` 해야 한다(HandModel.start 가 한다).
    """

    def __init__(
        self,
        port: int = WEBXR_PORT,
        listen_ip: str = "0.0.0.0",
        stale_timeout: float = 0.2,
        pos_frame: str = "hmd_yaw",
        tls: bool = True,
        certfile: str = None,
        keyfile: str = None,
    ):
        super().__init__(port, listen_ip, tls, certfile, keyfile, pos_frame=pos_frame)
        self._stale_timeout = float(stale_timeout)
        for side in SIDES:
            s = self._state[side]
            s["joints"] = None
            s["wrist_pos"] = np.zeros(3)
            s["wrist_quat"] = np.array([0.0, 0.0, 0.0, 1.0])
            s["timestamp"] = 0.0

    def _handle(self, msg: dict) -> None:
        hands = msg.get("hands")
        if not isinstance(hands, dict):
            return
        for side in SIDES:
            raw = hands.get(side)
            if not isinstance(raw, list) or len(raw) != WEBXR_HAND_JOINT_COUNT * XR_JOINT_FIELDS:
                continue
            try:
                a = np.asarray(raw, dtype=float).reshape(WEBXR_HAND_JOINT_COUNT, XR_JOINT_FIELDS)
            except (TypeError, ValueError):
                continue
            joints = webxr_hand_to_openxr26(a)
            if joints is None:
                continue
            with self._lock:
                s = self._state[side]
                s["joints"] = joints
                s["wrist_pos"] = a[WEBXR_WRIST, 4:7].copy()
                s["wrist_quat"] = norm_quat(a[WEBXR_WRIST, 0:4])
                s["timestamp"] = time.monotonic()

    def get(self, side: str) -> InputSample:
        """side 의 최신 손 샘플(정준 프레임, pos_frame 기준)."""
        with self._lock:
            s = self._state[side]
            joints = None if s["joints"] is None else s["joints"].copy()
            wrist_pos, wrist_quat = s["wrist_pos"].copy(), s["wrist_quat"].copy()
            ts = s["timestamp"]
        hmd_pos, hmd_quat, hmd_valid = self.get_hmd()
        age = time.monotonic() - ts
        tracked = joints is not None and not (
            self._stale_timeout > 0 and age > self._stale_timeout
        )
        hmd = Pose(hmd_pos, hmd_quat) if hmd_valid else None
        if joints is None:
            return InputSample(hmd=hmd, tracked=False, timestamp=ts)
        pos, quat = to_canonical_pose(Y_UP_RH_TO_CANONICAL, wrist_pos, wrist_quat)
        wrist = Pose(
            self.relative(pos, hmd_pos, hmd_quat, hmd_valid),
            self.relative_quat(quat, hmd_quat, hmd_valid),
        )
        hand = HandPose(
            wrist=wrist,
            xr_joints=joints,
            xr_valid=np.ones(XR_JOINT_COUNT, dtype=bool),
            tracked=tracked,
            timestamp=ts,
        )
        return InputSample(hand=hand, hmd=hmd, tracked=tracked, timestamp=ts)

    def connected(self, side: str) -> bool:
        """그 side 손이 지금 추적 중인지."""
        return self.get(side).tracked
