"""WebXR 핸드트래킹 — 브라우저(Quest 등)가 보내는 손 관절 25개를 OpenXR 26관절로 받는다.

WebXR Hand Input 의 25관절(wrist, thumb-metacarpal … pinky-finger-tip)은 OpenXR
`XrHandJointEXT` 에서 palm 하나만 빠진 같은 순서·같은 관절 축(−Z 손끝 방향, +Y 손등)이다.
palm 은 Spine·AirGlove-Client 와 같은 규칙(중지 중수골·근위 관절 위치의 중점,
방향 = 중지 중수골)으로 만든다.

메시지: ``{"hands": {"left": [25×8 float], "right": …}}`` — 관절마다
``[qx qy qz qw px py pz radius]``, 세션 바닥 좌표(local-floor). `get(side)` 는 손목 기준
26×8(`HandPose.xr_joints`)만 준다 — 손목 자세는 이 SDK 의 범위가 아니다.
"""

from __future__ import annotations

import threading
import time
from typing import Optional

import numpy as np
from scipy.spatial.transform import Rotation

from whatslab.core.constants import SIDES, XR_JOINT_COUNT, XR_JOINT_FIELDS
from whatslab.core.types import HandPose, InputSample

from .tls import preferred_ip
from .ws_server import WebXRServer

WEBXR_PORT = 8443
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


class WebXRHandReceiver:
    """헤드셋 브라우저의 WebXR 핸드트래킹 → OpenXR 26관절(양손).

    정적 페이지(static/index.html)와 WebSocket 을 한 포트에서 연다. 기본은 자체 서명 HTTPS
    (``tls=True``, 인증서는 `ensure_cert` 가 자동 생성), 유선 ``adb reverse`` 는 ``tls=False``.
    ``stale_timeout`` 초 동안 새 프레임이 없으면 추적 아님(0 이면 끄기).
    """

    def __init__(
        self,
        port: int = WEBXR_PORT,
        listen_ip: str = "0.0.0.0",
        stale_timeout: float = 0.2,
        tls: bool = True,
        certfile: str = None,
        keyfile: str = None,
    ):
        self._srv = WebXRServer(self._on_message, port, listen_ip, tls, certfile, keyfile)
        self._stale_timeout = float(stale_timeout)
        self._lock = threading.Lock()
        self._joints = {s: None for s in SIDES}
        self._timestamp = {s: 0.0 for s in SIDES}

    @property
    def port(self) -> int:
        """실제로 연 포트(port=0 이면 시작 뒤 정해진다)."""
        return self._srv.port

    @property
    def url(self) -> str:
        """헤드셋 브라우저로 열 주소."""
        host = preferred_ip() if self._srv.scheme == "https" else "localhost"
        return f"{self._srv.scheme}://{host}:{self._srv.port}/"

    @property
    def running(self) -> bool:
        """서버가 열려 있는지."""
        return self._srv.is_running

    def start(self) -> None:
        """서버를 열어 수신을 시작한다. 이미 시작했으면 아무것도 안 한다."""
        self._srv.start()

    def stop(self) -> None:
        """서버를 닫는다. 시작 안 했으면 아무것도 안 한다."""
        self._srv.stop()

    def _on_message(self, msg: dict) -> None:
        hands = msg.get("hands")
        if not isinstance(hands, dict):
            return
        for side in SIDES:
            raw = hands.get(side)
            if not isinstance(raw, list) or len(raw) != WEBXR_HAND_JOINT_COUNT * XR_JOINT_FIELDS:
                continue
            try:
                joints = webxr_hand_to_openxr26(np.asarray(raw, dtype=float))
            except (TypeError, ValueError):
                continue
            if joints is None:
                continue
            with self._lock:
                self._joints[side] = joints
                self._timestamp[side] = time.monotonic()

    def get(self, side: str) -> InputSample:
        """side 의 최신 손 샘플."""
        with self._lock:
            joints = self._joints[side]
            ts = self._timestamp[side]
        if joints is None:
            return InputSample(tracked=False, timestamp=ts)
        age = time.monotonic() - ts
        tracked = not (self._stale_timeout > 0 and age > self._stale_timeout)
        hand = HandPose(
            xr_joints=joints.copy(),
            xr_valid=np.ones(XR_JOINT_COUNT, dtype=bool),
            tracked=tracked,
            timestamp=ts,
        )
        return InputSample(hand=hand, tracked=tracked, timestamp=ts)

    def connected(self, side: str) -> bool:
        """그 side 손이 지금 추적 중인지."""
        return self.get(side).tracked
