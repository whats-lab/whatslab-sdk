from __future__ import annotations

import threading
from typing import Dict, Optional, Tuple

import numpy as np
from scipy.spatial.transform import Rotation

from whatslab.core.constants import SIDES

from ..base import Y_UP_RH_TO_CANONICAL, norm_quat, to_canonical_pose
from .tls import preferred_ip
from .ws_server import WebXRServer

WEBXR_PORT = 8443
# 위치 기준
#   hmd_yaw: 머리 위치 원점 + 머리 yaw 만 따라 돎(몸을 돌려도 앞이 앞)
#   hmd:     머리 위치 원점
#   room:    세션 바닥 좌표(local-floor) 그대로
POS_FRAMES = ("hmd_yaw", "hmd", "room")
# 앞 축 수평 성분이 이보다 작으면(정확히 위·아래를 봄) yaw 를 위 축으로 정한다.
_MIN_HORIZONTAL = 1e-6


def yaw_rz(hmd_quat: np.ndarray) -> np.ndarray:
    """머리 yaw 를 되돌리는 z 회전 Rz(-yaw).

    yaw 는 머리 앞 축(정준 x)을 바닥 평면에 투영한 방향이다 — 롤에 영향받지 않는다.
    정확히 위·아래를 볼 때(투영이 0)는 위 축의 수평 방향(아래를 보면 앞, 위를 보면 뒤를
    향한다)을 쓴다. 피치가 ±90° 에 가까울수록 투영이 짧아 머리를 조금 움직여도 yaw 가 크게
    바뀐다.
    """
    hmd_rot = Rotation.from_quat(np.asarray(hmd_quat, dtype=float)).as_matrix()
    fwd, up = hmd_rot[:, 0], hmd_rot[:, 2]
    v = fwd[:2]
    if np.linalg.norm(v) < _MIN_HORIZONTAL:
        v = -np.sign(fwd[2]) * up[:2]
    h = float(np.arctan2(v[1], v[0]))
    c, s = np.cos(-h), np.sin(-h)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def head_yaw_relative_quat(quat: np.ndarray, hmd_quat: np.ndarray) -> np.ndarray:
    """방위 쿼터니언에서 머리 yaw 를 뺀다 — 위치의 ``yaw_rz(hmd) @ p`` 와 같은 규약."""
    rot = Rotation.from_quat(np.asarray(quat, dtype=float)).as_matrix()
    return Rotation.from_matrix(yaw_rz(hmd_quat) @ rot).as_quat()


def parse_pose(item: object) -> Optional[Tuple[np.ndarray, np.ndarray]]:
    """메시지 항목 ``{pos: [3], quat: [4]}`` → (위치, 정규화 쿼터니언). 모양·값이 잘못되면 None."""
    if not isinstance(item, dict):
        return None
    try:
        pos = np.asarray(item.get("pos"), dtype=float)
        quat = np.asarray(item.get("quat"), dtype=float)
    except (TypeError, ValueError):
        return None
    if pos.shape != (3,) or quat.shape != (4,):
        return None
    if not (np.all(np.isfinite(pos)) and np.all(np.isfinite(quat))):
        return None
    return pos, norm_quat(quat)


class WebXRReceiverBase:
    """브라우저 WebXR 페이지(static/index.html)가 WebSocket 으로 보내는 프레임을 받는다.

    같은 고정 포트의 수신기들(컨트롤러·손)은 서버 하나를 공유하고, 메시지는 시작한 수신기 모두에
    간다. port=0 은 수신기마다 따로 임시 포트를 잡으므로 공유하지 않는다.
    """

    def __init__(
        self,
        port: int = WEBXR_PORT,
        listen_ip: str = "0.0.0.0",
        tls: bool = True,
        certfile: str = None,
        keyfile: str = None,
        *,
        pos_frame: str = "hmd_yaw",
    ):
        if pos_frame not in POS_FRAMES:
            raise ValueError(f"pos_frame 은 {list(POS_FRAMES)} 중 하나 — 받은 값 {pos_frame!r}")
        self._srv = WebXRServer.get(port, listen_ip, tls, certfile, keyfile)
        self._lock = threading.Lock()
        self._pos_frame = pos_frame
        self._state: Dict[str, dict] = {s: {} for s in SIDES}
        self._hmd_pos = np.zeros(3)
        self._hmd_quat = np.array([0.0, 0.0, 0.0, 1.0])
        self._hmd_valid = False
        self._started = False

    @property
    def port(self) -> int:
        return self._srv.port

    @property
    def url(self) -> str:
        host = preferred_ip() if self._srv.scheme == "https" else "localhost"
        return f"{self._srv.scheme}://{host}:{self._srv.port}/"

    @property
    def running(self) -> bool:
        """start() 했고 아직 stop() 하지 않았는지."""
        return self._started

    @property
    def pos_frame(self) -> str:
        """위치 기준(POS_FRAMES 중 하나)."""
        return self._pos_frame

    def start(self) -> None:
        """서버에 붙어 수신을 시작한다. 이미 시작했으면 아무것도 안 한다."""
        if self._started:
            return
        self._srv.add_handler(self._on_message)
        try:
            self._srv.start()
        except BaseException:  # bind·인증서 실패 — 시작 안 된 수신기가 메시지를 받지 않게
            self._srv.remove_handler(self._on_message)
            raise
        self._started = True

    def stop(self) -> None:
        """서버에서 떨어진다(마지막 수신기면 서버가 멈춘다). 시작 안 했으면 아무것도 안 한다."""
        if not self._started:
            return
        self._started = False
        self._srv.remove_handler(self._on_message)
        self._srv.stop()

    def get_hmd(self):
        with self._lock:
            pos, quat, valid = self._hmd_pos.copy(), self._hmd_quat.copy(), self._hmd_valid
        if not valid:
            return pos, quat, valid
        pos, quat = to_canonical_pose(Y_UP_RH_TO_CANONICAL, pos, quat)
        return pos, quat, valid

    def relative(
        self, pos: np.ndarray, hmd_pos: np.ndarray, hmd_quat: np.ndarray, hmd_valid: bool
    ) -> np.ndarray:
        """정준 프레임 위치를 pos_frame 기준으로."""
        if self._pos_frame == "room" or not hmd_valid:
            return pos
        rel = pos - hmd_pos
        if self._pos_frame == "hmd":
            return rel
        return yaw_rz(hmd_quat) @ rel

    def relative_quat(self, quat: np.ndarray, hmd_quat: np.ndarray, hmd_valid: bool) -> np.ndarray:
        """정준 프레임 방위를 pos_frame 기준으로.

        hmd_yaw 일 때만 머리 yaw 를 뺀다(위치와 같은 기준).
        """
        if self._pos_frame != "hmd_yaw" or not hmd_valid:
            return quat
        return head_yaw_relative_quat(quat, hmd_quat)

    def _on_message(self, msg: dict) -> None:
        hmd = msg.get("hmd")
        if isinstance(hmd, dict):
            self._set_hmd(hmd)
        self._handle(msg)

    def _set_hmd(self, item: dict) -> None:
        pose = parse_pose(item)
        if pose is None:
            return
        with self._lock:
            self._hmd_pos, self._hmd_quat = pose
            self._hmd_valid = True

    def _handle(self, msg: dict) -> None:
        raise NotImplementedError
