from __future__ import annotations

import logging
import math
import threading
import time
from typing import Optional

import numpy as np
from airglove_client import JOINT_COUNT, JOINT_NAMES, AirGloveError, Client, Side
from scipy.spatial.transform import Rotation

from whatslab.core.types import HandPose, InputSample
from ..webxr.base import _CANONICAL_M
from .base import AGA_FINGER_COUNT, GLOVE_CLIENT_PORT, GLOVE_OSC_PORT, GLOVE_TARGET_IP

_log = logging.getLogger(__name__)

XR_JOINT_COUNT = JOINT_COUNT
XR_JOINT_FIELDS = 8
XR_JOINT_NAMES = tuple(JOINT_NAMES)
XR_WRIST_INDEX = XR_JOINT_NAMES.index("wrist")
HAPTIC_LEVEL_MAX = 3
OPENXR_STALE_TIMEOUT = 0.2
DEVICE_STATUS_MAX_AGE = 2.0
SPINE_PROTOCOL = 3
DEFAULT_PAIR = 1
MIN_LIBRARY_VERSION = (0, 4)

_SIDES = {"left": Side.LEFT, "right": Side.RIGHT}


def openxr_quat_to_canonical(quat) -> np.ndarray:
    R = Rotation.from_quat(np.asarray(quat, dtype=float)).as_matrix()
    return Rotation.from_matrix(_CANONICAL_M @ R @ _CANONICAL_M.T).as_quat()


def haptic_levels(values) -> list:
    out = [0] * AGA_FINGER_COUNT
    for i, v in enumerate(list(values)[:AGA_FINGER_COUNT]):
        try:
            f = float(v)
        except (TypeError, ValueError):
            f = 0.0
        if not math.isfinite(f):
            f = 0.0
        out[i] = int(min(max(round(f), 0), HAPTIC_LEVEL_MAX))
    return out


def _xr_joints(frame) -> np.ndarray:
    return np.concatenate([np.asarray(frame.orientations, dtype=float),
                           np.asarray(frame.positions, dtype=float),
                           np.asarray(frame.radii, dtype=float).reshape(-1, 1)], axis=1)


def _library_version(version: str) -> tuple:
    try:
        return tuple(int(x) for x in str(version).split(".")[:2])
    except ValueError:
        return (0, 0)


class GloveOpenXRReceiver:

    def __init__(
        self,
        glove_port: int = GLOVE_OSC_PORT,
        listen_ip: str = "127.0.0.1",
        target_ip: str = GLOVE_TARGET_IP,
        client_port: int = GLOVE_CLIENT_PORT,
        stale_timeout: float = OPENXR_STALE_TIMEOUT,
        pair: int = DEFAULT_PAIR,
    ):
        if int(pair) < 1:
            raise ValueError(f"pair 는 1 이상 — 받은 값 {pair!r}")
        self._glove_port = int(glove_port)
        self._listen_ip = listen_ip
        self._target_ip = target_ip
        self._client_port = int(client_port)
        self._stale_timeout = float(stale_timeout)
        self._pair = int(pair)
        self._lock = threading.Lock()
        self._client: Optional[Client] = None
        self._protocol_checked = False

    @property
    def running(self) -> bool:
        with self._lock:
            return self._client is not None

    def start(self) -> None:
        with self._lock:
            if self._client is not None:
                return
            version = Client.version()
            if _library_version(version) < MIN_LIBRARY_VERSION:
                _log.warning("airglove_client 네이티브 라이브러리 %s 는 Spine 프로토콜 %d 이전 — "
                             "0.4.0 이상이 필요하다(AIRGLOVE_CLIENT_LIBRARY)", version, SPINE_PROTOCOL)
            client = Client(listen_address=self._listen_ip, listen_port=self._glove_port,
                            spine_address=self._target_ip, spine_port=self._client_port)
            if self._pair != DEFAULT_PAIR:
                set_pair = getattr(client, "set_pair", None)
                if set_pair is None:
                    client.close()
                    raise AirGloveError(f"이 airglove_client 는 set_pair 가 없어 pair {self._pair} 를 "
                                        "쓸 수 없다(0.4.0 이상 필요)")
                set_pair(self._pair)
            self._client = client
            self._protocol_checked = False

    def stop(self) -> None:
        with self._lock:
            client, self._client = self._client, None
        if client is not None:
            client.close()

    def _read(self, side: str):
        key = _SIDES[side]
        with self._lock:
            if self._client is None:
                return None
            if not self._protocol_checked:
                self._check_protocol()
            return self._client.hand(key)

    def _check_protocol(self) -> None:
        reported = self._client.spine_protocol()
        if reported is None:
            return
        self._protocol_checked = True
        if int(reported[0]) != SPINE_PROTOCOL:
            _log.warning("Spine 프로토콜 %s 보고됨 — 이 리시버는 %d 기준이다", reported[0], SPINE_PROTOCOL)

    def spine_protocol(self) -> Optional[int]:
        with self._lock:
            if self._client is None:
                return None
            reported = self._client.spine_protocol()
        return None if reported is None else int(reported[0])

    def raw_wrist(self, side: str) -> Optional[np.ndarray]:
        key = _SIDES[side]
        with self._lock:
            if self._client is None:
                return None
            frame = self._client.wrist(key)
        if frame is None or (self._stale_timeout > 0 and float(frame.age_s) >= self._stale_timeout):
            return None
        return np.asarray(frame.orientation, dtype=float).copy()

    def _fresh(self, age_s: float) -> bool:
        return not (self._stale_timeout > 0 and age_s >= self._stale_timeout)

    def get(self, side: str) -> InputSample:
        frame = self._read(side)
        if frame is None:
            return InputSample(tracked=False, timestamp=0.0)
        ts = time.monotonic() - float(frame.age_s)
        tracked = self._fresh(float(frame.age_s)) and bool(np.all(frame.valid))
        hand = HandPose(wrist=None, xr_joints=_xr_joints(frame), tracked=tracked, timestamp=ts)
        return InputSample(hand=hand, tracked=tracked, timestamp=ts)

    def connected(self, side: str) -> bool:
        key = _SIDES[side]
        with self._lock:
            if self._client is None:
                return False
            status = self._client.device_status()
            frame = self._client.hand(key)
        if (status is not None and float(status.age_s) < DEVICE_STATUS_MAX_AGE
                and (status.left_connected if key == Side.LEFT else status.right_connected)):
            return True
        return frame is not None and self._fresh(float(frame.age_s))

    def send_haptic(self, side: str, values: list) -> bool:
        key = _SIDES[side]
        levels = haptic_levels(values)
        with self._lock:
            if self._client is None:
                return False
            try:
                self._client.set_haptics(key, levels)
            except AirGloveError as e:
                _log.warning("햅틱 전송 실패 (%s): %s", side, e)
                return False
        return True
