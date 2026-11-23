from __future__ import annotations

import logging
import math
import threading
import time
from dataclasses import dataclass
from typing import Optional

import numpy as np
from airglove_client import JOINT_NAMES, AirGloveError, Client, Side

from whatslab.core.constants import FINGER_COUNT, OPENXR26_TO_HAND21, XR_JOINT_NAMES
from whatslab.core.types import HandPose, InputSample

_log = logging.getLogger(__name__)

GLOVE_OSC_PORT = 4040          # Spine → 이 클라이언트 (PC 당 하나)
GLOVE_CLIENT_PORT = 4042       # 이 클라이언트 → Spine
GLOVE_TARGET_IP = "127.0.0.1"  # Spine 은 로컬 전용

if tuple(JOINT_NAMES) != XR_JOINT_NAMES:   # airglove-client 와 core 의 관절 순서가 같아야 한다
    raise ImportError("airglove_client JOINT_NAMES 가 OpenXR 26관절 순서와 다르다")

HAPTIC_LEVEL_MAX = 3
OPENXR_STALE_TIMEOUT = 0.2
DEVICE_STATUS_MAX_AGE = 2.0
SPINE_PROTOCOL = 3
DEFAULT_PAIR = 1
MIN_LIBRARY_VERSION = (0, 4)

_SIDES = {"left": Side.LEFT, "right": Side.RIGHT}
_HAND21 = list(OPENXR26_TO_HAND21)


def haptic_levels(values) -> list:
    out = [0] * FINGER_COUNT
    for i, v in enumerate(list(values)[:FINGER_COUNT]):
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


@dataclass(frozen=True)
class GloveFrame:
    side: str
    xr_joints: np.ndarray
    valid: np.ndarray
    seq: int
    sender_time_us: int
    age_s: float


class AirGloveReceiver:
    """AirGlove Spine 3.0.0 OpenXR 26관절 수신기(airglove-client 래퍼)."""

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
                _log.warning(
                    "airglove_client 네이티브 라이브러리 %s 는 Spine 프로토콜 %d 이전 — "
                    "0.4.0 이상이 필요하다(AIRGLOVE_CLIENT_LIBRARY)",
                    version, SPINE_PROTOCOL)
            client = Client(
                listen_address=self._listen_ip,
                listen_port=self._glove_port,
                spine_address=self._target_ip,
                spine_port=self._client_port,
            )
            if self._pair != DEFAULT_PAIR:
                client.set_pair(self._pair)
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
            _log.warning(
                "Spine 프로토콜 %s 보고됨 — 이 리시버는 %d 기준이다", reported[0], SPINE_PROTOCOL
            )

    def spine_protocol(self) -> Optional[int]:
        with self._lock:
            if self._client is None:
                return None
            reported = self._client.spine_protocol()
        return None if reported is None else int(reported[0])

    def frame(self, side: str) -> Optional[GloveFrame]:
        frame = self._read(side)
        if frame is None:
            return None
        return GloveFrame(
            side=side,
            xr_joints=_xr_joints(frame),
            valid=np.asarray(frame.valid, dtype=bool).copy(),
            seq=int(frame.seq),
            sender_time_us=int(frame.sender_time_us),
            age_s=float(frame.age_s),
        )

    def device_status(self):
        with self._lock:
            return None if self._client is None else self._client.device_status()

    def alarms(self) -> list:
        with self._lock:
            return [] if self._client is None else list(self._client.alarms())

    def stats(self) -> dict:
        with self._lock:
            return {} if self._client is None else dict(self._client.stats())

    def _fresh(self, age_s: float) -> bool:
        return not (self._stale_timeout > 0 and age_s >= self._stale_timeout)

    def get(self, side: str) -> InputSample:
        f = self.frame(side)
        if f is None:
            return InputSample(tracked=False, timestamp=0.0)
        ts = time.monotonic() - f.age_s
        # 리타게팅이 쓰는 21점만 본다 — 안 쓰는 palm·중수골이 무효여도 추적 중이다.
        tracked = self._fresh(f.age_s) and bool(np.all(f.valid[_HAND21]))
        hand = HandPose(xr_joints=f.xr_joints, xr_valid=f.valid, tracked=tracked, timestamp=ts)
        return InputSample(hand=hand, tracked=tracked, timestamp=ts)

    def connected(self, side: str) -> bool:
        key = _SIDES[side]
        with self._lock:
            if self._client is None:
                return False
            status = self._client.device_status()
            frame = self._client.hand(key)
        if (
            status is not None
            and float(status.age_s) < DEVICE_STATUS_MAX_AGE
            and (status.left_connected if key == Side.LEFT else status.right_connected)
        ):
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
