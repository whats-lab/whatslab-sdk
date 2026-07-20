"""GloveReceiverBase — SharedOscServer 위에 얹는 글러브(AirGlove 등) 공통 기반.

Quest 쪽 QuestReceiverBase(quest_base.py) 와 대칭 구조다. 같은 글러브 기기가
보내는 여러 modality(현재는 손가락 회전 하나뿐이나 확장 대비)가 포트 하나
(OSC 서버 소켓)를 공유하도록 한다:

  - `SharedOscServer.get(glove_port)` 로 서버를 획득(직접 소켓을 열지 않음).
  - `/device/status/get` (좌/우 공통 연결 상태)은 여기서 한 번만 등록해 모든
    서브클래스가 `connected(side)` 로 재사용한다.
  - `start()/stop()` 은 SharedOscServer 의 refcount 에 위임 — 여러 receiver 가
    동시에 start 해도 마지막 stop() 에서만 실제로 서버가 닫힌다.
  - 피드백/햅틱 송신용 OSC client(target_ip:client_port) 배선을 여기 준비해둔다
    (start() 시 lazy 생성) — 실제 송신 메시지 포맷은 modality 마다 다르므로
    서브클래스(GloveHumanHandReceiver 등)가 구현한다. 하트비트("/device/status/get")
    송신은 이 base 가 공통으로 담당한다(기존 AirGloveReceiver 와 동일 주기).

서브클래스는 자신의 modality OSC 주소를 `self._srv.add_handler(...)` 로 등록하고,
`self._state[side]` 에 원하는 필드를 채운 뒤 `get(side) -> InputSample` 을 구현한다.
RECEIVER 는 입력 전용 계약이다 — 리타게팅/IK/캘리브레이션은 여기 두지 않는다
(Model 계층 소관).

side 는 물리적 글러브의 좌/우 — 재해석 금지(크로스핸드 조합은 Model 의 arm_side/hand_side).
"""
from __future__ import annotations

import logging
import threading
import time
from typing import Dict, Optional

from pythonosc.udp_client import SimpleUDPClient

from .osc_transport import SharedOscServer

_log = logging.getLogger(__name__)

GLOVE_OSC_PORT = 4040       # AGA 글러브 → PC OSC 수신 포트. 기존 airglove.OSC_SERVER_PORT 와 동일 값.
GLOVE_CLIENT_PORT = 4042    # PC → 글러브 (하트비트/햅틱) 송신 포트. 기존 airglove.OSC_CLIENT_PORT 와 동일 값.
GLOVE_TARGET_IP = "127.0.0.1"
HEARTBEAT_INTERVAL_SEC = 1.0


class GloveReceiverBase:
    """글러브 계열 modality receiver 공통 기반 (core.Receiver 프로토콜: start/stop/get).

    직접 인스턴스화하지 않는다 — modality 별 서브클래스(GloveHumanHandReceiver 등)를
    통해 사용한다. `get(side)` 는 서브클래스가 구현해야 한다.
    """

    def __init__(
        self,
        glove_port: int = GLOVE_OSC_PORT,
        listen_ip: str = "0.0.0.0",
        target_ip: str = GLOVE_TARGET_IP,
        client_port: int = GLOVE_CLIENT_PORT,
    ):
        self._srv = SharedOscServer.get(glove_port, listen_ip)
        self._lock = threading.Lock()
        # 서브클래스가 자유롭게 필드를 채우는 side 별 상태 컨테이너.
        self._state: Dict[str, dict] = {"left": {}, "right": {}}
        self._connected: Dict[str, bool] = {"left": False, "right": False}

        self._target_ip = target_ip
        self._client_port = client_port
        self._udp_client: Optional[SimpleUDPClient] = None  # start() 에서 lazy 생성

        self._running = False
        self._heartbeat_thread: Optional[threading.Thread] = None

        self._srv.add_handler("/device/status/get", self._on_device_status)

    # ---------------------------------------------------------------- public
    def start(self) -> None:
        if self._udp_client is None:
            self._udp_client = SimpleUDPClient(self._target_ip, self._client_port)
        self._srv.start()
        if not self._running:
            self._running = True
            self._heartbeat_thread = threading.Thread(
                target=self._heartbeat_loop, daemon=True, name="GloveHeartbeat"
            )
            self._heartbeat_thread.start()

    def stop(self) -> None:
        self._running = False
        self._srv.stop()

    def connected(self, side: str) -> bool:
        with self._lock:
            return self._connected[side]

    # ----------------------------------------------------------- OSC handlers
    def _on_device_status(self, address, *args):
        if len(args) < 3:
            return
        with self._lock:
            self._connected["left"] = bool(args[1])
            self._connected["right"] = bool(args[2])

    def _heartbeat_loop(self):
        while self._running:
            try:
                if self._udp_client is not None:
                    self._udp_client.send_message("/device/status/get", "4")
            except Exception as e:
                _log.warning("[Glove] heartbeat 실패: %s", e)
            time.sleep(HEARTBEAT_INTERVAL_SEC)

    @staticmethod
    def _split(args):
        """`(side, x, y, z, ...)` 형태 OSC 인자에서 side 를 분리."""
        side = args[0]
        if isinstance(side, (list, tuple)):
            side = side[0]
        return side, args[1:]
