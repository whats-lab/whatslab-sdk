import socket

import pytest

from whatslab.core.constants import FINGER_NAMES
from whatslab.receiver.webxr import ws_server


def require_sensor_urdf(*configs, sides=("left", "right")):
    pin = pytest.importorskip("pinocchio")
    import os

    from whatslab.paths import models_root
    urdfs = {"orca_hand": "orca_hand/urdf/{side}.urdf"}

    for name in configs or ("orca_hand",):
        for side in sides:
            urdf = os.path.join(models_root(), urdfs[name].format(side=side))
            if not os.path.isfile(urdf):
                pytest.skip(f"{name}/{side}: URDF 없음")
            m = pin.buildModelFromUrdf(urdf)
            want = [f"{side}_sensor_dorsum"] + [
                f"{side}_sensor_{f}_distal" for f in FINGER_NAMES]
            missing = [n for n in want
                       if not m.existFrame(n, pin.FrameType.BODY)]
            if missing:
                pytest.skip(f"{name}/{side}: 센서 프레임 필요 {missing}")


@pytest.fixture
def free_port():
    """지금 비어 있는 TCP 포트 — 서버를 공유하려면 고정 포트가 필요하다(port=0 은 공유 안 함)."""
    with socket.socket() as probe:
        probe.bind(("0.0.0.0", 0))
        return probe.getsockname()[1]


@pytest.fixture(autouse=True)
def _stop_leaked_webxr_servers():
    """실패한 테스트가 고정 포트 서버를 켠 채 남겨도 다음 테스트에 번지지 않게 한다."""
    yield
    for srv in list(ws_server._registry.values()):
        while srv.is_running:
            srv.stop()
