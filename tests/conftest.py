import socket

import pytest


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
                f"{side}_sensor_{f}_distal" for f in ("thumb", "index", "middle", "ring", "pinky")]
            missing = [n for n in want
                       if not m.existFrame(n, pin.FrameType.BODY)]
            if missing:
                pytest.skip(f"{name}/{side}: 센서 프레임 필요 {missing}")


@pytest.fixture
def free_port():
    with socket.socket() as probe:
        probe.bind(("0.0.0.0", 0))
        return probe.getsockname()[1]

