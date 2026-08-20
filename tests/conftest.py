import pytest

from whatslab.receiver import osc_transport


FINGERS = ("thumb", "index", "middle", "ring", "pinky")


def require_sensor_urdf(*configs, sides=("left", "right")):
    pin = pytest.importorskip("pinocchio")
    from whatslab.solvers.hand.uni_retargeter import UniRetargeter

    for name in configs or ("orca_hand",):
        for side in sides:
            urdf = UniRetargeter(side, name).urdf_path
            if urdf is None:
                pytest.skip(f"{name}/{side}: URDF 없음")
            m = pin.buildModelFromUrdf(urdf)
            want = [f"{side}_sensor_dorsum"] + [
                f"{side}_sensor_{f}_distal" for f in FINGERS]
            missing = [n for n in want
                       if not m.existFrame(n, pin.FrameType.BODY)]
            if missing:
                pytest.skip(f"{name}/{side}: 센서 프레임 필요 {missing}")


@pytest.fixture(autouse=True)
def _reset_osc_registry():
    osc_transport._registry.clear()
    yield
    for srv in list(osc_transport._registry.values()):
        try:
            while srv.is_running:
                srv.stop()
        except Exception:
            pass
    osc_transport._registry.clear()
