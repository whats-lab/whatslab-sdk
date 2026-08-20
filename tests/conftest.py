import pytest

from whatslab.receiver import osc_transport


def require_sensor_urdf(*configs, sides=("left", "right")):
    pytest.importorskip("pinocchio")
    from whatslab.solvers.hand.hand_configs import CONFIG_REGISTRY

    for name in configs or ("orca_hand",):
        for side in sides:
            try:
                CONFIG_REGISTRY[name]()._get_fingers(side)
            except (FileNotFoundError, ValueError) as e:
                pytest.skip(f"{name}/{side}: 센서 프레임 URDF 필요 ({e})")


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
