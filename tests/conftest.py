import pytest

from whatslab.receiver import osc_transport


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
