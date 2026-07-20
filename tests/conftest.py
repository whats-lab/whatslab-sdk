"""테스트 공통 픽스처.

모델 프리셋(QuestModel/GloveModel)이 OSC 포트를 인자로 받지 않고 기본 포트를
쓰므로, 테스트마다 SharedOscServer 레지스트리를 비워 포트 싱글턴이 이전 테스트의
디스패처(누적 핸들러)를 재사용하지 않게 한다.
"""
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
