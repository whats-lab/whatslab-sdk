import socket

import pytest


@pytest.fixture
def free_port():
    with socket.socket() as probe:
        probe.bind(("0.0.0.0", 0))
        return probe.getsockname()[1]



@pytest.fixture
def running(request):
    def start(obj):
        obj.start()
        request.addfinalizer(obj.stop)
        return obj

    return start
