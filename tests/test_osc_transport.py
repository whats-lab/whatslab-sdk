"""SharedOscServer — 포트당 단일 pythonosc UDP 서버(get-or-create + refcount).

실제 소켓을 열지 않고 dispatcher.call_handlers_for_packet 으로 디스패치를 검증한다.
"""
from pythonosc.osc_message_builder import OscMessageBuilder

from whatslab.receiver.osc_transport import SharedOscServer


def _packet(address: str, *args) -> bytes:
    b = OscMessageBuilder(address=address)
    for a in args:
        b.add_arg(a)
    return b.build().dgram


def test_get_same_port_returns_same_instance():
    a = SharedOscServer.get(9990)
    b = SharedOscServer.get(9990)
    assert a is b


def test_get_different_port_returns_different_instance():
    a = SharedOscServer.get(9991)
    c = SharedOscServer.get(9992)
    assert a is not c


def test_two_handlers_each_receive_own_address():
    srv = SharedOscServer.get(9993)
    received_left = []
    received_right = []

    srv.add_handler("/left/quat/get", lambda address, *args: received_left.append(args))
    srv.add_handler("/right/quat/get", lambda address, *args: received_right.append(args))

    # 실제 UDP 소켓 없이 OSC 패킷을 dispatcher 에 직접 투입해 라우팅을 검증
    srv.dispatcher.call_handlers_for_packet(_packet("/left/quat/get", 1.0, 2.0), ("127.0.0.1", 0))
    srv.dispatcher.call_handlers_for_packet(_packet("/right/quat/get", 3.0), ("127.0.0.1", 0))

    assert received_left == [(1.0, 2.0)]
    assert received_right == [(3.0,)]
    assert received_left != received_right


def test_refcount_start_stop_keeps_server_alive_until_last_stop():
    srv = SharedOscServer.get(9994)

    srv.start()
    srv.start()
    assert srv.is_running

    srv.stop()
    assert srv.is_running  # 아직 하나의 start 가 살아있음

    srv.stop()
    assert not srv.is_running  # 마지막 stop 에서 종료
