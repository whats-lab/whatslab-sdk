"""GloveReceiverBase + GloveHumanHandReceiver — 공유 SharedOscServer 위에서 동작 검증.

실제 UDP 소켓 없이 dispatcher.call_handlers_for_packet 으로 OSC 패킷을 주입한다
(test_osc_transport.py / test_quest_receiver.py 와 동일한 접근).

구 AirGloveReceiver 의 파싱 로직(parse_aga_raw)을 그대로 옮겼으므로, 동일한
72-float 원시 패킷에 대해 동일한 17×4 손가락 쿼터니언을 산출해야 한다(동등성 검증).
"""
import numpy as np
from pythonosc.osc_message_builder import OscMessageBuilder

from whatslab.core.interfaces import Receiver
from whatslab.core.types import InputSample
from whatslab.receiver import GloveHumanHandReceiver
from whatslab.receiver.glove_human_hand import parse_aga_raw, wrist_to_canonical


def _packet(address: str, *args) -> bytes:
    b = OscMessageBuilder(address=address)
    for a in args:
        b.add_arg(a)
    return b.build().dgram


def _send(disp, address, *args):
    disp.call_handlers_for_packet(_packet(address, *args), ("127.0.0.1", 0))


def test_import_without_pyosc():
    # 모듈 import 만으로 python-osc 를 요구하면 안 됨 (여기까지 왔으면 통과)
    assert GloveHumanHandReceiver is not None


def test_receivers_conform_protocol():
    r = GloveHumanHandReceiver(glove_port=4140)
    assert isinstance(r, Receiver), "GloveHumanHandReceiver 이 Receiver 프로토콜 불충족"


def test_get_returns_input_sample_neutral():
    r = GloveHumanHandReceiver(glove_port=4141)
    s = r.get("right")
    assert isinstance(s, InputSample)
    assert s.tracked is False           # 수신 전
    assert s.hand is not None
    arr = s.hand.to_sensor_array()
    assert arr.shape == (17, 4)
    assert np.allclose(arr[:, 3], 1.0)  # 항등 쿼터니언
    assert len(s.hand.joint_rot) == 16


def test_glove_no_controller():
    # 글러브: controller 없음, hand 는 있음(회전만)
    g = GloveHumanHandReceiver(glove_port=4142).get("left")
    assert g.controller is None and g.hand is not None


def _make_raw(seed: int) -> np.ndarray:
    rng = np.random.default_rng(seed)
    return rng.uniform(-1.0, 1.0, size=72).astype(np.float32)


def test_both_sides_independent():
    recv = GloveHumanHandReceiver(glove_port=4143)
    disp = recv._srv.dispatcher

    left_raw = _make_raw(1)
    right_raw = _make_raw(2)
    _send(disp, "/left/quat/get", "1", *left_raw.tolist())
    _send(disp, "/right/quat/get", "1", *right_raw.tolist())

    left = recv.get("left")
    right = recv.get("right")
    assert not np.allclose(left.hand.to_sensor_array(), right.hand.to_sensor_array())


def test_equivalent_to_old_parse_aga_raw():
    """AGA 72-float 패킷 주입 → get(side) 결과가 구 AirGloveReceiver 파싱과 동일해야 한다."""
    recv = GloveHumanHandReceiver(glove_port=4144)
    disp = recv._srv.dispatcher

    raw = _make_raw(42)
    _send(disp, "/right/quat/get", "1", *raw.tolist())

    sample = recv.get("right")
    assert sample.tracked

    expected_quats = parse_aga_raw(raw)  # 구 AirGloveReceiver.parse_aga_raw 와 동일 함수

    got = sample.hand.to_sensor_array()
    # 손가락 관절([1:])은 raw 그대로, 손목([0])만 정준 변환된다.
    assert np.allclose(got[1:], expected_quats[1:])
    assert np.allclose(sample.hand.wrist.quat, wrist_to_canonical(expected_quats[0]))
    # 손목 회전이 실제로 변환됐음(raw 와 다름) — pass-through 아님을 검증.
    assert not np.allclose(sample.hand.wrist.quat, expected_quats[0])
    # wrist pos 는 글러브 소스에서 신뢰 대상이 아님(회전만 제공) — pos 는 0 그대로.
    assert np.allclose(sample.hand.wrist.pos, np.zeros(3))


def test_two_glove_receivers_share_same_port_server():
    """GloveReceiverBase 는 SharedOscServer.get(port) 로 서버를 공유한다."""
    a = GloveHumanHandReceiver(glove_port=4145)
    b = GloveHumanHandReceiver(glove_port=4145)
    assert a._srv is b._srv


def test_device_status_sets_connected():
    recv = GloveHumanHandReceiver(glove_port=4146)
    disp = recv._srv.dispatcher
    assert not recv.connected("left")
    assert not recv.connected("right")

    _send(disp, "/device/status/get", "4", True, False)
    assert recv.connected("left")
    assert not recv.connected("right")
