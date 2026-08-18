import numpy as np
from pythonosc.osc_message_builder import OscMessageBuilder

from whatslab.core.interfaces import Receiver
from whatslab.core.types import InputSample
from whatslab.receiver import GloveHumanHandReceiver
from whatslab.receiver.glove.human_hand import parse_aga_raw, wrist_to_canonical


def _packet(address: str, *args) -> bytes:
    b = OscMessageBuilder(address=address)
    for a in args:
        b.add_arg(a)
    return b.build().dgram


def _send(disp, address, *args):
    disp.call_handlers_for_packet(_packet(address, *args), ("127.0.0.1", 0))


def test_import_without_pyosc():
    assert GloveHumanHandReceiver is not None


def test_receivers_conform_protocol():
    r = GloveHumanHandReceiver(glove_port=4140)
    assert isinstance(r, Receiver), "GloveHumanHandReceiver 이 Receiver 프로토콜 불충족"


def test_get_returns_input_sample_neutral():
    r = GloveHumanHandReceiver(glove_port=4141)
    s = r.get("right")
    assert isinstance(s, InputSample)
    assert s.tracked is False
    assert s.hand is not None
    arr = s.hand.to_sensor_array()
    assert arr.shape == (17, 4)
    assert np.allclose(arr[:, 3], 1.0)
    assert len(s.hand.joint_rot) == 16


def test_glove_no_controller():
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
    recv = GloveHumanHandReceiver(glove_port=4144)
    disp = recv._srv.dispatcher

    raw = _make_raw(42)
    _send(disp, "/right/quat/get", "1", *raw.tolist())

    sample = recv.get("right")
    assert sample.tracked

    expected_quats = parse_aga_raw(raw)

    got = sample.hand.to_sensor_array()
    assert np.allclose(got[1:], expected_quats[1:])
    assert np.allclose(sample.hand.wrist.quat, wrist_to_canonical(expected_quats[0]))
    assert not np.allclose(sample.hand.wrist.quat, expected_quats[0])
    assert np.allclose(sample.hand.wrist.pos, np.zeros(3))


def test_two_glove_receivers_share_same_port_server():
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
