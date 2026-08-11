import numpy as np
from pythonosc.osc_message_builder import OscMessageBuilder

from whatslab.core.interfaces import Receiver
from whatslab.core.types import InputSample
from whatslab.receiver import GloveRobotHandReceiver
from whatslab.receiver.glove import GloveHumanAnglesReceiver


def _packet(address: str, *args) -> bytes:
    b = OscMessageBuilder(address=address)
    for a in args:
        b.add_arg(a)
    return b.build().dgram


def _send(disp, address, *args):
    disp.call_handlers_for_packet(_packet(address, *args), ("127.0.0.1", 0))


def test_receivers_conform_protocol():
    for cls, port in ((GloveRobotHandReceiver, 4140),
                      (GloveHumanAnglesReceiver, 4147)):
        r = cls(glove_port=port)
        assert isinstance(r, Receiver), "%s 이 Receiver 프로토콜 불충족" % cls.__name__


def test_get_returns_input_sample_untracked_before_any_message():
    r = GloveHumanAnglesReceiver(glove_port=4141)
    s = r.get("right")
    assert isinstance(s, InputSample)
    assert s.tracked is False
    assert s.controller is None
    assert s.hand is not None
    assert s.hand.joint_angles == {}
    assert s.hand.wrist is None


def test_joint_angles_arrive_as_name_value_pairs():
    recv = GloveHumanAnglesReceiver(glove_port=4148)
    _send(recv._srv.dispatcher, "/right/joint_angles/get", "20",
          "right_index_mcp_flex", 0.25, "right_thumb_ip_flex", -0.5)
    s = recv.get("right")
    assert s.tracked
    assert s.hand.joint_angles == {"right_index_mcp_flex": 0.25,
                                   "right_thumb_ip_flex": -0.5}


def test_wrist_is_committed_without_frame_conversion():
    recv = GloveHumanAnglesReceiver(glove_port=4149)
    wire = [0.5, 0.5, 0.5, -0.5]
    _send(recv._srv.dispatcher, "/right/wrist/get", "19",
          *(float(v) for v in wire))
    s = recv.get("right")
    assert s.hand.wrist is not None
    assert np.allclose(s.hand.wrist.quat, wire)


def test_both_sides_independent():
    recv = GloveHumanAnglesReceiver(glove_port=4143)
    disp = recv._srv.dispatcher
    _send(disp, "/left/joint_angles/get", "20", "left_index_mcp_flex", 0.1)
    _send(disp, "/right/joint_angles/get", "20", "right_index_mcp_flex", 0.9)
    assert recv.get("left").hand.joint_angles != recv.get("right").hand.joint_angles


def test_two_glove_receivers_share_same_port_server():
    a = GloveHumanAnglesReceiver(glove_port=4145)
    b = GloveHumanAnglesReceiver(glove_port=4145)
    assert a._srv is b._srv


def test_device_status_sets_connected():
    recv = GloveHumanAnglesReceiver(glove_port=4146)
    disp = recv._srv.dispatcher
    assert not recv.connected("left")
    assert not recv.connected("right")

    _send(disp, "/device/status/get", "4", True, False)
    assert recv.connected("left")
    assert not recv.connected("right")
