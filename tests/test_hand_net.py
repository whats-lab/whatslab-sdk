import numpy as np
import pytest

pin = pytest.importorskip("pinocchio")
torch = pytest.importorskip("torch")

from whatslab.solvers.hand.keyvector import MIRROR_Z
from whatslab.solvers.hand.net_retargeter import HandNet, NetHandRetargeter

CONFIGS = ("orca_hand", "robotis_hx5_d20")


def test_handnet_output_is_bounded_and_correctly_shaped():
    net = HandNet(6, [4, 3, 3, 3, 4]).double()
    y = net(torch.randn(3, 5, 6, dtype=torch.float64) * 50.0)
    assert y.shape == (3, 17)
    assert float(y.detach().abs().max()) <= 1.0


@pytest.mark.parametrize("cfg", CONFIGS)
def test_output_respects_joint_limits(cfg):
    r = NetHandRetargeter("left", cfg)
    zero = {n: 0.0 for n in r.human_joint_names}
    q = r.compute(zero)
    assert q.shape == (len(r.joint_names),)
    assert np.all(q >= r.lower - 1e-9)
    assert np.all(q <= r.upper + 1e-9)


@pytest.mark.parametrize("cfg", CONFIGS)
def test_joint_names_match_column_count(cfg):
    r = NetHandRetargeter("left", cfg)
    assert len(r.joint_names) == len(r._iq) == len(r._iv)
    assert len(r.joint_names) == sum(r.net.joint_counts)
    assert len(set(r.joint_names)) == len(r.joint_names)


def test_compute_is_deterministic():
    r = NetHandRetargeter("left", "robotis_hx5_d20")
    zero = {n: 0.0 for n in r.human_joint_names}
    a = r.compute(zero)
    r.reset()
    assert np.abs(a - r.compute(zero)).max() == 0.0


def test_mirror_flag_negates_z_channels():
    r = NetHandRetargeter("left", "robotis_hx5_d20", mirror_to="right")
    plain = NetHandRetargeter("left", "robotis_hx5_d20")
    zero = {n: 0.0 for n in r.human_joint_names}
    assert np.abs(r.encode_human(zero) - plain.encode_human(zero) * MIRROR_Z).max() < 1e-12


def test_checkpoint_roundtrip(tmp_path):
    r = NetHandRetargeter("left", "robotis_hx5_d20")
    zero = {n: 0.0 for n in r.human_joint_names}
    before = r.compute(zero)
    path = tmp_path / "net.pt"
    torch.save({"net": r.state_dict()}, path)
    other = NetHandRetargeter("left", "robotis_hx5_d20", checkpoint=str(path))
    assert np.abs(before - other.compute(zero)).max() == 0.0
