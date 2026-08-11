import numpy as np
import pytest

pin = pytest.importorskip("pinocchio")
torch = pytest.importorskip("torch")

from whatslab.solvers.hand.net_retargeter import NetHandRetargeter
from whatslab.solvers.hand.torch_fk import TorchKeyvectorFK

CONFIGS = ("orca_hand", "robotis_hx5_d20")


def _rig(cfg, side="left"):
    r = NetHandRetargeter(side, cfg)
    return r, TorchKeyvectorFK(r.kv, r._iq, r.joint_names)


def _random_q(r, n, seed=0):
    rng = np.random.default_rng(seed)
    return rng.uniform(r.lower, r.upper, (n, len(r.joint_names)))


@pytest.mark.parametrize("cfg", CONFIGS)
def test_matches_pinocchio_encode(cfg):
    r, tfk = _rig(cfg)
    q = _random_q(r, 8)
    ref = []
    for row in q:
        full = pin.neutral(r.model)
        full[r._iq] = row
        ref.append(r.kv.encode(full))
    got = tfk(torch.as_tensor(q, dtype=torch.float64)).detach().numpy()
    assert np.abs(got - np.stack(ref)).max() < 1e-12


@pytest.mark.parametrize("cfg", CONFIGS)
def test_gradient_matches_pinocchio_jacobian(cfg):
    r, tfk = _rig(cfg)
    q0 = _random_q(r, 4, seed=1)
    rng = np.random.default_rng(2)
    w = rng.normal(size=(4, 5, 6))

    q = torch.tensor(q0, dtype=torch.float64, requires_grad=True)
    (tfk(q) * torch.as_tensor(w, dtype=torch.float64)).sum().backward()
    got = q.grad.numpy().copy()

    ref = np.empty_like(got)
    for i, row in enumerate(q0):
        full = pin.neutral(r.model)
        full[r._iq] = row
        jac = r.kv.jacobian(full, r._iv)
        ref[i] = np.einsum("fkn,fk->n", jac, w[i])
    assert np.abs(got - ref).max() < 1e-9


def test_float32_stays_within_training_tolerance():
    r, _ = _rig("robotis_hx5_d20")
    q = _random_q(r, 16, seed=3)
    f64 = TorchKeyvectorFK(r.kv, r._iq, r.joint_names, dtype=torch.float64)
    f32 = TorchKeyvectorFK(r.kv, r._iq, r.joint_names, dtype=torch.float32)
    a = f64(torch.as_tensor(q, dtype=torch.float64)).detach().numpy()
    b = f32(torch.as_tensor(q, dtype=torch.float32)).detach().numpy()
    assert np.abs(a - b).max() < 1e-5


def test_batch_is_independent():
    r, tfk = _rig("orca_hand")
    q = _random_q(r, 5, seed=4)
    full = tfk(torch.as_tensor(q, dtype=torch.float64)).detach().numpy()
    for i in range(q.shape[0]):
        one = tfk(torch.as_tensor(q[i:i + 1], dtype=torch.float64)).detach().numpy()
        assert np.abs(full[i] - one[0]).max() < 1e-14
