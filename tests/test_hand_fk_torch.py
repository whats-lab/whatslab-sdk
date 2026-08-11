import numpy as np
import pytest

pin = pytest.importorskip("pinocchio")
torch = pytest.importorskip("torch")

from whatslab.solvers.hand.fk_torch import KeyvectorFK
from whatslab.solvers.hand.net_retargeter import NetHandRetargeter


def _fk(side="left", cfg="robotis_hx5_d20"):
    r = NetHandRetargeter(side, cfg)
    return r, KeyvectorFK(r.kv, r._iq, r._iv, pin.neutral(r.model))


def test_index_mismatch_is_rejected():
    r, _ = _fk()
    with pytest.raises(ValueError):
        KeyvectorFK(r.kv, r._iq, r._iv[:-1], pin.neutral(r.model))


def test_forward_matches_numpy_encode():
    r, fk = _fk()
    q = torch.zeros(1, len(r.joint_names), dtype=torch.float64)
    got = fk(q).detach().numpy()[0]
    want = r.kv.encode(pin.neutral(r.model))
    assert np.abs(got - want).max() < 1e-12


def test_gradient_matches_finite_difference():
    r, fk = _fk()
    n = len(r.joint_names)
    rng = np.random.default_rng(0)
    q0 = rng.uniform(-0.2, 0.2, (2, n))
    w = rng.normal(size=(2, 5, 6))
    q = torch.tensor(q0, dtype=torch.float64, requires_grad=True)
    (fk(q) * torch.tensor(w)).sum().backward()
    got = q.grad.numpy()
    eps = 1e-6
    num = np.zeros_like(q0)
    for b in range(q0.shape[0]):
        for j in range(n):
            d = np.zeros_like(q0)
            d[b, j] = eps
            hi = (fk(torch.tensor(q0 + d)).detach().numpy() * w).sum()
            lo = (fk(torch.tensor(q0 - d)).detach().numpy() * w).sum()
            num[b, j] = (hi - lo) / (2.0 * eps)
    assert np.abs(got - num).max() < 1e-5
