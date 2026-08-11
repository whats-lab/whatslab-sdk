import numpy as np
import pytest

torch = pytest.importorskip("torch")

from whatslab.solvers.hand.net_losses import (AffineHandNet, ResidualAffine,
                                             chamfer_both,
                                             chamfer_partial, distance_loss,
                                             motion_loss_global, motion_loss_local,
                                             pinch_loss, soft_pinch_loss)


def _kv(vals):
    return torch.tensor(np.asarray(vals, dtype=float))


def test_residual_affine_starts_as_identity():
    aff = ResidualAffine(5).double()
    x = torch.randn(4, 5, 6, dtype=torch.float64)
    assert torch.equal(aff(x), x)


def test_affine_wrapper_passes_through_at_init():
    inner = torch.nn.Linear(6, 3).double()

    class Wrap(torch.nn.Module):
        def forward(self, x):
            return inner(x[:, 0])

    net = AffineHandNet(Wrap(), 5).double()
    x = torch.randn(2, 5, 6, dtype=torch.float64)
    assert torch.allclose(net(x), inner(x[:, 0]))


def test_partial_chamfer_ignores_uncovered_target_region():
    a = _kv([[[0.0, 0.0, 0.0, 0, 0, 0]]]).reshape(1, 1, 6)
    b = _kv([[[0.0, 0.0, 0.0, 0, 0, 0]], [[9.0, 0.0, 0.0, 0, 0, 0]]]).reshape(2, 1, 6)
    both = chamfer_both(a[:, 0, :3], b[:, 0, :3])
    part = chamfer_partial(a[:, 0, :3], b[:, 0, :3])
    assert float(part) == pytest.approx(0.0, abs=1e-12)
    assert float(both) > 39.0


def test_distance_loss_is_zero_when_pairwise_distances_match():
    x = torch.randn(3, 5, 6, dtype=torch.float64)
    assert float(distance_loss(x, x.clone())) == pytest.approx(0.0, abs=1e-18)


def test_distance_loss_penalises_uniform_scaling():
    x = torch.randn(3, 5, 6, dtype=torch.float64)
    assert float(distance_loss(x, x * 2.0)) > 0.0


def test_global_motion_loss_is_minimal_when_aligned():
    d = torch.randn(4, 5, 6, dtype=torch.float64)
    assert float(motion_loss_global(d, d)) == pytest.approx(-1.0, abs=1e-9)
    assert float(motion_loss_global(d, -d)) == pytest.approx(1.0, abs=1e-9)


def test_local_motion_loss_is_rotation_invariant():
    torch.manual_seed(0)
    a = torch.randn(4, 5, 6, dtype=torch.float64)
    b = torch.randn(4, 5, 6, dtype=torch.float64)
    q, _ = torch.linalg.qr(torch.randn(6, 6, dtype=torch.float64))
    ra = a @ q.T
    rb = b @ q.T
    assert float(motion_loss_local(a, ra, b, rb)) == pytest.approx(0.0, abs=1e-18)
    assert float(motion_loss_global(a, ra)) > -0.99


def test_pinch_loss_only_fires_on_close_human_pairs():
    x = torch.zeros(2, 5, 6, dtype=torch.float64)
    x[0, 1, 0] = 10.0
    y = torch.zeros(2, 5, 6, dtype=torch.float64)
    y[:, 1, 0] = 1.0
    assert float(pinch_loss(x, y, threshold=1.0)) == pytest.approx(1.0, abs=1e-9)
    assert float(pinch_loss(x, y, threshold=0.0)) == 0.0


def test_soft_pinch_uses_nearest_candidates_only():
    y = torch.zeros(1, 5, 6, dtype=torch.float64)
    cand = torch.zeros(3, 5, 6, dtype=torch.float64)
    cand[1, 0, 0] = 5.0
    cand[2, 0, 0] = 9.0
    assert float(soft_pinch_loss(y, cand, top_k=1)) == pytest.approx(0.0, abs=1e-12)
    assert float(soft_pinch_loss(y, cand, top_k=3, tau=1e6)) > 0.0
