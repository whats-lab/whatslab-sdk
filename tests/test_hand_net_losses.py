import numpy as np
import pytest

torch = pytest.importorskip("torch")

from whatslab.solvers.hand.net_losses import (bone_loss, chamfer_both,
                                             coverage_loss,
                                             motion_loss_global, pinch_loss,
                                             position_loss, posture_loss,
                                             saturation_loss, unit_to_joint)


def _kv(vals):
    return torch.tensor(np.asarray(vals, dtype=float))


def test_unit_to_joint_matches_plain_linear_map_at_margin_one():
    lo = np.array([-1.0, 0.0])
    hi = np.array([1.0, 2.0])
    u = np.array([-1.0, 0.5])
    want = lo + (u + 1.0) * 0.5 * (hi - lo)
    assert np.allclose(unit_to_joint(u, lo, hi, 1.0), want)


def test_unit_to_joint_reaches_limits_before_tanh_saturates():
    lo = np.array([-1.0])
    hi = np.array([1.0])
    assert unit_to_joint(np.array([0.8]), lo, hi, 1.25)[0] == pytest.approx(1.0)
    assert unit_to_joint(np.array([1.0]), lo, hi, 1.25)[0] <= hi[0] + 1e-2


def test_saturation_loss_only_penalises_beyond_knee():
    assert float(saturation_loss(torch.zeros(4, 3, dtype=torch.float64))) == 0.0
    assert float(saturation_loss(torch.full((4, 3), 0.89,
                                            dtype=torch.float64))) == 0.0
    assert float(saturation_loss(torch.full((4, 3), 1.0,
                                            dtype=torch.float64))) > 0.0


def test_posture_loss_ignores_spread_and_penalises_bias():
    centred = torch.tensor([[-1.0], [1.0]], dtype=torch.float64)
    assert float(posture_loss(centred)) == pytest.approx(0.0, abs=1e-18)
    biased = torch.tensor([[0.5], [0.5]], dtype=torch.float64)
    assert float(posture_loss(biased)) == pytest.approx(0.25)


def test_position_loss_is_zero_on_match_and_grows_with_offset():
    x = torch.randn(3, 5, 6, dtype=torch.float64)
    assert float(position_loss(x, x.clone())) == pytest.approx(0.0, abs=1e-18)
    off = x.clone()
    off[:, :, 0] += 0.1
    assert float(position_loss(x, off)) == pytest.approx(0.01, abs=1e-9)


def test_pinch_loss_only_fires_on_close_human_pairs():
    x = torch.zeros(2, 5, 6, dtype=torch.float64)
    x[0, 1, 0] = 10.0
    y = torch.zeros(2, 5, 6, dtype=torch.float64)
    y[:, 1, 0] = 1.0
    assert float(pinch_loss(x, y, threshold=1.0)) == pytest.approx(1.0, abs=1e-9)
    assert float(pinch_loss(x, y, threshold=0.0)) == 0.0


def test_global_motion_loss_is_minimal_when_displacements_align():
    torch.manual_seed(0)
    dx = torch.randn(4, 5, 6, dtype=torch.float64)
    assert float(motion_loss_global(dx, dx.clone())) == pytest.approx(-1.0, abs=1e-9)
    assert float(motion_loss_global(dx, -dx)) == pytest.approx(1.0, abs=1e-9)


def test_bone_loss_is_zero_on_matching_intra_finger_direction():
    torch.manual_seed(0)
    x = torch.randn(3, 5, 6, dtype=torch.float64)
    assert float(bone_loss(x, x.clone())) == pytest.approx(0.0, abs=1e-12)
    flipped = x.clone()
    flipped[..., :3] = x[..., 3:]
    flipped[..., 3:] = x[..., :3]
    assert float(bone_loss(x, flipped)) > 1.0


def test_coverage_penalises_collapse():
    torch.manual_seed(0)
    bank = torch.randn(200, 5, 6, dtype=torch.float64)
    spread = torch.randn(32, 5, 6, dtype=torch.float64)
    collapsed = bank[0].unsqueeze(0).repeat(32, 1, 1)
    assert float(coverage_loss(collapsed, bank)) > float(coverage_loss(spread, bank))
