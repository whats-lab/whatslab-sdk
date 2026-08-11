import numpy as np
import pytest

pin = pytest.importorskip("pinocchio")

from whatslab.solvers.hand.human_fk import FINGERS, HumanHandFK
from whatslab.solvers.hand.keyvector import (HandKeyvector, chain_weights,
                                            human_chains)


def _human_kv(side):
    fk = HumanHandFK(side)
    return fk, HandKeyvector(fk.model, fk.data, human_chains(fk),
                             side + "_sensor_dorsum")


def _walk(segs, frac=0.5):
    k, t = chain_weights(segs, frac)
    return float(np.sum(segs[:k]) + segs[k] * t)


def test_chain_weights_lands_on_requested_arc_fraction():
    for segs in ([1.0, 1.0], [1.0, 1.0, 2.0], [3.0, 1.0], [0.5, 0.0, 2.5]):
        segs = np.asarray(segs, dtype=float)
        for frac in (0.25, 0.5, 0.75):
            assert _walk(segs, frac) == pytest.approx(frac * segs.sum(), rel=1e-9)


def test_chain_weights_index_stays_in_range():
    for segs in ([1.0, 1.0], [1.0, 1.0, 2.0], [3.0, 1.0]):
        k, t = chain_weights(segs)
        assert 0 <= k < len(segs)
        assert 0.0 <= t <= 1.0


def test_chain_weights_rejects_degenerate_chain():
    with pytest.raises(ValueError):
        chain_weights([0.0, 0.0])
    with pytest.raises(ValueError):
        chain_weights([])


def test_encode_shape_and_lref_normalisation():
    fk, kv = _human_kv("left")
    x = kv.encode(pin.neutral(fk.model))
    assert x.shape == (5, 6)
    assert np.linalg.norm(x[FINGERS.index("middle"), :3]) == pytest.approx(1.0, abs=1e-9)


def test_prox_sits_at_half_arc_length_and_is_pose_independent():
    fk, kv = _human_kv("left")
    rng = np.random.default_rng(0)
    for _ in range(3):
        q = pin.neutral(fk.model)
        for n in fk.joint_names:
            iq = fk._idx_q[n]
            q[iq] = rng.uniform(fk.model.lowerPositionLimit[iq],
                                fk.model.upperPositionLimit[iq])
        pts = kv.points(q)
        for f in FINGERS:
            k, t = kv.mid[f]
            segs = np.linalg.norm(np.diff(pts[f], axis=0), axis=1)
            walked = float(segs[:k].sum() + segs[k] * t)
            assert walked == pytest.approx(0.5 * float(segs.sum()), rel=1e-9)


def test_encode_is_invariant_to_dorsum_origin_choice():
    fk, kv = _human_kv("left")
    q = pin.neutral(fk.model)
    pts = kv.points(q)
    manual = kv.rot.T @ (pts["index"][-1] - kv.origin) / kv.l_ref
    assert np.abs(kv.encode(q)[FINGERS.index("index"), :3] - manual).max() < 1e-12
