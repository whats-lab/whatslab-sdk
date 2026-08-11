import os

import numpy as np
import pytest

pin = pytest.importorskip("pinocchio")

from whatslab.solvers.hand.human_fk import palm_frame_from_fingers, FINGERS, HumanHandFK
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


def test_encode_shape_and_lref_is_palm_frame_middle_length():
    fk, kv = _human_kv("left")
    q = pin.neutral(fk.model)
    x = kv.encode(q)
    assert x.shape == (5, 6)
    pts = kv.points(q)
    palm_o, rot = palm_frame_from_fingers(pts)
    assert kv.l_ref == pytest.approx(
        float(np.linalg.norm(rot.T @ (pts["middle"][-1] - palm_o))), abs=1e-12)
    tip = float(np.linalg.norm(pts["middle"][-1] - kv.origin))
    assert np.linalg.norm(x[FINGERS.index("middle"), :3]) == pytest.approx(
        tip / kv.l_ref, abs=1e-9)


def test_lref_scale_matches_bench_metric_scale_on_both_hands():
    import sys
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
        os.path.abspath(__file__))), "tools"))
    from bench_hand_retarget import net_probe

    from whatslab.solvers.hand.net_retargeter import NetHandRetargeter
    fk, hkv = _human_kv("left")
    for cfg in ("robotis_hx5_d20", "orca_hand"):
        r = NetHandRetargeter("left", cfg)
        r_len = net_probe(r)[2]()[2]
        hp = r.fk.points({n: 0.0 for n in r.fk.joint_names})
        o, rot = palm_frame_from_fingers({f: hp[f] for f in FINGERS})
        h_len = float(np.linalg.norm(rot.T @ (hp["middle"][-1] - o)))
        assert r.kv.l_ref / r.hkv.l_ref == pytest.approx(r_len / h_len, rel=1e-9)


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


def test_four_finger_subset_encodes_without_pinky():
    fk = HumanHandFK("left")
    fours = [f for f in FINGERS if f != "pinky"]
    kv4 = HandKeyvector(fk.model, fk.data, human_chains(fk, fours),
                        "left_sensor_dorsum")
    assert kv4.fingers == fours
    x = kv4.encode(pin.neutral(fk.model))
    assert x.shape == (4, 6)
    kv5 = HandKeyvector(fk.model, fk.data, human_chains(fk), "left_sensor_dorsum")
    assert kv5.encode(pin.neutral(fk.model)).shape == (5, 6)
    j = kv4.jacobian(pin.neutral(fk.model),
                     [fk.model.joints[fk.model.getJointId(n)].idx_v
                      for n in fk.joint_names])
    assert j.shape[:2] == (4, 6)


def test_palm_frame_needs_three_non_thumb_fingers():
    fk = HumanHandFK("left")
    with pytest.raises(ValueError, match="3개 이상"):
        HandKeyvector(fk.model, fk.data,
                      human_chains(fk, ["thumb", "index", "middle"]),
                      "left_sensor_dorsum")
