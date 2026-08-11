import os
import sys

import numpy as np
import pytest

pin = pytest.importorskip("pinocchio")
torch = pytest.importorskip("torch")
ort = pytest.importorskip("onnxruntime")
pytest.importorskip("onnxscript")

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))), "tools"))

from export_hand_net_onnx import OPSET, HandRetargetGraph  # noqa: E402

from whatslab.solvers.hand.net_retargeter import NetHandRetargeter  # noqa: E402

CONFIGS = ("orca_hand", "robotis_hx5_d20")


def _export(tmp_path, cfg, mirror=False):
    os.makedirs(str(tmp_path), exist_ok=True)
    r = NetHandRetargeter("left", cfg)
    r.net = r.net.float().eval()
    graph = HandRetargetGraph(r, mirror=mirror).eval()
    n = len(r.fk.joint_names)
    rng = np.random.default_rng(0)
    idx = [r.fk._idx_q[j] for j in r.fk.joint_names]
    lo = r.fk.model.lowerPositionLimit[idx]
    hi = r.fk.model.upperPositionLimit[idx]
    q = rng.uniform(lo, hi, (6, n)).astype(np.float32)
    path = str(tmp_path / ("%s.onnx" % cfg))
    torch.onnx.export(graph, (torch.as_tensor(q[:2]),), path, opset_version=OPSET,
                      input_names=["q_human"], output_names=["q_robot"],
                      dynamic_shapes={"q_human": {0: torch.export.Dim("batch")}})
    return r, path, q


@pytest.mark.parametrize("cfg", CONFIGS)
def test_onnx_matches_reference_path(tmp_path, cfg):
    r, path, q = _export(tmp_path, cfg)
    sess = ort.InferenceSession(path, providers=["CPUExecutionProvider"])
    got = sess.run(None, {"q_human": q})[0]
    ref = np.stack([r.compute({n: float(v) for n, v in zip(r.fk.joint_names, row)})
                    for row in q])
    assert np.abs(got - ref).max() < 1e-4


def test_batch_axis_is_dynamic(tmp_path):
    _, path, q = _export(tmp_path, "robotis_hx5_d20")
    sess = ort.InferenceSession(path, providers=["CPUExecutionProvider"])
    assert sess.get_inputs()[0].shape[0] == "batch"
    for b in (1, 3, 6):
        assert sess.run(None, {"q_human": q[:b]})[0].shape[0] == b


def test_output_respects_joint_limits(tmp_path):
    r, path, q = _export(tmp_path, "robotis_hx5_d20")
    sess = ort.InferenceSession(path, providers=["CPUExecutionProvider"])
    got = sess.run(None, {"q_human": q})[0]
    assert np.all(got >= r.lower - 1e-4)
    assert np.all(got <= r.upper + 1e-4)


def test_mirror_flag_negates_z_channels(tmp_path):
    _, plain, q = _export(tmp_path / "a", "robotis_hx5_d20", mirror=False)
    _, flipped, _ = _export(tmp_path / "b", "robotis_hx5_d20", mirror=True)
    a = ort.InferenceSession(plain, providers=["CPUExecutionProvider"])
    b = ort.InferenceSession(flipped, providers=["CPUExecutionProvider"])
    assert np.abs(a.run(None, {"q_human": q})[0]
                  - b.run(None, {"q_human": q})[0]).max() > 1e-6
