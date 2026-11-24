import os
import subprocess
import sys

import numpy as np
import pytest

from whatslab.retarget.geort import GeortEngine, load_prompt_model
from whatslab.retarget.geort.preprocess import PALM_MCPS, canonical_points

torch = pytest.importorskip("torch")
pytest.importorskip("trimesh")
dexhand = pytest.importorskip("dexhand_description")

from whatslab.retarget.geort.train import cli  # noqa: E402
from whatslab.retarget.geort.train.hand import (  # noqa: E402
    HandKinematics,
    config_names,
    read_hand,
    urdf_path,
)

HANDS = ("inspire_right", "leap_right")
SMALL = ["--steps", "3", "--batch", "32", "--width", "32", "--latent", "16", "--prompt-dim", "8",
         "--collision-batch", "8", "--device", "cpu"]


def needs_urdf(hand):
    if not urdf_path(read_hand(hand)).is_file():
        pytest.skip(f"dexhand-description has no URDF for {hand}")


def human_points(n, rng):
    roots = {1: (0.025, 0.02), 5: (-0.02, 0.09), 9: (0.0, 0.095), 13: (0.018, 0.09),
             17: (0.035, 0.08)}
    points = np.zeros((n, 21, 3))
    for root, (x, z) in roots.items():
        bend = rng.uniform(0.0, 1.4, size=(n, 1))
        for k in range(4):
            reach = 0.025 * k
            points[:, root + k, 0] = x
            points[:, root + k, 1] = -reach * np.sin(bend[:, 0] * k / 3)
            points[:, root + k, 2] = -(z + reach * np.cos(bend[:, 0] * k / 3))
    return points + rng.normal(0.0, 0.001, size=points.shape) * (np.arange(21) > 0)[:, None]


@pytest.fixture
def human(tmp_path):
    rng = np.random.default_rng(0)
    raw = human_points(300, rng)
    template = raw[0, list(PALM_MCPS)]
    size = float(np.linalg.norm(template, axis=-1).mean())
    points = canonical_points(raw, template, size)
    path = tmp_path / "human.npz"
    np.savez(path, points=points.astype(np.float32), source=np.zeros(len(points), np.int8),
             template=template, size=np.array(size))
    return path


def build(tmp_path, human, hands, name="data.npz"):
    out = tmp_path / name
    assert cli.main(["data", "--hands", ",".join(hands), "--human", str(human), "--out", str(out),
                     "--frames", "200"]) == 0
    return out


def engine(tmp_path, model, name):
    cfg = tmp_path / f"{name}.yaml"
    cfg.write_text(f"name: {name}\nside: right\nmodel: {model}\n")
    return GeortEngine.from_config(cfg)


def open_hand():
    j = np.zeros((26, 8))
    j[:, 3] = 1.0
    for root, x in {2: -0.03, 6: -0.02, 11: 0.0, 16: 0.02, 21: 0.04}.items():
        for k in range(4 if root == 2 else 5):
            j[root + k, 4:7] = [x, 0.0, -0.03 - 0.025 * k]
    return j


def test_three_steps_make_a_model_the_sdk_solves(tmp_path, human):
    for hand in HANDS:
        needs_urdf(hand)
    data = build(tmp_path, human, HANDS)
    model = tmp_path / "model.npz"
    assert cli.main(["train", "--data", str(data), "--out", str(model), *SMALL]) == 0
    assert sorted(load_prompt_model(model).embodiments) == sorted(HANDS)
    for hand in HANDS:
        solver = engine(tmp_path, model, hand)
        q = solver.solve(open_hand())
        lim = solver.joint_limits
        assert np.all(np.isfinite(q)) and np.all(q >= lim[:, 0]) and np.all(q <= lim[:, 1])


def test_stages_keep_untuned_hands_unchanged(tmp_path, human):
    for hand in HANDS:
        needs_urdf(hand)
    data = build(tmp_path, human, HANDS)
    body, joint, tuned = (tmp_path / f"{s}.npz" for s in ("body", "joint", "tuned"))
    args = ["train", "--data", str(data), *SMALL]
    assert cli.main([*args, "--out", str(body), "--human-only"]) == 0
    assert load_prompt_model(body).embodiments == {}
    assert cli.main(["train", "--data", str(data), "--out", str(joint), "--init", str(body),
                     "--freeze", "encoder", *SMALL]) == 0
    one = build(tmp_path, human, HANDS[:1], "one.npz")
    assert cli.main(["train", "--data", str(one), "--out", str(tuned), "--init", str(joint),
                     "--freeze", "body", *SMALL]) == 0
    before, after = load_prompt_model(joint).embodiments, load_prompt_model(tuned).embodiments
    points = before[HANDS[1]].check_points
    assert np.array_equal(before[HANDS[1]].forward(points), after[HANDS[1]].forward(points))
    assert not np.array_equal(before[HANDS[0]].forward(points), after[HANDS[0]].forward(points))


def test_palm_frame_without_ring_mcp_mirrors_index_about_middle():
    cfg = read_hand("dg5f_s_right")
    needs_urdf("dg5f_s_right")
    full = HandKinematics(cfg, urdf_path(cfg))
    cfg["palm"] = {k: v for k, v in cfg["palm"].items() if k != "ring"}
    three = HandKinematics(cfg, urdf_path(cfg))
    assert np.allclose(full.palm[:3, 2], three.palm[:3, 2])
    assert abs(three.palm[:3, 0] @ full.palm[:3, 0]) > 0.99


def test_bundled_training_configs_pass_the_check():
    for hand in config_names():
        if urdf_path(read_hand(hand)).is_file():
            assert cli.main(["check", hand]) == 0, hand


def test_inference_path_does_not_import_torch():
    code = ("import sys, whatslab.retarget.geort; "
            "assert 'torch' not in sys.modules; "
            "assert 'whatslab.retarget.geort.train' not in sys.modules")
    env = {**os.environ, "PYTHONPATH": os.pathsep.join(sys.path)}
    subprocess.run([sys.executable, "-c", code], check=True, env=env)


def test_torch_fk_matches_numpy_fk():
    from whatslab.retarget.geort.train.model import ExactFK

    for hand in HANDS:
        needs_urdf(hand)
        cfg = read_hand(hand)
        kin = HandKinematics(cfg, urdf_path(cfg))
        q = np.random.default_rng(1).uniform(kin.lower, kin.upper, size=(16, len(kin.lower)))
        tips = ExactFK(kin.to_dict(), "cpu")(torch.tensor(q, dtype=torch.float32)).numpy()
        assert np.allclose(tips, kin.tips(q), atol=1e-5), hand
