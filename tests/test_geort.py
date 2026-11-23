import numpy as np
import pytest

from whatslab.core.types import HandPose, InputSample
from whatslab.retarget.controller import HandRetargetController, geort_config_path
from whatslab.retarget.geort import (
    GeortConfigError,
    GeortHandSolver,
    GeortInputError,
    load_prompt_model,
    read_config,
)
from whatslab.retarget.geort.keypoints import canonical_points, mediapipe21
from whatslab.retarget.xr_input import masked_keypoints

ORCA = "orca_right"


def _ctrl(tmp_path, model, side="right", **over):
    return HandRetargetController(side, f"geort:{_cfg(tmp_path, model, **over)}")


def _solver(tmp_path, model, side="right", **over):
    return GeortHandSolver.from_config(_cfg(tmp_path, model, **over), human_side=side)


def _q(solver, joints, valid=None):
    return solver.solve(masked_keypoints(joints, valid))


def identity_joints(spread: float = 0.0) -> np.ndarray:
    j = np.zeros((26, 8))
    j[:, 3] = 1.0
    j[:, 7] = 0.01
    rng = np.random.default_rng(0)
    for root, x in {2: -0.03, 6: -0.02, 11: 0.0, 16: 0.02, 21: 0.04}.items():
        for k in range(4 if root == 2 else 5):
            j[root + k, 4:7] = [x, 0.0, -0.03 - 0.025 * k]
    j[0, 4:7] = [0.0, 0.0, -0.05]
    j[:, 4:7] += spread * rng.standard_normal((26, 3))
    j[1, 4:7] = 0.0
    return j


@pytest.fixture
def geort_model():
    import os

    from whatslab.retarget.geort.solver import DEFAULT_MODEL, MODEL_ENV
    path = os.path.expanduser(os.environ.get(MODEL_ENV) or DEFAULT_MODEL)
    if not os.path.isfile(path):
        pytest.skip(f"GeoRT 모델 없음: {path} (${MODEL_ENV} 로 지정)")
    return path


def _cfg(tmp_path, model, **over):
    d = {"name": ORCA, "side": "right", "model": model, "low_pass_alpha": 1.0, **over}
    p = tmp_path / "geort.yaml"
    p.write_text("".join(f"{k}: {v}\n" for k, v in d.items() if v is not None))
    return p


def _sample(joints, tracked=True, valid=None):
    return InputSample(hand=HandPose(xr_joints=joints, xr_valid=valid, tracked=tracked),
                       tracked=tracked)


def test_canonical_points_are_scale_and_translation_invariant():
    j = identity_joints(0.003)
    rng = np.random.default_rng(3)
    template = mediapipe21(j)[[5, 9, 13, 17]] - mediapipe21(j)[0]
    a = canonical_points(mediapipe21(j), template, 0.08)
    moved = j.copy()
    moved[:, 4:7] = moved[:, 4:7] * 1.7 + rng.normal(size=3)
    b = canonical_points(mediapipe21(moved), template, 0.08)
    assert np.allclose(a, b, atol=1e-9)


def test_masked_keypoints_nan_only_invalid_positions():
    j = identity_joints()
    valid = np.ones(26, dtype=bool)
    valid[9] = False
    kp = masked_keypoints(j, valid)
    assert np.isnan(kp[9, 4:7]).all() and np.isfinite(kp[9, :4]).all()
    assert np.isfinite(np.delete(kp, 9, axis=0)).all()
    assert np.array_equal(masked_keypoints(j, None), j)


def test_model_reproduces_bake_self_check(geort_model):
    model = load_prompt_model(__import__("pathlib").Path(geort_model))
    for name, emb in model.embodiments.items():
        assert np.allclose(emb.forward(emb.check_points), emb.check_qpos, atol=1e-5), name


def test_controller_solves_within_limits(tmp_path, geort_model):
    ctrl = _ctrl(tmp_path, geort_model)
    assert len(ctrl.joint_names) == 16
    cmd = ctrl.compute(_sample(identity_joints(0.002)))
    lim = ctrl.engine.joint_limits
    assert cmd.joint_angles.shape == (16,)
    assert np.all(cmd.joint_angles >= lim[:, 0] - 1e-9) and np.all(
        cmd.joint_angles <= lim[:, 1] + 1e-9
    )
    assert ctrl.last_error is None


def test_controller_holds_last_when_untracked_or_invalid(tmp_path, geort_model):
    ctrl = _ctrl(tmp_path, geort_model)
    first = ctrl.compute(_sample(identity_joints(0.002))).joint_angles
    assert np.array_equal(
        ctrl.compute(_sample(identity_joints(0.01), tracked=False)).joint_angles, first
    )
    valid = np.ones(26, dtype=bool)
    valid[12] = False
    held = ctrl.compute(_sample(identity_joints(0.01), valid=valid)).joint_angles
    assert np.array_equal(held, first)
    assert isinstance(ctrl.last_error, GeortInputError)
    with pytest.raises(GeortInputError):
        _q(_solver(tmp_path, geort_model), identity_joints(), valid)


def test_controller_raises_non_input_errors(tmp_path, geort_model):
    ctrl = _ctrl(tmp_path, geort_model)
    with pytest.raises(ValueError) as info:
        ctrl.compute(_sample(np.zeros((25, 8))))
    assert not isinstance(info.value, GeortInputError)
    assert ctrl.last_error is None


def test_robot_side_is_model_side_for_mirrored_input(tmp_path, geort_model):
    assert _ctrl(tmp_path, geort_model, side="left").robot_side == "right"


def test_left_hand_is_mirrored_onto_right_model(tmp_path, geort_model):
    right = _solver(tmp_path, geort_model, "right")
    left = _solver(tmp_path, geort_model, "left")
    j = identity_joints(0.003)
    mirrored = j.copy()
    mirrored[:, 4] *= -1.0
    assert right.joint_names == left.joint_names
    assert np.allclose(_q(right, j), _q(left, mirrored), atol=1e-6)


def test_low_pass_alpha_smooths(tmp_path, geort_model):
    smooth = _solver(tmp_path, geort_model, low_pass_alpha=0.5)
    a = _q(smooth, identity_joints(0.0))
    b_raw = _q(_solver(tmp_path, geort_model), identity_joints(0.01))
    b = _q(smooth, identity_joints(0.01))
    assert np.allclose(b, a + 0.5 * (b_raw - a), atol=1e-9)


def test_unknown_embodiment_is_config_error(tmp_path, geort_model):
    with pytest.raises(GeortConfigError) as e:
        _solver(tmp_path, geort_model, name="nope")
    assert e.value.code == "GEORT_MODEL_MISMATCH"


def test_missing_model_raises_file_not_found(tmp_path):
    with pytest.raises(FileNotFoundError):
        _solver(tmp_path, str(tmp_path / "none.npz"))


def test_config_validation(tmp_path):
    with pytest.raises(GeortConfigError):
        read_config(_cfg(tmp_path, "m.npz", name=None))
    with pytest.raises(GeortConfigError):
        read_config(_cfg(tmp_path, "m.npz", low_pass_alpha=0))
    cfg = read_config(_cfg(tmp_path, "m.npz"))
    assert cfg.model_path == tmp_path / "m.npz"


def test_bundled_config_resolves_from_configs_root(monkeypatch, tmp_path):
    monkeypatch.setenv("WHATSLAB_GEORT_MODEL", str(tmp_path / "x.npz"))
    cfg = read_config("geort/orca_right.yaml")
    assert cfg.name == ORCA and cfg.side == "right" and cfg.low_pass_alpha == 0.5
    assert cfg.model_path == tmp_path / "x.npz"


@pytest.mark.parametrize(
    "config,urdf",
    [
        ("geort/orca_right.yaml", "orca_hand/urdf/right.urdf"),
        ("geort/hx5_right.yaml", "robotis_hx5_d20/urdf/hx5_d20_right.urdf"),
    ],
)
def test_bundled_configs_match_dexhand_urdf(config, urdf, geort_model):
    import os
    import xml.etree.ElementTree as ET

    dexhand = pytest.importorskip("dexhand_description")
    path = os.path.join(dexhand.get_share(), urdf)
    if not os.path.isfile(path):
        pytest.skip(f"URDF 없음: {path}")
    ctrl = GeortHandSolver.from_config(config)
    joints = {
        j.get("name")
        for j in ET.parse(path).getroot().findall("joint")
        if j.get("type") != "fixed"
    }
    assert set(ctrl.joint_names) <= joints


def test_unused_joints_do_not_block_solving(tmp_path, geort_model):
    valid = np.ones(26, dtype=bool)
    valid[[0, 6, 11, 16, 21]] = False
    q = _q(_solver(tmp_path, geort_model), identity_joints(0.002), valid)
    assert q.shape == (16,) and np.all(np.isfinite(q))


def test_low_pass_restarts_after_tracking_loss(tmp_path, geort_model):
    ctrl = _ctrl(tmp_path, geort_model, low_pass_alpha=0.3)
    fresh = _ctrl(tmp_path, geort_model, low_pass_alpha=0.3)
    ctrl.compute(_sample(identity_joints(0.0)))
    ctrl.compute(_sample(identity_joints(0.0), tracked=False))
    again = ctrl.compute(_sample(identity_joints(0.01))).joint_angles
    first = fresh.compute(_sample(identity_joints(0.01))).joint_angles
    assert np.allclose(again, first)


def test_env_model_path_is_relative_to_cwd(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    monkeypatch.setenv("WHATSLAB_GEORT_MODEL", "models/m.npz")
    cfg = read_config("geort/orca_right.yaml")
    assert cfg.model_path == tmp_path / "models" / "m.npz"


def test_first_command_is_inside_joint_limits(tmp_path, geort_model):
    ctrl = _ctrl(tmp_path, geort_model)
    held = ctrl.compute(_sample(identity_joints(), tracked=False)).joint_angles
    lim = ctrl.engine.joint_limits
    assert np.all(held >= lim[:, 0]) and np.all(held <= lim[:, 1])


def test_bare_geort_name_uses_packaged_config_not_cwd(tmp_path, monkeypatch):
    (tmp_path / "geort").mkdir()
    (tmp_path / "geort" / "orca_right.yaml").write_text("name: other\nside: right\n")
    monkeypatch.chdir(tmp_path)
    path = geort_config_path("orca_right")
    assert not path.startswith(str(tmp_path)) and path.endswith("geort/orca_right.yaml")
    assert read_config(path).name == ORCA


def test_model_file_is_read_once_per_path(tmp_path, geort_model, monkeypatch):
    from whatslab.retarget.geort import solver

    solver._cached_prompt_model.cache_clear()
    calls = []
    real = solver.load_prompt_model
    monkeypatch.setattr(solver, "load_prompt_model", lambda p: calls.append(p) or real(p))
    _solver(tmp_path, geort_model, side="right")
    _solver(tmp_path, geort_model, side="left")
    assert len(calls) == 1


def test_reset_clears_last_error(tmp_path, geort_model):
    ctrl = _ctrl(tmp_path, geort_model)
    valid = np.ones(26, dtype=bool)
    valid[12] = False
    ctrl.compute(_sample(identity_joints(0.01), valid=valid))
    assert isinstance(ctrl.last_error, GeortInputError)
    ctrl.reset()
    assert ctrl.last_error is None
