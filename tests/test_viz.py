import pytest

pytest.importorskip("pinocchio")
pytest.importorskip("trimesh")
pytest.importorskip("viser")

from whatslab.viz.scene import _bone_pairs  # noqa: E402


def test_bone_pairs_from_skeleton():
    pairs = _bone_pairs()
    assert len(pairs) == 22
    for parent_i, child_i in pairs:
        assert 0 <= parent_i < 23 and 0 <= child_i < 23
        assert parent_i != child_i


def test_viz_module_imports_without_viser():
    import whatslab.viz  # noqa: F401
    assert "RobotArmViz" in whatslab.viz.__all__
