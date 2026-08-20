import pytest

pytest.importorskip("pinocchio")
pytest.importorskip("trimesh")
pytest.importorskip("viser")

def test_viz_module_imports_without_viser():
    import whatslab.viz  # noqa: F401
    assert "RobotArmViz" in whatslab.viz.__all__
