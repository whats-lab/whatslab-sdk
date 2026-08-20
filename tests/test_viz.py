import pytest

pytest.importorskip("pinocchio")
pytest.importorskip("trimesh")
pytest.importorskip("viser")

def test_viz_exports_are_flat_classes():
    import whatslab.viz

    for name in ("RobotArmViz", "HandViz", "URDFScene"):
        assert name in whatslab.viz.__all__, name
    assert whatslab.viz.HandViz.__bases__ == (object,)
    assert whatslab.viz.RobotArmViz.__bases__ == (object,)
