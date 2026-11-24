import pytest

pytest.importorskip("trimesh")
pytest.importorskip("viser")

def test_viz_exports_are_flat_classes():
    import whatslab.viz

    for name in ("HandViz", "URDFScene"):
        assert name in whatslab.viz.__all__, name
    assert whatslab.viz.HandViz.__bases__ == (object,)


class _Handle:
    position = (0.0, 0.0, 0.0)
    wxyz = (1.0, 0.0, 0.0, 0.0)
    points = None


class _Scene:
    def __init__(self):
        self.meshes = []
        self.trimeshes = []

    def add_frame(self, *a, **k):
        return _Handle()

    def add_mesh_trimesh(self, name, mesh):
        self.meshes.append(name)
        self.trimeshes.append(mesh)
        return _Handle()

    def add_line_segments(self, *a, **k):
        return _Handle()


class _Server:
    def __init__(self):
        self.scene = _Scene()


def _urdf(tmp_path, visual, collision):
    import trimesh

    trimesh.creation.box().export(tmp_path / "box.stl")
    path = tmp_path / "hand.urdf"
    path.write_text(f"""<robot name="h">
  <link name="base">
    <visual><geometry><mesh filename="{visual}"/></geometry></visual>
    <collision><geometry><mesh filename="{collision}"/></geometry></collision>
  </link>
  <link name="tip"/>
  <joint name="j" type="revolute"><parent link="base"/><child link="tip"/>
    <axis xyz="0 0 1"/><limit lower="-1" upper="1" effort="1" velocity="1"/></joint>
</robot>""")
    return str(path)


def test_missing_visual_mesh_falls_back_to_collision(tmp_path):
    from whatslab.viz import URDFScene

    srv = _Server()
    scene = URDFScene(srv, _urdf(tmp_path, "missing.stl", "box.stl"), str(tmp_path))
    assert scene.mesh_mode and srv.scene.meshes


def test_unloadable_meshes_fall_back_to_skeleton(tmp_path):
    from whatslab.viz import URDFScene

    (tmp_path / "hand.blend").write_text("")
    scene = URDFScene(_Server(), _urdf(tmp_path, "missing.stl", "hand.blend"), str(tmp_path))
    assert not scene.mesh_mode
    scene.fk(scene.q_from_named({"j": 0.5}))


def test_visual_material_colours_plain_meshes(tmp_path):
    import numpy as np
    import trimesh

    from whatslab.viz import URDFScene

    trimesh.creation.box().export(tmp_path / "box.stl")
    painted = trimesh.creation.box()
    painted.visual.face_colors = [10, 20, 30, 255]
    painted.export(tmp_path / "painted.glb")
    path = tmp_path / "hand.urdf"
    path.write_text("""<robot name="h">
  <material name="red"><color rgba="1 0 0 1"/></material>
  <link name="base">
    <visual><geometry><mesh filename="box.stl"/></geometry><material name="red"/></visual>
    <visual><geometry><mesh filename="box.stl"/></geometry>
      <material name="green"><color rgba="0 0.5 0 1"/></material></visual>
    <visual><geometry><mesh filename="box.stl"/></geometry></visual>
    <visual><geometry><mesh filename="painted.glb"/></geometry><material name="red"/></visual>
  </link>
</robot>""")
    srv = _Server()
    URDFScene(srv, str(path), str(tmp_path / "elsewhere"))
    red, green, plain, painted = srv.scene.trimeshes
    assert np.all(red.visual.face_colors == [255, 0, 0, 255])
    assert np.all(green.visual.face_colors == [0, 128, 0, 255])
    assert plain.visual.kind is None
    assert painted.visual.kind is not None
    assert np.all(painted.visual.face_colors == [10, 20, 30, 255])
