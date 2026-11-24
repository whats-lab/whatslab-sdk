from __future__ import annotations

import logging
import os
import re
import xml.etree.ElementTree as ET
from typing import Dict, List, Optional, Tuple

import numpy as np
import trimesh
import viser
from scipy.spatial.transform import Rotation

from whatslab.kinematics import Kinematics, origin_matrix
from whatslab.paths import mesh_urdf, models_root

_log = logging.getLogger(__name__)

_servers: Dict[int, "object"] = {}


def get_server(port: int = 8080):
    srv = _servers.get(port)
    if srv is None:
        srv = viser.ViserServer(port=port)
        _servers[port] = srv
        print(f"[viz] viser: http://localhost:{srv.get_port()}")
    return srv


def _rebased_urdf(urdf: str, root: str) -> Optional[str]:
    with open(urdf, "r") as f:
        text = f.read()
    root = root.rstrip("/")
    names = {n for n in re.findall(r'package://([^/"\']+)/', text)
             if not os.path.isdir(os.path.join(root, n))}
    if not names:
        return None
    for name in names:
        text = text.replace("package://%s/" % name, root + "/")
    return text


def _mesh_path(filename: str, urdf_dir: str, pkg_dirs: List[str]) -> Optional[str]:
    if filename.startswith("package://"):
        rel = filename[len("package://"):]
        paths = [os.path.join(d, rel) for d in pkg_dirs]
    else:
        path = filename[len("file://"):] if filename.startswith("file://") else filename
        paths = [path if os.path.isabs(path) else os.path.join(urdf_dir, path)]
    return next((p for p in paths if os.path.isfile(p)), None)


def _mesh_loadable(path: Optional[str]) -> bool:
    ext = os.path.splitext(path or "")[1].lstrip(".").lower()
    return path is not None and ext in trimesh.available_formats()


def _rgba(material) -> Optional[np.ndarray]:
    color = None if material is None else material.find("color")
    return None if color is None else np.array([float(v) for v in color.get("rgba").split()])


def _geometries(robot, tag: str, urdf_dir: str, pkg_dirs: List[str]) -> List[Dict]:
    named: Dict[str, Optional[np.ndarray]] = {}
    for m in robot.findall("material"):
        named.setdefault(m.get("name"), _rgba(m))
    out = []
    for link in robot.findall("link"):
        for i, el in enumerate(link.findall(tag)):
            mesh = el.find("geometry/mesh")
            material = el.find("material")
            name = None if material is None else material.get("name")
            rgba = named[name] if name in named else _rgba(material)
            if name:
                named.setdefault(name, rgba)
            out.append({
                "name": f"{link.get('name')}_{i}",
                "link": link.get("name"),
                "origin": origin_matrix(el.find("origin")),
                "filename": None if mesh is None else mesh.get("filename"),
                "path": None if mesh is None else _mesh_path(mesh.get("filename"), urdf_dir,
                                                             pkg_dirs),
                "scale": np.array([float(v) for v in
                                   (mesh.get("scale", "1 1 1") if mesh is not None
                                    else "1 1 1").split()]),
                "rgba": rgba if tag == "visual" else None,
            })
    return out


def _wxyz(rot: np.ndarray) -> Tuple[float, float, float, float]:
    x, y, z, w = Rotation.from_matrix(np.asarray(rot, dtype=float)).as_quat()
    return (float(w), float(x), float(y), float(z))


class URDFScene:

    def __init__(self, server, urdf: str, mesh_dir: str,
                 root_path: str = "/robot"):
        urdf_dir = os.path.dirname(os.path.abspath(urdf))
        pkg_dirs = [urdf_dir, mesh_dir, os.path.dirname(mesh_dir)]
        urdf_text = _rebased_urdf(urdf, mesh_dir)
        with open(urdf, "r") as f:
            text = urdf_text if urdf_text is not None else f.read()
        robot = ET.fromstring(text)
        self.kin = Kinematics.from_string(text)
        self.root = server.scene.add_frame(root_path, show_axes=False)
        self.handles: List = []
        self.geoms: List[Dict] = []
        best = 0
        for tag in ("visual", "collision"):
            geoms = _geometries(robot, tag, urdf_dir, pkg_dirs)
            meshes = [g for g in geoms if g["filename"] is not None]
            if not meshes or not all(_mesh_loadable(g["path"]) for g in meshes):
                continue
            if len(meshes) > best:
                self.geoms, best = geoms, len(meshes)
        self._link_ids = self.kin.frame_ids([g["link"] for g in self.geoms])
        for g in self.geoms:
            if g["path"] is None:
                self.handles.append(None)
                continue
            mesh = trimesh.load(g["path"], force="mesh")
            mesh.apply_scale(g["scale"])
            if mesh.visual.kind is None and g["rgba"] is not None:
                mesh.visual.face_colors = np.round(g["rgba"] * 255).astype(np.uint8)
            self.handles.append(server.scene.add_mesh_trimesh(f"{root_path}/{g['name']}", mesh))
        self.mesh_mode = any(h is not None for h in self.handles)
        if not self.mesh_mode:
            _log.warning("메쉬 로드 실패 → 스켈레톤 모드로 강등: %s", urdf)
            ball = trimesh.creation.icosphere(radius=0.008)
            ball.visual.face_colors = [250, 200, 90, 255]
            self.joint_handles = [
                server.scene.add_mesh_trimesh(f"{root_path}/j{j}", ball.copy())
                for j in range(1, self.kin.dof + 1)]
            n_bones = sum(1 for p in self.kin.joint_parents if p >= 0)
            self.bones = server.scene.add_line_segments(
                f"{root_path}/bones", points=np.zeros((max(n_bones, 1), 2, 3)),
                colors=(200, 160, 70), line_width=3.0)

    def set_root(self, tf: np.ndarray) -> None:
        self.root.position = tuple(float(v) for v in tf[:3, 3])
        self.root.wxyz = _wxyz(tf[:3, :3])

    def q_from_named(self, name_to_val: Dict[str, float]) -> np.ndarray:
        q = self.kin.neutral()
        for name, val in name_to_val.items():
            if name in self.kin.joint_names:
                q[self.kin.joint_names.index(name)] = float(val)
        return q

    def fk(self, q: np.ndarray) -> None:
        if self.mesh_mode:
            links = self.kin.link_placements(q)[self._link_ids]
            for h, link, g in zip(self.handles, links, self.geoms):
                if h is None:
                    continue
                tf = link @ g["origin"]
                h.position = tuple(tf[:3, 3])
                h.wxyz = _wxyz(tf[:3, :3])
        else:
            joints = self.kin.joint_frames(q)
            segs = []
            for j, par in enumerate(self.kin.joint_parents):
                p = joints[j, :3, 3]
                self.joint_handles[j].position = tuple(p)
                if par >= 0:
                    segs.append([joints[par, :3, 3], p])
            if segs:
                self.bones.points = np.asarray(segs)


class HandViz:

    def __init__(self, urdf: str, joint_names, port: int = 8080,
                 root_path: str = "/hand", root_pose=None):
        if not urdf:
            raise ValueError("HandViz: urdf 경로가 없다 — 메쉬를 못 띄운다")
        self.urdf = urdf
        self.joint_names = list(joint_names)
        self.port = port
        self.root_path = root_path
        self.root_pose = root_pose
        self._scene = None

    def start(self) -> None:
        self._scene = URDFScene(get_server(self.port), mesh_urdf(self.urdf), models_root(),
                                self.root_path)
        if self.root_pose is not None:
            self._scene.set_root(np.asarray(self.root_pose, dtype=float))

    def update(self, q) -> None:
        if self._scene is None:
            self.start()
        named = dict(zip(self.joint_names, np.asarray(q, dtype=float).ravel()))
        self._scene.fk(self._scene.q_from_named(named))

    @property
    def mesh_mode(self) -> bool:
        return bool(self._scene is not None and self._scene.mesh_mode)
