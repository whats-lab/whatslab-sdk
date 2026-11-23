from __future__ import annotations

import logging
import os
import re
from typing import Dict, List, Optional, Tuple

import numpy as np
import pinocchio as pin
import trimesh
import viser

from whatslab.paths import models_root

_log = logging.getLogger(__name__)

_servers: Dict[int, "object"] = {}


def get_server(port: int = 8080):
    srv = _servers.get(port)
    if srv is None:
        srv = viser.ViserServer(port=port)
        srv.scene.add_frame("/canonical", show_axes=True,
                            axes_length=0.2, axes_radius=0.006)
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


def _wxyz(rot: np.ndarray) -> Tuple[float, float, float, float]:
    q = pin.Quaternion(np.asarray(rot, dtype=float))
    return (float(q.w), float(q.x), float(q.y), float(q.z))


class URDFScene:

    def __init__(self, server, urdf: str, mesh_dir: str,
                 root_path: str = "/robot"):
        self.model = pin.buildModelFromUrdf(urdf)
        self.data = self.model.createData()
        self.root = server.scene.add_frame(root_path, show_axes=False)
        self._idx_q = {self.model.names[j]: self.model.joints[j].idx_q
                       for j in range(1, self.model.njoints)}
        self.handles: List = []
        self.gmodel = None
        pkg_dirs = [mesh_dir, os.path.dirname(mesh_dir)]
        urdf_text = _rebased_urdf(urdf, mesh_dir)
        best = 0
        for gtype in (pin.GeometryType.VISUAL, pin.GeometryType.COLLISION):
            try:
                if urdf_text is None:
                    gm = pin.buildGeomFromUrdf(self.model, urdf, gtype,
                                               package_dirs=pkg_dirs)
                else:
                    gm = pin.buildGeomFromUrdfString(self.model, urdf_text,
                                                     gtype,
                                                     package_dirs=pkg_dirs)
            except Exception as e:
                _log.info("%s 지오메트리 건너뜀: %s", gtype, e)
                continue
            n = sum(1 for g in gm.geometryObjects
                    if os.path.exists(str(g.meshPath)))
            if n > best:
                self.gmodel, best = gm, n
        if self.gmodel is not None:
            self.gdata = pin.GeometryData(self.gmodel)
            for g in self.gmodel.geometryObjects:
                path = str(g.meshPath)
                if not path or path in ("BOX", "SPHERE", "CYLINDER") \
                        or not os.path.exists(path):
                    self.handles.append(None)
                    continue
                try:
                    mesh = trimesh.load(path, force="mesh")
                    mesh.apply_scale(np.asarray(g.meshScale))
                    self.handles.append(server.scene.add_mesh_trimesh(
                        f"{root_path}/{g.name}", mesh))
                except Exception as e:
                    _log.info("메쉬 건너뜀 %s: %s", path, e)
                    self.handles.append(None)
        self.mesh_mode = any(h is not None for h in self.handles)
        if not self.mesh_mode:
            _log.warning("메쉬 로드 실패 → 스켈레톤 모드로 강등: %s", urdf)
            ball = trimesh.creation.icosphere(radius=0.008)
            ball.visual.face_colors = [250, 200, 90, 255]
            self.joint_handles = [
                server.scene.add_mesh_trimesh(f"{root_path}/j{j}", ball.copy())
                for j in range(1, self.model.njoints)]
            n_bones = sum(1 for j in range(1, self.model.njoints)
                          if int(self.model.parents[j]) >= 1)
            self.bones = server.scene.add_line_segments(
                f"{root_path}/bones", points=np.zeros((max(n_bones, 1), 2, 3)),
                colors=(200, 160, 70), line_width=3.0)

    def set_root(self, tf: np.ndarray) -> None:
        self.root.position = tuple(float(v) for v in tf[:3, 3])
        self.root.wxyz = _wxyz(tf[:3, :3])

    def q_from_named(self, name_to_val: Dict[str, float]) -> np.ndarray:
        q = pin.neutral(self.model)
        for name, val in name_to_val.items():
            if name in self._idx_q:
                q[self._idx_q[name]] = float(val)
        return q

    def fk(self, q: np.ndarray) -> None:
        pin.forwardKinematics(self.model, self.data, np.asarray(q, dtype=float))
        if self.mesh_mode:
            pin.updateGeometryPlacements(self.model, self.data,
                                         self.gmodel, self.gdata)
            for h, o_to_g in zip(self.handles, self.gdata.oMg):
                if h is None:
                    continue
                h.position = tuple(o_to_g.translation)
                h.wxyz = _wxyz(o_to_g.rotation)
        else:
            segs = []
            for j in range(1, self.model.njoints):
                p = self.data.oMi[j].translation
                self.joint_handles[j - 1].position = tuple(p)
                par = int(self.model.parents[j])
                if par >= 1:
                    segs.append([self.data.oMi[par].translation.copy(), p.copy()])
            if segs:
                self.bones.points = np.asarray(segs)

    def frame_pose(self, frame_name: str) -> np.ndarray:
        pin.updateFramePlacements(self.model, self.data)
        return self.data.oMf[self.model.getFrameId(frame_name)].homogeneous


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
        self._scene = URDFScene(get_server(self.port), self.urdf, models_root(),
                                self.root_path)
        if self.root_pose is not None:
            self._scene.set_root(np.asarray(self.root_pose, dtype=float))

    def update(self, q, timestamp: Optional[float] = None) -> None:
        if self._scene is None:
            self.start()
        _ = timestamp
        named = dict(zip(self.joint_names, np.asarray(q, dtype=float).ravel()))
        self._scene.fk(self._scene.q_from_named(named))

    @property
    def mesh_mode(self) -> bool:
        return bool(self._scene is not None and self._scene.mesh_mode)
