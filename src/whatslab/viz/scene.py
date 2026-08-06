from __future__ import annotations

import logging
import os
from typing import Dict, List, Optional, Tuple

import numpy as np
import pinocchio as pin
import trimesh
import viser

from whatslab.core.types import HUMAN_HAND, JOINT_INDEX
from whatslab.paths import models_root

_log = logging.getLogger(__name__)

_AXIS_RGB = ((230, 60, 60), (60, 200, 60), (70, 130, 240))
_servers: Dict[int, "object"] = {}


def get_server(port: int = 8080):
    srv = _servers.get(port)
    if srv is None:
        srv = viser.ViserServer(port=port)
        srv.scene.add_frame("/canonical", show_axes=True,
                            axes_length=0.2, axes_radius=0.006)
        _servers[port] = srv
        print(f"[viz] viser: http://localhost:{port}")
    return srv


def _wxyz(R: np.ndarray) -> Tuple[float, float, float, float]:
    q = pin.Quaternion(np.asarray(R, dtype=float))
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
        try:
            for gtype in (pin.GeometryType.COLLISION, pin.GeometryType.VISUAL):
                gm = pin.buildGeomFromUrdf(self.model, urdf, gtype,
                                           package_dirs=pkg_dirs)
                if len(gm.geometryObjects) > 0:
                    self.gmodel = gm
                    break
            if self.gmodel is None:
                raise RuntimeError("URDF 에 지오메트리 없음")
            self.gdata = pin.GeometryData(self.gmodel)
            for g in self.gmodel.geometryObjects:
                path = str(g.meshPath)
                if not path or path in ("BOX", "SPHERE", "CYLINDER") \
                        or not os.path.exists(path):
                    self.handles.append(None)
                    continue
                mesh = trimesh.load(path, force="mesh")
                mesh.apply_scale(np.asarray(g.meshScale))
                self.handles.append(
                    server.scene.add_mesh_trimesh(f"{root_path}/{g.name}", mesh))
        except Exception as e:
            _log.warning("메쉬 로드 실패 → 스켈레톤 모드로 강등: %s", e)
        self.mesh_mode = any(h is not None for h in self.handles)
        if not self.mesh_mode:
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

    def set_root(self, T: np.ndarray) -> None:
        self.root.position = tuple(float(v) for v in T[:3, 3])
        self.root.wxyz = _wxyz(T[:3, :3])

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
            for h, oMg in zip(self.handles, self.gdata.oMg):
                if h is None:
                    continue
                h.position = tuple(oMg.translation)
                h.wxyz = _wxyz(oMg.rotation)
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


class RobotArmViz:

    def __init__(self, model, port: int = 8080, axis_len: float = 0.12):
        self.model = model
        self.port = port
        self.axis_len = axis_len
        self._arm = None
        self._hand = None
        self._target = None

    def start(self) -> None:
        rig = self.model.rig
        srv = get_server(self.port)
        mesh_dir = models_root()
        self._arm = URDFScene(srv, rig.arm.urdf_abspath(), mesh_dir, "/arm")
        self._arm.set_root(self.model.to_canonical(np.eye(4)))
        if self.model.has_hand:
            self._hand = URDFScene(srv, rig.hand.urdf_abspath(), mesh_dir,
                                   "/arm/hand")
            self._aMb = (rig.arm.ee_origin.T @ rig.attach.T
                         @ rig.hand.axis_align.T)
        self._target = srv.scene.add_frame("/target", show_axes=True,
                                           axes_length=self.axis_len,
                                           axes_radius=0.005)
        self._ee = srv.scene.add_frame("/ee", show_axes=True,
                                       axes_length=self.axis_len * 0.7,
                                       axes_radius=0.008)

    def update(self, q, target_pose=None, hand_q=None, hand_names=None,
               timestamp: Optional[float] = None) -> None:
        if self._arm is None:
            self.start()
        _ = timestamp
        arm_named = dict(zip(self.model.arm_joint_names, np.asarray(q, dtype=float)))
        self._arm.fk(self._arm.q_from_named(arm_named))
        if self._hand is not None:
            T_h = self._arm.frame_pose(self.model.rig.arm.ee_parent) @ self._aMb
            self._hand.set_root(T_h)
            hand_named = dict(arm_named)
            if hand_q is not None and hand_names is not None:
                hand_named.update(zip(hand_names, np.asarray(hand_q, dtype=float)))
            self._hand.fk(self._hand.q_from_named(hand_named))
        if target_pose is not None:
            T = np.asarray(target_pose, dtype=float)
            self._target.position = tuple(T[:3, 3])
            self._target.wxyz = _wxyz(T[:3, :3])
        T_ee = self.model.ee_pose(np.asarray(q, dtype=float))
        self._ee.position = tuple(T_ee[:3, 3])
        self._ee.wxyz = _wxyz(T_ee[:3, :3])


class RobotHandViz:

    def __init__(self, retargeter, port: int = 8080, root_path: str = "/robot_hand"):
        self._robot = retargeter._seq_stage1.optimizer.robot
        self._port = port
        self._root_path = root_path
        self._joints = None

    def start(self) -> None:
        srv = get_server(self._port)
        m = self._robot.model
        ball = trimesh.creation.icosphere(radius=0.005)
        ball.visual.face_colors = [250, 200, 90, 255]
        self._joints = [srv.scene.add_mesh_trimesh(f"{self._root_path}/j{j}",
                                                   ball.copy())
                        for j in range(1, m.njoints)]
        n = sum(1 for j in range(1, m.njoints) if int(m.parents[j]) >= 1)
        self._bones = srv.scene.add_line_segments(
            f"{self._root_path}/bones", points=np.zeros((max(n, 1), 2, 3)),
            colors=(200, 160, 70), line_width=3.0)

    def update(self, q, timestamp: Optional[float] = None) -> None:
        if self._joints is None:
            self.start()
        _ = timestamp
        m, d = self._robot.model, self._robot.data
        qv = np.asarray(q, dtype=float)
        if qv.shape[0] != m.nq:
            qv = np.resize(qv, m.nq)
        pin.forwardKinematics(m, d, qv)
        segs = []
        for j in range(1, m.njoints):
            p = d.oMi[j].translation
            self._joints[j - 1].position = tuple(p)
            par = int(m.parents[j])
            if par >= 1:
                segs.append([d.oMi[par].translation.copy(), p.copy()])
        if segs:
            self._bones.points = np.asarray(segs)


def _bone_pairs() -> List[Tuple[int, int]]:
    return [(JOINT_INDEX[s.parent], i) for i, s in enumerate(HUMAN_HAND)
            if s.parent is not None]


class HandSkeletonViz:

    def __init__(self, port: int = 8080, root_path: str = "/human_hand"):
        self._port = port
        self._root_path = root_path
        self._bones_idx = _bone_pairs()
        self._pts = None

    def start(self) -> None:
        srv = get_server(self._port)
        ball = trimesh.creation.icosphere(radius=0.004)
        ball.visual.face_colors = [80, 200, 210, 255]
        self._pts = [srv.scene.add_mesh_trimesh(f"{self._root_path}/p{i}",
                                                ball.copy())
                     for i in range(len(HUMAN_HAND))]
        self._lines = srv.scene.add_line_segments(
            f"{self._root_path}/bones",
            points=np.zeros((max(len(self._bones_idx), 1), 2, 3)),
            colors=(150, 150, 160), line_width=2.0)

    def update(self, positions_23: np.ndarray, timestamp: Optional[float] = None) -> None:
        if self._pts is None:
            self.start()
        _ = timestamp
        p = np.asarray(positions_23, dtype=float)
        for i, h in enumerate(self._pts):
            h.position = tuple(p[i])
        segs = [[p[a], p[b]] for a, b in self._bones_idx]
        if segs:
            self._lines.points = np.asarray(segs)
