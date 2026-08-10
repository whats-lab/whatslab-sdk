from __future__ import annotations

import logging
import os
from typing import Dict, List, Optional, Tuple

import numpy as np
import pinocchio as pin
import trimesh
import viser

from whatslab.core.types import HUMAN_HAND, JOINT_INDEX
from whatslab.solvers.hand.human_fk import FINGERS as _HFING
from whatslab.solvers.hand.human_fk import palm_frame as _hpf
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


def _human_palm_frame(hp):
    return _hpf({f: hp[f][0] for f in _HFING}, hp["palm"])


_PALM_TO_WORLD = np.array([[1.0, 0.0, 0.0],
                           [0.0, 0.0, -1.0],
                           [0.0, 1.0, 0.0]])


def upright_root(palm_origin, palm_frame_R) -> np.ndarray:
    A = _PALM_TO_WORLD @ np.asarray(palm_frame_R, dtype=float).T
    T = np.eye(4)
    T[:3, :3] = A
    T[:3, 3] = -A @ np.asarray(palm_origin, dtype=float)
    return T


class _UrdfHandViz:

    def __init__(self, urdf: str, joint_names, port: int = 8080,
                 root_path: str = "/hand", root_pose=None):
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


class RobotHandViz(_UrdfHandViz):

    def __init__(self, retargeter, port: int = 8080, root_path: str = "/robot_hand",
                 root_pose=None, upright: bool = True):
        urdf = getattr(retargeter, "urdf_path", None)
        if urdf is None:
            raise ValueError(
                f"{type(retargeter).__name__} 에 urdf_path 가 없다 — 메쉬를 못 띄운다")
        if root_pose is None and upright:
            o = getattr(retargeter, "_r_origin", None)
            R = getattr(retargeter, "_r_frame", None)
            if o is not None and R is not None:
                root_pose = upright_root(o, R)
        super().__init__(urdf, retargeter.joint_names, port, root_path, root_pose)


class HumanHandViz(_UrdfHandViz):

    def __init__(self, fk, port: int = 8080, root_path: str = "/human_hand",
                 root_pose=None, upright: bool = True, offset=None):
        if root_pose is None and upright:
            hp = fk.neutral_points()
            o, R = _human_palm_frame(hp)
            root_pose = upright_root(o, R)
        if offset is not None and root_pose is not None:
            shift = np.eye(4)
            shift[:3, 3] = np.asarray(offset, dtype=float)
            root_pose = shift @ root_pose
        super().__init__(fk.urdf_path, fk.joint_names, port, root_path, root_pose)


class KPHandViz:

    _TARGET_RGB = (90, 210, 250)
    _ACHIEVED_RGB = (250, 170, 70)

    def __init__(self, engine, port: int = 8080, root_path: str = "/kp_hand",
                 mesh: bool = True, upright: bool = True):
        self.engine = engine
        self.port = port
        self.root_path = root_path
        self._mesh = mesh
        self._T = (upright_root(engine._r_origin, engine._r_frame) if upright
                   else np.eye(4))
        self._scene = None
        self._tgt = None
        self._ach = None
        self._err = None
        self._fingers = list(engine.keypoints)
        self._n = sum(len(v) for v in engine.keypoints.values())

    def start(self) -> None:
        srv = get_server(self.port)
        if self._mesh:
            urdf = getattr(self.engine, "urdf_path", None)
            if urdf:
                self._scene = URDFScene(srv, urdf, models_root(), self.root_path)
                self._scene.set_root(self._T)
        def _balls(name, rgb, radius):
            ball = trimesh.creation.icosphere(radius=radius)
            ball.visual.face_colors = [*rgb, 255]
            return [srv.scene.add_mesh_trimesh(f"{self.root_path}/{name}{i}",
                                               ball.copy())
                    for i in range(self._n)]
        self._tgt = _balls("t", self._TARGET_RGB, 0.006)
        self._ach = _balls("a", self._ACHIEVED_RGB, 0.004)
        self._err = srv.scene.add_line_segments(
            f"{self.root_path}/err", points=np.zeros((self._n, 2, 3)),
            colors=(230, 80, 80), line_width=2.0)
        self._n_bone = self._n - len(self._fingers)
        self._bone = srv.scene.add_line_segments(
            f"{self.root_path}/tgt_bones",
            points=np.zeros((max(self._n_bone, 1), 2, 3)),
            colors=self._TARGET_RGB, line_width=4.0)

    def update(self, timestamp: Optional[float] = None) -> None:
        if self._tgt is None:
            self.start()
        _ = timestamp
        eng = self.engine
        if self._scene is not None:
            self._scene.fk(self._scene.q_from_named(
                dict(zip(eng.joint_names, eng.current_q()))))
        T = eng.last_targets()
        A = eng.achieved_points()
        R, off = self._T[:3, :3], self._T[:3, 3]

        def w(p):
            return R @ np.asarray(p, dtype=float) + off

        segs, bones, i = [], [], 0
        for f in self._fingers:
            pts = A[f] if T is None else T[f]
            for k in range(len(eng.keypoints[f])):
                a = w(A[f][k])
                t = w(pts[k])
                self._ach[i].position = tuple(float(v) for v in a)
                self._tgt[i].position = tuple(float(v) for v in t)
                segs.append([t.copy(), a.copy()])
                if k > 0:
                    bones.append([w(pts[k - 1]), t.copy()])
                i += 1
        self._err.points = np.asarray(segs)
        if bones:
            self._bone.points = np.asarray(bones)


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
