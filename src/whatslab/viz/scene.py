"""viser 기반 3D 씬 — URDF 메쉬/스켈레톤 + 사람 손 스켈레톤 (rerun 대체).

viser 는 "인터랙티브 3D 씬 + GUI 위젯"이라 라이브 텔레옵 모니터링/값 튜닝에
rerun("시계열 로그 뷰어")보다 맞고, dex_vla env 의 numpy1×rerun 쿼터니언 호환
문제류가 사라진다. `whatslab-sdk[viz]` (viser, trimesh) 필요.

  · URDFScene    : URDF 하나를 메쉬(STL) 또는 스켈레톤으로 렌더 + 관절 구동
  · RobotArmViz  : RobotModel(팔[+손]) 을 solver q 로 구동 + 목표 프레임 (라이브)
  · RobotHandViz : 리타게터의 로봇 손 모델을 q 로 FK (링크 스켈레톤)
  · HandSkeletonViz : 사람 손 23관절 스켈레톤 (점 + 뼈대)

viser 서버는 포트당 하나를 공유(get_server)해 여러 viz 가 한 화면에 공존한다.
"""
from __future__ import annotations

import os
from typing import Dict, List, Optional, Tuple

import numpy as np
import pinocchio as pin       # viz 모듈 — 시각화 deps 필수(없으면 명확히 ImportError)
import trimesh
import viser

from whatslab.core.types import HUMAN_HAND, JOINT_INDEX
from whatslab.paths import models_root

# viser/trimesh/pinocchio (`whatslab-sdk[viz]`) 는 이 모듈의 필수 의존 — 최상단 import.
_AXIS_RGB = ((230, 60, 60), (60, 200, 60), (70, 130, 240))   # x,y,z = R,G,B
_servers: Dict[int, "object"] = {}


def get_server(port: int = 8080):
    """포트당 ViserServer 하나를 캐시해 재사용 (여러 viz 공존)."""
    srv = _servers.get(port)
    if srv is None:
        srv = viser.ViserServer(port=port)
        srv.scene.add_frame("/canonical", show_axes=True,
                            axes_length=0.2, axes_radius=0.006)
        _servers[port] = srv
        print(f"[viz] viser: http://localhost:{port}")
    return srv


def _wxyz(R: np.ndarray) -> Tuple[float, float, float, float]:
    q = pin.Quaternion(np.asarray(R, dtype=float))       # SVD 없는 변환
    return (float(q.w), float(q.x), float(q.y), float(q.z))


class URDFScene:
    """URDF 를 viser 프레임 아래 로드 — 메쉬(STL) 있으면 메쉬, 없으면 스켈레톤.

    관절 구동: fk(q_full) 또는 set_joints({관절명: 값}) — 지정 안 된 관절은 중립.
    """

    def __init__(self, server, urdf: str, mesh_dir: str,
                 root_path: str = "/robot"):
        self.model = pin.buildModelFromUrdf(urdf)
        self.data = self.model.createData()
        self.root = server.scene.add_frame(root_path, show_axes=False)
        self._idx_q = {self.model.names[j]: self.model.joints[j].idx_q
                       for j in range(1, self.model.njoints)}
        self.handles: List = []
        self.gmodel = None
        # package:// 해석: URDF 가 `package://dexhand_description/...` 로 참조하면
        # pkg_dir 아래에서 dexhand_description/... 를 찾으므로 mesh_dir 의 **부모**가
        # 필요하다. 상대 경로 메쉬 대비 mesh_dir 도 함께 준다.
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
        except Exception:
            pass
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
        """{관절명: 값} → 이 URDF 의 전체 q (미지정 관절은 중립)."""
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
        """FK 후 프레임의 (루트 기준) 4x4 — 조인트/ fixed 프레임 공통."""
        pin.updateFramePlacements(self.model, self.data)
        return self.data.oMf[self.model.getFrameId(frame_name)].homogeneous


class RobotArmViz:
    """RobotModel(팔[+손]) 을 solver q 로 구동 — 팔/손 URDF 메쉬 + 목표 프레임.

    베이스 프레임에서 렌더한다(solver.fk 와 동일). 손이 있으면 팔 ee.parent 에
    [ee.origin ∘ attach ∘ hand.axis_align] 로 손을 붙여 그린다(RobotModel 체인과
    동일). 손의 카펄 등 지지 체인 관절은 solver q 로 함께 구동된다.
    """

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
        # 팔 베이스를 정준 프레임에 배치(M = mount∘axis_align) → rig 의 rpy 반영.
        # /arm 자식(메쉬·손)이 모두 이 변환을 상속한다.
        self._arm.set_root(self.model.to_canonical(np.eye(4)))
        if self.model.has_hand:
            self._hand = URDFScene(srv, rig.hand.urdf_abspath(), mesh_dir,
                                   "/arm/hand")
            self._aMb = (rig.arm.ee_origin.T @ rig.attach.T
                         @ rig.hand.axis_align.T)
        self._target = srv.scene.add_frame("/target", show_axes=True,
                                           axes_length=self.axis_len,
                                           axes_radius=0.005)
        # 실제 로봇 EE(target_ee) 프레임 — 목표(/target)와 비교용. 더 짧고 굵게.
        self._ee = srv.scene.add_frame("/ee", show_axes=True,
                                       axes_length=self.axis_len * 0.7,
                                       axes_radius=0.008)

    def update(self, q, target_pose=None, hand_q=None, hand_names=None,
               timestamp: Optional[float] = None) -> None:
        """q=팔 관절값(arm_joint_names 순). hand_q/hand_names 를 주면 손가락도 구동.

        손 URDF 는 카펄 등 지지체인 관절을 팔 q 에서, 나머지 손가락 관절을
        hand_q 에서 받는다(hand_names 기준). 미지정 관절은 중립.
        """
        if self._arm is None:
            self.start()
        _ = timestamp                                   # viser 는 라이브 뷰만
        arm_named = dict(zip(self.model.arm_joint_names, np.asarray(q, dtype=float)))
        self._arm.fk(self._arm.q_from_named(arm_named))  # 팔 (베이스 기준)
        if self._hand is not None:
            T_h = self._arm.frame_pose(self.model.rig.arm.ee_parent) @ self._aMb
            self._hand.set_root(T_h)                    # /arm 자식 → 팔 상속
            hand_named = dict(arm_named)                # 카펄 등 지지체인은 팔 q
            if hand_q is not None and hand_names is not None:
                hand_named.update(zip(hand_names, np.asarray(hand_q, dtype=float)))
            self._hand.fk(self._hand.q_from_named(hand_named))
        if target_pose is not None:
            T = np.asarray(target_pose, dtype=float)
            self._target.position = tuple(T[:3, 3])
            self._target.wxyz = _wxyz(T[:3, :3])
        # 실제 로봇 EE(target_ee) 정준 pose — solver q 로 FK
        T_ee = self.model.ee_pose(np.asarray(q, dtype=float))
        self._ee.position = tuple(T_ee[:3, 3])
        self._ee.wxyz = _wxyz(T_ee[:3, :3])


class RobotHandViz:
    """리타게터의 로봇 손 모델을 q 로 FK 해 링크 스켈레톤을 표시."""

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
    """사람 손 23관절 스켈레톤 (점 + 뼈대). positions_23: (23,3), index0=wrist."""

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
