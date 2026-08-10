from __future__ import annotations

import os
from typing import Dict, List, Mapping, Optional, Sequence

import numpy as np
import pinocchio as pin

from whatslab.core.types import HUMAN_HAND, JOINT_INDEX
from whatslab.paths import models_root

FINGERS = ("thumb", "index", "middle", "ring", "pinky")
BONE_LINKS: Dict[str, Sequence[str]] = {
    "thumb": ("thumb_cmc1", "thumb_mcp", "thumb_ip", "thumb_tip"),
    "index": ("index_mcp", "index_pip", "index_dip", "index_tip"),
    "middle": ("middle_mcp", "middle_pip", "middle_dip", "middle_tip"),
    "ring": ("ring_mcp", "ring_pip", "ring_dip", "ring_tip"),
    "pinky": ("pinky_mcp", "pinky_pip", "pinky_dip", "pinky_tip"),
}
PALM_LINKS = ("sensor_dorsum", "wrist")
_LINK_ALIAS = {"pinky0": "pinky_cmc"}


def rot_between(u: np.ndarray, v: np.ndarray) -> np.ndarray:
    u = u / max(np.linalg.norm(u), 1e-12)
    v = v / max(np.linalg.norm(v), 1e-12)
    c = np.cross(u, v)
    d = float(u @ v)
    if np.linalg.norm(c) < 1e-12:
        return np.eye(3) if d > 0 else -np.eye(3)
    K = np.array([[0, -c[2], c[1]], [c[2], 0, -c[0]], [-c[1], c[0], 0]])
    return np.eye(3) + K + K @ K * ((1 - d) / (np.linalg.norm(c) ** 2))


def palm_frame(base_pts: Dict[str, np.ndarray], palm_pt: np.ndarray):
    o = np.mean([base_pts[f] for f in ("index", "middle", "ring", "pinky")], axis=0)
    x = base_pts["index"] - base_pts["pinky"]
    x = x / np.linalg.norm(x)
    y = o - np.asarray(palm_pt, dtype=float)
    y = y - x * (y @ x)
    ny = float(np.linalg.norm(y))
    if ny < 1e-3:
        raise ValueError(
            f"팜 프레임 y 축이 특이하다(|y|={ny * 1e3:.2f}mm) — 팜 기준점이 너클"
            " 평면에 너무 가깝다")
    return o, np.column_stack([x, y / ny, np.cross(x, y / ny)])


def link_candidates(side: str, joint_name: str) -> Sequence[str]:
    pfx = side + "_"
    if joint_name.endswith("_tip"):
        return (pfx + f"sensor_{joint_name[:-4]}_distal", pfx + joint_name)
    return (pfx + _LINK_ALIAS.get(joint_name, joint_name),)


class HumanHandFK:

    def __init__(self, side: str = "right", urdf_path: Optional[str] = None):
        self.side = side.lower()
        if urdf_path is None:
            urdf_path = os.path.join(models_root(), "base_hand", "urdf",
                                     f"{self.side}.urdf")
        self.urdf_path = urdf_path
        self.model = pin.buildModelFromUrdf(urdf_path)
        self.data = self.model.createData()

        self.joint_names: List[str] = [
            self.model.names[j] for j in range(1, self.model.njoints)
            if self.model.joints[j].nq > 0]
        self._idx_q = {self.model.names[j]: self.model.joints[j].idx_q
                       for j in range(1, self.model.njoints)}

        self.links: Dict[str, str] = {}
        for spec in HUMAN_HAND:
            for cand in link_candidates(self.side, spec.name):
                if self._has(cand):
                    self.links[spec.name] = cand
                    break
        missing = [s.name for s in HUMAN_HAND if s.name not in self.links]
        if missing:
            raise ValueError(f"{urdf_path}: 사람 골격 링크 없음 {missing}")

        self.palm_link = next(
            (self.side + "_" + n for n in PALM_LINKS if self._has(self.side + "_" + n)),
            None)
        if self.palm_link is None:
            raise ValueError(
                f"{urdf_path}: 팜 기준 링크 없음 ({self.side}_ + {PALM_LINKS})")

        self.keypoints = {f: [self.links[n] for n in BONE_LINKS[f]] for f in FINGERS}
        self._fids = {name: self.model.getFrameId(link, pin.FrameType.BODY)
                      for name, link in self.links.items()}
        self._palm_fid = self.model.getFrameId(self.palm_link, pin.FrameType.BODY)

    def _has(self, name: str) -> bool:
        return self.model.existFrame(name, pin.FrameType.BODY)

    def q_from_named(self, angles: Mapping[str, float]) -> np.ndarray:
        q = pin.neutral(self.model)
        pfx = self.side + "_"
        for name, val in angles.items():
            iq = self._idx_q.get(name, self._idx_q.get(pfx + name))
            if iq is not None:
                q[iq] = float(val)
        return q

    def _fk(self, angles) -> None:
        q = angles if isinstance(angles, np.ndarray) else self.q_from_named(angles)
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)

    def _pos(self, fid: int) -> np.ndarray:
        return self.data.oMf[fid].translation.copy()

    def points(self, angles) -> Dict[str, np.ndarray]:
        self._fk(angles)
        out = {f: np.array([self._pos(self._fids[n]) for n in BONE_LINKS[f]])
               for f in FINGERS}
        out["palm"] = self._pos(self._palm_fid)
        return out

    def positions(self, angles) -> np.ndarray:
        self._fk(angles)
        out = np.zeros((len(HUMAN_HAND), 3), dtype=float)
        for spec in HUMAN_HAND:
            out[JOINT_INDEX[spec.name]] = self._pos(self._fids[spec.name])
        return out

    def neutral_points(self) -> Dict[str, np.ndarray]:
        return self.points(pin.neutral(self.model))
