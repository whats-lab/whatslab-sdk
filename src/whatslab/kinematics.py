from __future__ import annotations

import xml.etree.ElementTree as ET
from typing import Dict, List, Sequence, Tuple

import numpy as np

MOVABLE = ("revolute", "continuous", "prismatic")


def _vec(el, attr: str, default: str) -> np.ndarray:
    text = default if el is None else el.get(attr, default)
    return np.array([float(v) for v in text.split()])


def rpy_matrix(rpy: Sequence[float]) -> np.ndarray:
    (cr, cp, cy), (sr, sp, sy) = np.cos(rpy), np.sin(rpy)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def origin_matrix(el) -> np.ndarray:
    tf = np.eye(4)
    tf[:3, :3] = rpy_matrix(_vec(el, "rpy", "0 0 0"))
    tf[:3, 3] = _vec(el, "xyz", "0 0 0")
    return tf


def _skew(a: np.ndarray) -> np.ndarray:
    k = np.zeros((len(a), 3, 3))
    k[:, 0, 1], k[:, 0, 2], k[:, 1, 2] = -a[:, 2], a[:, 1], -a[:, 0]
    return k - k.transpose(0, 2, 1)


class Kinematics:
    def __init__(self, urdf_path: str):
        self._build(ET.parse(urdf_path).getroot())

    @classmethod
    def from_string(cls, text: str) -> "Kinematics":
        kin = cls.__new__(cls)
        kin._build(ET.fromstring(text))
        return kin

    def _build(self, robot) -> None:
        joints = robot.findall("joint")
        by_parent: Dict[str, List] = {}
        child_links = set()
        for j in joints:
            kind = j.get("type")
            if kind not in MOVABLE + ("fixed",):
                raise ValueError(f"관절 {j.get('name')}: 다루지 않는 type {kind!r}")
            by_parent.setdefault(j.find("parent").get("link"), []).append(j)
            child_links.add(j.find("child").get("link"))
        roots = [ln.get("name") for ln in robot.findall("link")
                 if ln.get("name") not in child_links]
        if len(roots) != 1:
            raise ValueError(f"루트 링크가 하나가 아니다: {roots}")

        self.link_names: List[str] = []
        self._link_parent_list: List[int] = []
        self._link_offset_list: List[np.ndarray] = []
        self.joint_names: List[str] = []
        self.joint_parents: List[int] = []
        origins, axes, prismatic, lower, upper = [], [], [], [], []
        self._add_link(roots[0], -1, np.eye(4))
        stack = self._children(by_parent, roots[0], -1, np.eye(4))
        while stack:
            j, parent, offset = stack.pop()
            tf = offset @ origin_matrix(j.find("origin"))
            child = j.find("child").get("link")
            if j.get("type") == "fixed":
                self._add_link(child, parent, tf)
                stack += self._children(by_parent, child, parent, tf)
                continue
            limit = j.find("limit")
            bounded = j.get("type") != "continuous" and limit is not None
            axis = _vec(j.find("axis"), "xyz", "1 0 0")
            self.joint_names.append(j.get("name"))
            self.joint_parents.append(parent)
            origins.append(tf)
            axes.append(axis / np.linalg.norm(axis))
            prismatic.append(j.get("type") == "prismatic")
            lower.append(float(limit.get("lower", 0.0)) if bounded else -np.inf)
            upper.append(float(limit.get("upper", 0.0)) if bounded else np.inf)
            idx = len(self.joint_names) - 1
            self._add_link(child, idx, np.eye(4))
            stack += self._children(by_parent, child, idx, np.eye(4))

        n = len(self.joint_names)
        self._link_index = {name: i for i, name in enumerate(self.link_names)}
        self._link_parent = np.array(self._link_parent_list, dtype=int) + 1
        self._link_offset = np.array(self._link_offset_list)
        del self._link_parent_list, self._link_offset_list
        self._origin = np.array(origins).reshape(-1, 4, 4)
        axis = np.array(axes).reshape(-1, 3)
        self._prismatic = np.array(prismatic, dtype=bool)
        self._axis_slide = axis * self._prismatic[:, None]
        self._axis_turn = axis * ~self._prismatic[:, None]
        self._k = _skew(self._axis_turn)
        basis = np.zeros((n, 4, 4, 3))
        basis[:, :3, :3, 0] = self._k
        basis[:, :3, :3, 1] = self._k @ self._k
        basis[:, :3, 3, 2] = self._axis_slide
        self._basis = np.einsum("nab,nbcj->nacj", self._origin, basis).reshape(n, 16, 3)
        self._slides = bool(self._prismatic.any())
        parents = np.array(self.joint_parents, dtype=int) + 1
        jump = np.concatenate([[0], parents])
        self._jumps = []
        while jump.any():
            self._jumps.append(jump)
            jump = jump[jump]
        self.joint_limits = np.stack([lower, upper], axis=1).reshape(-1, 2)

        ancestors = np.zeros((n + 1, n), dtype=bool)
        for i, p in enumerate(parents):
            ancestors[i + 1] = ancestors[p]
            ancestors[i + 1, i] = True
        self._mask = ancestors[self._link_parent].T[:, None, :]
        self._resolve_mimic(joints)

    def _add_link(self, name: str, parent: int, offset: np.ndarray) -> None:
        self.link_names.append(name)
        self._link_parent_list.append(parent)
        self._link_offset_list.append(offset)

    @staticmethod
    def _children(by_parent, link: str, parent: int, offset: np.ndarray) -> List:
        joints = sorted(by_parent.get(link, []), key=lambda e: e.get("name"), reverse=True)
        return [(j, parent, offset) for j in joints]

    def _resolve_mimic(self, joints) -> None:
        mimic = {}
        for j in joints:
            m = j.find("mimic")
            if m is not None:
                mimic[j.get("name")] = (m.get("joint"), float(m.get("multiplier", 1.0)),
                                        float(m.get("offset", 0.0)))
        resolved = []
        for name in mimic:
            src, mult, off = mimic[name]
            while src in mimic:
                src, k, o = mimic[src]
                mult, off = mult * k, mult * o + off
            resolved.append((name, src, mult, off))
        self.mimic_names = [m[0] for m in resolved]
        self.mimic_source_names = [m[1] for m in resolved]
        self.idx_mimic = np.array([self.joint_names.index(m[0]) for m in resolved], dtype=int)
        self.idx_source = np.array([self.joint_names.index(m[1]) for m in resolved], dtype=int)
        self.mimic_multipliers = np.array([m[2] for m in resolved])
        self.mimic_offsets = np.array([m[3] for m in resolved])
        self.actuated_joint_names = [n for n in self.joint_names if n not in mimic]

    @property
    def dof(self) -> int:
        return len(self.joint_names)

    def neutral(self) -> np.ndarray:
        return np.zeros(self.dof)

    def frame_ids(self, names: Sequence[str]) -> List[int]:
        missing = [n for n in names if n not in self._link_index]
        if missing:
            raise ValueError(f"URDF 에 없는 링크: {missing}")
        return [self._link_index[n] for n in names]

    def forward_qpos(self, qpos: np.ndarray) -> np.ndarray:
        qpos[self.idx_mimic] = qpos[self.idx_source] * self.mimic_multipliers + self.mimic_offsets
        return qpos

    def _world(self, q: np.ndarray) -> np.ndarray:
        q = np.asarray(q, dtype=float)
        coef = np.stack([np.sin(q), 1.0 - np.cos(q), q], axis=1)[:, :, None]
        world = np.empty((self.dof + 1, 4, 4))
        world[0] = np.eye(4)
        world[1:] = self._origin + (self._basis @ coef).reshape(-1, 4, 4)
        for jump in self._jumps:
            world = world[jump] @ world
        return world

    def joint_frames(self, q: np.ndarray) -> np.ndarray:
        return self._world(q)[1:]

    def link_placements(self, q: np.ndarray) -> np.ndarray:
        return self._world(q)[self._link_parent] @ self._link_offset

    def _jacobian(self, q: np.ndarray, frame_ids: Sequence[int], angular: bool):
        world = self._world(q)
        ids = np.asarray(frame_ids, dtype=int)
        tf = world[self._link_parent[ids]] @ self._link_offset[ids]
        rot = world[1:, :3, :3]
        skew = rot @ self._k @ rot.transpose(0, 2, 1)
        jac = skew @ (tf[:, :3, 3].T) - skew @ world[1:, :3, 3:]
        if self._slides:
            jac += rot @ self._axis_slide[:, :, None]
        if angular:
            turn = rot @ self._axis_turn[:, :, None]
            jac = np.concatenate([jac, np.broadcast_to(turn, jac.shape)], axis=1)
        jac = jac * self._mask[:, :, ids]
        return tf, jac.transpose(2, 1, 0)

    def jacobians(self, q: np.ndarray, frame_ids: Sequence[int]) -> Tuple[np.ndarray, np.ndarray]:
        return self._jacobian(q, frame_ids, True)

    def frames(self, qpos: np.ndarray, frame_ids: Sequence[int]) -> Tuple[np.ndarray, np.ndarray]:
        tf, jac = self._jacobian(qpos, frame_ids, False)
        return tf[:, :3, 3], jac
