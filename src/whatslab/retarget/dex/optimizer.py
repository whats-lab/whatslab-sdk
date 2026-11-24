from __future__ import annotations

import xml.etree.ElementTree as ET
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import numpy as np
import pinocchio as pin
from scipy.optimize import OptimizeResult, minimize

DEXPILOT_HUBER_DELTA = 0.03
DEXPILOT_ETA1 = 1e-4
DEXPILOT_ETA2 = 3e-2
DEXPILOT_PAIR_WEIGHT = 200.0
DEXPILOT_SECOND_PAIR_WEIGHT = 400.0
DEXPILOT_SECOND_PAIR_DIST = 0.03
LIMIT_EPSILON = 1e-3
FTOL = 1e-6
MAX_ITER = 100
USABLE_STATUS = (0, 9)


class HandKinematics:
    def __init__(self, urdf_path: str):
        self.model = pin.buildModelFromUrdf(urdf_path)
        self.data = self.model.createData()
        if self.model.nq != self.model.nv:
            raise ValueError(f"{urdf_path}: nq != nv 인 관절(continuous 등)은 다루지 않는다")
        self.joint_names: List[str] = [
            n for i, n in enumerate(self.model.names) if self.model.nqs[i] > 0
        ]
        self.joint_limits = np.stack(
            [self.model.lowerPositionLimit, self.model.upperPositionLimit], axis=1
        )
        direct = {
            j.get("name"): (m.get("joint"), float(m.get("multiplier", 1.0)),
                            float(m.get("offset", 0.0)))
            for j in ET.parse(urdf_path).getroot().iter("joint")
            for m in j.findall("mimic")
        }
        mimic = []
        for name, (source, k, offset) in direct.items():
            while source in direct:
                source, k2, o2 = direct[source]
                k, offset = k * k2, k * o2 + offset
            mimic.append((name, source, k, offset))
        self.mimic_names = [m[0] for m in mimic]
        self.mimic_source_names = [m[1] for m in mimic]
        self.idx_mimic = np.array([self.joint_names.index(m[0]) for m in mimic], dtype=int)
        self.idx_source = np.array([self.joint_names.index(m[1]) for m in mimic], dtype=int)
        self.mimic_multipliers = np.array([m[2] for m in mimic])
        self.mimic_offsets = np.array([m[3] for m in mimic])

    def add_offset_frame(self, link: str, offset: Sequence[float]) -> str:
        name = f"{link}_offset"
        if self.model.existFrame(name):
            raise ValueError(f"URDF 에 이미 있는 프레임 이름: {name}")
        parent = self.frame_ids([link])[0]
        frame = self.model.frames[parent]
        self.model.addFrame(pin.Frame(
            name, frame.parentJoint, parent,
            frame.placement * pin.SE3(np.eye(3), np.asarray(offset, dtype=float)), pin.BODY))
        self.data = self.model.createData()
        return name

    @property
    def dof(self) -> int:
        return self.model.nq

    def frame_ids(self, names: Sequence[str]) -> List[int]:
        missing = [n for n in names if not self.model.existFrame(n, pin.BODY)]
        if missing:
            raise ValueError(f"URDF 에 없는 링크: {missing}")
        return [self.model.getFrameId(n, pin.BODY) for n in names]

    def forward_qpos(self, qpos: np.ndarray) -> np.ndarray:
        qpos[self.idx_mimic] = qpos[self.idx_source] * self.mimic_multipliers + self.mimic_offsets
        return qpos

    def frames(self, qpos: np.ndarray, frame_ids: Sequence[int]) -> Tuple[np.ndarray, np.ndarray]:
        pin.computeJointJacobians(self.model, self.data, qpos)
        pos = np.array(
            [pin.updateFramePlacement(self.model, self.data, f).translation for f in frame_ids]
        )
        jac = np.array([
            pin.getFrameJacobian(self.model, self.data, f, pin.LOCAL_WORLD_ALIGNED)[:3]
            for f in frame_ids
        ])
        return pos, jac


def huber_vector_loss(
    robot_vec: np.ndarray, target_vec: np.ndarray, weight: np.ndarray, delta: np.ndarray
) -> Tuple[float, np.ndarray]:
    diff = robot_vec - target_vec
    dist = np.linalg.norm(diff, axis=1)
    huber = np.where(dist < delta, 0.5 * dist * dist / delta, dist - 0.5 * delta)
    return float(np.sum(weight * huber)), (weight / np.maximum(dist, delta))[:, None] * diff


def dexpilot_pairs(num_fingers: int) -> Tuple[List[int], List[int]]:
    origin = [j for i in range(1, num_fingers) for j in range(i + 1, num_fingers + 1)]
    task = [i for i in range(1, num_fingers) for _ in range(i + 1, num_fingers + 1)]
    return origin + [0] * num_fingers, task + list(range(1, num_fingers + 1))


@dataclass
class VectorTerm:
    origin_links: List[str]
    task_links: List[str]
    weight: float
    huber_delta: float


class DexPilotState:
    def __init__(self, num_fingers: int, project_dist: float, escape_dist: float):
        if not 2 <= num_fingers <= 5:
            raise ValueError(f"DexPilot 은 손가락 2~5개만 된다 — 받은 수 {num_fingers}")
        self.num_fingers = num_fingers
        self.project_dist = project_dist
        self.escape_dist = escape_dist
        self.num_pairs = num_fingers * (num_fingers - 1) // 2
        self.num_thumb = num_fingers - 1
        pairs = [(i, j) for i in range(num_fingers - 2) for j in range(i + 1, num_fingers - 1)]
        self.second_task = np.array([p[0] for p in pairs], dtype=int)
        self.second_origin = np.array([p[1] for p in pairs], dtype=int)
        self.projected_dist = np.array(
            [DEXPILOT_ETA1] * self.num_thumb
            + [DEXPILOT_ETA2] * (self.num_pairs - self.num_thumb)
        )
        self.projected = np.zeros(self.num_pairs, dtype=bool)

    @property
    def num_rows(self) -> int:
        return self.num_pairs + self.num_fingers

    def reset(self) -> None:
        self.projected[:] = False

    def update(self, ref: np.ndarray, scaling: float) -> Tuple[np.ndarray, np.ndarray]:
        n, s1 = self.num_pairs, self.num_thumb
        dist = np.linalg.norm(ref[:n], axis=1)
        thumb = self.projected[:s1]
        thumb[dist[:s1] < self.project_dist] = True
        thumb[dist[:s1] > self.escape_dist] = False
        self.projected[s1:] = (
            thumb[self.second_origin]
            & thumb[self.second_task]
            & (dist[s1:] <= DEXPILOT_SECOND_PAIR_DIST)
        )
        high = np.array(
            [DEXPILOT_PAIR_WEIGHT] * s1 + [DEXPILOT_SECOND_PAIR_WEIGHT] * (n - s1)
        )
        weight = np.concatenate(
            [np.where(self.projected, high, 1.0), np.full(self.num_fingers, n + self.num_fingers)]
        )
        normal = ref * scaling
        projected = ref[:n] / (dist[:, None] + 1e-6) * self.projected_dist[:, None]
        target = np.concatenate([np.where(self.projected[:, None], projected, normal[:n]),
                                 normal[n:]])
        return target.astype(np.float32), weight / self.num_rows


class RetargetOptimizer:
    def __init__(
        self,
        kin: HandKinematics,
        target_joint_names: Sequence[str],
        base_qpos: np.ndarray,
        dexpilot_links: Tuple[str, List[str]] | None,
        vector_terms: Sequence[VectorTerm],
        scaling: float,
        normal_delta: float,
        project_dist: float,
        escape_dist: float,
    ):
        mimic = set(kin.mimic_names) & set(target_joint_names)
        if mimic:
            raise ValueError(
                f"mimic 관절은 target_joint_names 에 넣지 않는다: {sorted(mimic)} — "
                "target_joint_names 를 구동 관절만으로 직접 준다"
            )
        unknown = [n for n in target_joint_names if n not in kin.joint_names]
        if unknown:
            raise ValueError(f"URDF 에 없는 관절: {unknown}")
        self.kin = kin
        self.target_joint_names = list(target_joint_names)
        self.idx_target = np.array(
            [kin.joint_names.index(n) for n in target_joint_names], dtype=int
        )
        n_target = len(self.idx_target)
        chain = np.zeros((kin.dof, n_target))
        chain[self.idx_target, np.arange(n_target)] = 1.0
        for m, s, k in zip(kin.idx_mimic, kin.mimic_source_names, kin.mimic_multipliers):
            chain[m, self.target_joint_names.index(s)] += k
        self.chain = chain
        self.base_qpos = np.asarray(base_qpos, dtype=float)
        self.scaling = scaling
        self.normal_delta = normal_delta

        origins: List[str] = []
        tasks: List[str] = []
        self.dexpilot = None
        if dexpilot_links is not None:
            wrist, tips = dexpilot_links
            self.dexpilot = DexPilotState(len(tips), project_dist, escape_dist)
            links = [wrist] + list(tips)
            o, t = dexpilot_pairs(len(tips))
            origins += [links[i] for i in o]
            tasks += [links[i] for i in t]
        self.num_dexpilot_rows = len(origins)
        weights, deltas = [], [DEXPILOT_HUBER_DELTA] * len(origins)
        for term in vector_terms:
            m = len(term.origin_links)
            origins += term.origin_links
            tasks += term.task_links
            weights += [term.weight / m] * m
            deltas += [term.huber_delta] * m
        self.vector_weight = np.array(weights)
        self.huber_delta = np.array(deltas)
        links = list(dict.fromkeys(origins + tasks))
        self.frame_ids = kin.frame_ids(links)
        self.origin_idx = np.array([links.index(n) for n in origins], dtype=int)
        self.task_idx = np.array([links.index(n) for n in tasks], dtype=int)

        self.bounds: Optional[np.ndarray] = None
        self.max_iter = MAX_ITER
        self.last_result: Optional[OptimizeResult] = None

    @property
    def num_rows(self) -> int:
        return len(self.origin_idx)

    def set_joint_limits(self, limits: np.ndarray) -> None:
        self.bounds = np.stack([limits[:, 0] - LIMIT_EPSILON, limits[:, 1] + LIMIT_EPSILON], 1)

    def set_max_evals(self, max_evals: int) -> None:
        self.max_iter = max_evals

    def reset(self) -> None:
        if self.dexpilot is not None:
            self.dexpilot.reset()

    def targets(self, ref: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        ref = np.asarray(ref, dtype=np.float32)
        n = self.num_dexpilot_rows
        vec_target = (ref[n:] * self.scaling).astype(np.float32)
        if self.dexpilot is None:
            return vec_target, self.vector_weight
        dex_target, dex_weight = self.dexpilot.update(ref[:n], self.scaling)
        return (np.concatenate([dex_target, vec_target]),
                np.concatenate([dex_weight, self.vector_weight]))

    def full_qpos(self, x: np.ndarray) -> np.ndarray:
        qpos = self.base_qpos.copy()
        qpos[self.idx_target] = x
        return self.kin.forward_qpos(qpos)

    def objective(self, ref: np.ndarray, center: np.ndarray):
        target, weight = self.targets(ref)
        target = target.astype(float)
        center = np.asarray(center, dtype=np.float32)

        def objective(x: np.ndarray) -> Tuple[float, np.ndarray]:
            pos, jac = self.kin.frames(self.full_qpos(x), self.frame_ids)
            robot_vec = pos[self.task_idx] - pos[self.origin_idx]
            loss, d_vec = huber_vector_loss(robot_vec, target, weight, self.huber_delta)
            d_pos = np.zeros_like(pos)
            np.add.at(d_pos, self.task_idx, d_vec)
            np.add.at(d_pos, self.origin_idx, -d_vec)
            grad = (np.einsum("lk,lkj->j", d_pos, jac) @ self.chain
                    + 2 * self.normal_delta * (x - center))
            return loss, grad

        return objective

    def solve(self, ref: np.ndarray, start: np.ndarray) -> Optional[np.ndarray]:
        self.last_result = minimize(
            self.objective(ref, start),
            np.asarray(start, dtype=float),
            method="SLSQP",
            jac=True,
            bounds=self.bounds,
            options={"maxiter": self.max_iter, "ftol": FTOL},
        )
        x = self.last_result.x
        if self.last_result.status not in USABLE_STATUS or not np.all(np.isfinite(x)):
            return None
        return x.astype(np.float32)
