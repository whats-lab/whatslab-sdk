from typing import Dict, List, Sequence, Tuple

import numpy as np
import pinocchio as pin

from .human_fk import (BONE_LINKS, FINGERS, HumanHandFK, link_candidates,
                       palm_frame_from_fingers)

KV_DIM = 6
MIRROR_Z = np.array([1.0, 1.0, -1.0] * 2)
REACH_SAMPLES = 25
REACH_PASSES = 4


def chain_weights(seg_lengths: Sequence[float], frac: float = 0.5) -> Tuple[int, float]:
    lengths = np.asarray(seg_lengths, dtype=float)
    if lengths.size == 0 or not np.all(lengths >= 0.0):
        raise ValueError("사슬 구간 길이가 비었거나 음수다: %s" % (lengths,))
    total = float(lengths.sum())
    if total <= 1e-12:
        raise ValueError("사슬 전체 길이가 0 이다 — 같은 위치의 프레임을 사슬로 줬다")
    target = total * float(frac)
    acc = 0.0
    for k, seg in enumerate(lengths):
        if seg > 0.0 and acc + seg >= target - 1e-12:
            return int(k), float(min(max((target - acc) / seg, 0.0), 1.0))
        acc += seg
    return int(lengths.size) - 1, 1.0


def human_chains(fk: HumanHandFK) -> Dict[str, List[str]]:
    out = {}
    for f in FINGERS:
        names = []
        for jn in BONE_LINKS[f]:
            cand = [c for c in link_candidates(fk.side, jn) if fk.model.existFrame(c)]
            if not cand:
                raise ValueError("사람 URDF 에 %s 사슬 링크가 없다: %s"
                                 % (f, link_candidates(fk.side, jn)))
            names.append(cand[0])
        out[f] = names
    return out


def sensor_chains(model, side: str, n_links: int = 3) -> Dict[str, List[str]]:
    tips = {f: "%s_sensor_%s_distal" % (side, f) for f in FINGERS}
    missing = [n for n in tips.values()
               if not model.existFrame(n, pin.FrameType.BODY)]
    if missing:
        raise ValueError("URDF 에 센서 프레임이 없다: %s" % missing)
    out = {}
    for f, tip in tips.items():
        jid = int(model.frames[model.getFrameId(tip, pin.FrameType.BODY)].parent)
        links = []
        for j in [int(j) for j in model.supports[jid] if j > 0]:
            bodies = [fr.name for fr in model.frames
                      if fr.type == pin.FrameType.BODY and int(fr.parent) == j
                      and "_sensor_" not in fr.name]
            if bodies:
                links.append(bodies[0])
        if len(links) < n_links:
            raise ValueError("%s 사슬 링크가 %d 개뿐이다 (%d 개 필요): %s"
                             % (f, len(links), n_links, links))
        out[f] = links[-n_links:] + [tip]
    return out


def finger_columns(model, tip_fids: Dict[str, int]) -> Dict[str, List[int]]:
    cols = {}
    for f, fid in tip_fids.items():
        jid = int(model.frames[fid].parent)
        cols[f] = {int(model.joints[j].idx_v) for j in model.supports[jid] if j > 0}
    shared = set.intersection(*cols.values()) if len(cols) > 1 else set()
    return {f: sorted(v - shared) for f, v in cols.items()}


class HandKeyvector:

    def __init__(self, model, data, chains: Dict[str, Sequence[str]],
                 dorsum_frame: str, frac: float = 0.5):
        self.model = model
        self.data = data
        self.fids = {f: [self._bid(n) for n in chains[f]] for f in FINGERS}
        for f in FINGERS:
            if len(self.fids[f]) < 2:
                raise ValueError("%s 사슬이 2점 미만이다: %s" % (f, list(chains[f])))
        self.dorsum = self._bid(dorsum_frame)

        pts = self.points(pin.neutral(model))
        self.mid = {f: chain_weights(
            np.linalg.norm(np.diff(pts[f], axis=0), axis=1), frac) for f in FINGERS}
        self.origin = self._pos(self.dorsum).copy()
        self.rot = palm_frame_from_fingers(pts)[1]
        self.l_ref = max(self._reach(f) for f in FINGERS)
        if self.l_ref <= 1e-9:
            raise ValueError("L_ref 가 0 이다 — dorsum 과 모든 손끝이 같은 위치다")

    def _reach(self, finger: str) -> float:
        lo = self.model.lowerPositionLimit
        hi = self.model.upperPositionLimit
        idx = [int(self.model.joints[j].idx_q)
               for j in self.model.supports[
                   self.model.frames[self.fids[finger][-1]].parent]
               if j > 0 and self.model.joints[j].nq > 0]
        q = pin.neutral(self.model)
        for i in idx:
            q[i] = 0.0 if lo[i] <= 0.0 <= hi[i] else (
                lo[i] if abs(lo[i]) < abs(hi[i]) else hi[i])

        def dist(qv):
            return float(np.linalg.norm(
                self.points(qv)[finger][-1] - self._pos(self.dorsum)))

        best = dist(q)
        for _ in range(REACH_PASSES):
            improved = False
            for i in idx:
                a, b = max(float(lo[i]), -np.pi), min(float(hi[i]), np.pi)
                for v in np.linspace(a, b, REACH_SAMPLES):
                    qq = q.copy()
                    qq[i] = v
                    d = dist(qq)
                    if d > best + 1e-12:
                        best, q, improved = d, qq, True
            if not improved:
                break
        return best

    def _bid(self, name: str) -> int:
        if not self.model.existFrame(name):
            raise ValueError("URDF 에 프레임이 없다: %s" % name)
        return self.model.getFrameId(name, pin.FrameType.BODY)

    def _pos(self, fid: int) -> np.ndarray:
        return self.data.oMf[fid].translation

    def points(self, q: np.ndarray) -> Dict[str, np.ndarray]:
        pin.forwardKinematics(self.model, self.data, np.asarray(q, dtype=float))
        pin.updateFramePlacements(self.model, self.data)
        return {f: np.array([self._pos(i).copy() for i in self.fids[f]])
                for f in FINGERS}

    def prox(self, pts: Dict[str, np.ndarray], finger: str) -> np.ndarray:
        k, t = self.mid[finger]
        return pts[finger][k] * (1.0 - t) + pts[finger][k + 1] * t

    def local(self, p: np.ndarray) -> np.ndarray:
        return self.rot.T @ (np.asarray(p, dtype=float) - self.origin) / self.l_ref

    def encode(self, q: np.ndarray) -> np.ndarray:
        pts = self.points(q)
        out = np.zeros((len(FINGERS), KV_DIM))
        for i, f in enumerate(FINGERS):
            out[i, :3] = self.local(pts[f][-1])
            out[i, 3:] = self.local(self.prox(pts, f))
        return out

    def jacobian(self, q: np.ndarray, idx_v: Sequence[int]) -> np.ndarray:
        pin.computeJointJacobians(self.model, self.data, np.asarray(q, dtype=float))
        pin.updateFramePlacements(self.model, self.data)
        cols = np.asarray(idx_v, dtype=int)
        out = np.zeros((len(FINGERS), KV_DIM, cols.size))
        for i, f in enumerate(FINGERS):
            k, t = self.mid[f]
            jt = self._frame_jac(self.fids[f][-1])
            jp = (self._frame_jac(self.fids[f][k]) * (1.0 - t)
                  + self._frame_jac(self.fids[f][k + 1]) * t)
            out[i, :3] = (self.rot.T @ jt)[:, cols] / self.l_ref
            out[i, 3:] = (self.rot.T @ jp)[:, cols] / self.l_ref
        return out

    def _frame_jac(self, fid: int) -> np.ndarray:
        return pin.getFrameJacobian(self.model, self.data, fid,
                                    pin.LOCAL_WORLD_ALIGNED)[:3]
