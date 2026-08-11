from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np
import pinocchio as pin

from .human_fk import (BONE_LINKS, FINGERS, HumanHandFK, link_candidates,
                       non_thumb, palm_frame_from_fingers)

KV_DIM = 6
FRAME_DIM = 24
MIRROR_Z = np.array([1.0, 1.0, -1.0] * 2)
MIRROR_M = np.diag([1.0, 1.0, -1.0])


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


def human_chains(fk: HumanHandFK,
                 fingers: Optional[Sequence[str]] = None) -> Dict[str, List[str]]:
    out = {}
    for f in (FINGERS if fingers is None else fingers):
        names = []
        for jn in BONE_LINKS[f]:
            cand = [c for c in link_candidates(fk.side, jn) if fk.model.existFrame(c)]
            if not cand:
                raise ValueError("사람 URDF 에 %s 사슬 링크가 없다: %s"
                                 % (f, link_candidates(fk.side, jn)))
            names.append(cand[0])
        out[f] = names
    return out


def sensor_prox(model, side: str,
                fingers: Optional[Sequence[str]] = None) -> Dict[str, str]:
    out = {}
    for f in (FINGERS if fingers is None else fingers):
        name = "%s_sensor_%s_proximal" % (side, f)
        if not model.existFrame(name, pin.FrameType.BODY):
            raise ValueError("URDF 에 proximal 센서 프레임이 없다: %s" % name)
        out[f] = name
    return out


def sensor_chains(model, side: str, n_links: int = 3,
                  fingers: Optional[Sequence[str]] = None) -> Dict[str, List[str]]:
    tips = {f: "%s_sensor_%s_distal" % (side, f)
            for f in (FINGERS if fingers is None else fingers)}
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
                 dorsum_frame: str, prox_frames: Dict[str, str]):
        self.model = model
        self.data = data
        self.fingers = [f for f in FINGERS if f in chains]
        if len(self.fingers) != len(chains):
            raise ValueError("사슬에 알 수 없는 손가락 이름이 있다: %s"
                             % sorted(set(chains) - set(FINGERS)))
        self.fids = {f: [self._bid(n) for n in chains[f]] for f in self.fingers}
        for f in self.fingers:
            if len(self.fids[f]) < 2:
                raise ValueError("%s 사슬이 2점 미만이다: %s" % (f, list(chains[f])))
        self.dorsum = self._bid(dorsum_frame)

        self.prox_fids = {f: self._bid(prox_frames[f]) for f in self.fingers}
        pts = self.points(pin.neutral(model))
        self.origin = self._pos(self.dorsum).copy()
        palm_o, self.rot = palm_frame_from_fingers(pts)
        self.ref_finger = ("middle" if "middle" in self.fingers
                           else non_thumb(self.fingers)[0])
        self.l_ref = float(np.linalg.norm(
            self.rot.T @ (pts[self.ref_finger][-1] - palm_o)))
        if self.l_ref <= 1e-9:
            raise ValueError("L_ref 가 0 이다 — 팜 원점과 %s 끝이 같은 위치다"
                             % self.ref_finger)

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
                for f in self.fingers}

    def prox(self, pts: Dict[str, np.ndarray], finger: str) -> np.ndarray:
        return self._pos(self.prox_fids[finger]).copy()

    def local(self, p: np.ndarray) -> np.ndarray:
        return self.rot.T @ (np.asarray(p, dtype=float) - self.origin) / self.l_ref

    def encode(self, q: np.ndarray) -> np.ndarray:
        pts = self.points(q)
        out = np.zeros((len(self.fingers), KV_DIM))
        for i, f in enumerate(self.fingers):
            out[i, :3] = self.local(pts[f][-1])
            out[i, 3:] = self.local(self.prox(pts, f))
        return out

    def rot_of(self, fid: int) -> np.ndarray:
        return self.rot.T @ self.data.oMf[fid].rotation

    def encode_frames(self, q: np.ndarray) -> np.ndarray:
        pts = self.points(q)
        out = np.zeros((len(self.fingers), FRAME_DIM))
        for i, f in enumerate(self.fingers):
            out[i, 0:3] = self.local(pts[f][-1])
            out[i, 3:12] = self.rot_of(self.fids[f][-1]).reshape(9)
            out[i, 12:15] = self.local(self.prox(pts, f))
            out[i, 15:24] = self.rot_of(self.prox_fids[f]).reshape(9)
        return out

    def jacobian(self, q: np.ndarray, idx_v: Sequence[int]) -> np.ndarray:
        pin.computeJointJacobians(self.model, self.data, np.asarray(q, dtype=float))
        pin.updateFramePlacements(self.model, self.data)
        cols = np.asarray(idx_v, dtype=int)
        out = np.zeros((len(self.fingers), KV_DIM, cols.size))
        for i, f in enumerate(self.fingers):
            jt = self._frame_jac(self.fids[f][-1])
            jp = self._frame_jac(self.prox_fids[f])
            out[i, :3] = (self.rot.T @ jt)[:, cols] / self.l_ref
            out[i, 3:] = (self.rot.T @ jp)[:, cols] / self.l_ref
        return out

    def _frame_jac(self, fid: int) -> np.ndarray:
        return pin.getFrameJacobian(self.model, self.data, fid,
                                    pin.LOCAL_WORLD_ALIGNED)[:3]


def mirror_frames(x: np.ndarray) -> np.ndarray:
    out = np.array(x, dtype=float, copy=True)
    for a, b in ((0, 3), (12, 15)):
        out[..., a:b] = out[..., a:b] * np.array([1.0, 1.0, -1.0])
    for a in (3, 15):
        r = out[..., a:a + 9].reshape(out.shape[:-1] + (3, 3))
        out[..., a:a + 9] = (MIRROR_M @ r @ MIRROR_M).reshape(
            out.shape[:-1] + (9,))
    return out
