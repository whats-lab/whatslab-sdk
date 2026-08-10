from __future__ import annotations

import os
from typing import Dict, List, Optional, Sequence

import numpy as np
import pinocchio as pin

from whatslab.core.types import JOINT_INDEX, SENSED_JOINTS

from .hand_configs import CONFIG_REGISTRY
from .spherical_fk import HandSphericalFK

FINGERS = ("thumb", "index", "middle", "ring", "pinky")
HUMAN_KEYPOINTS: Dict[str, Sequence[str]] = {
    "thumb": ("thumb_cmc1", "thumb_mcp", "thumb_ip", "thumb_tip"),
    "index": ("index_mcp", "index_pip", "index_dip", "index_tip"),
    "middle": ("middle_mcp", "middle_pip", "middle_dip", "middle_tip"),
    "ring": ("ring_mcp", "ring_pip", "ring_dip", "ring_tip"),
    "pinky": ("pinky_mcp", "pinky_pip", "pinky_dip", "pinky_tip"),
}
PAIRS = tuple(("thumb", f) for f in ("index", "middle", "ring", "pinky"))
SEP_PAIRS = (("index", "middle"), ("middle", "ring"))

SNAP_ENGAGE = 0.03
SNAP_FULL = 0.01
SNAP_CONTACT = 1e-4
SEP_MIN = 0.03
HUBER_DELTA = 0.02
DQ_MAX_STEP = 0.4


def _rot_between(u: np.ndarray, v: np.ndarray) -> np.ndarray:
    u = u / max(np.linalg.norm(u), 1e-12)
    v = v / max(np.linalg.norm(v), 1e-12)
    c = np.cross(u, v)
    d = float(u @ v)
    if np.linalg.norm(c) < 1e-12:
        return np.eye(3) if d > 0 else -np.eye(3)
    K = np.array([[0, -c[2], c[1]], [c[2], 0, -c[0]], [-c[1], c[0], 0]])
    return np.eye(3) + K + K @ K * ((1 - d) / (np.linalg.norm(c) ** 2))


def _palm_frame(base_pts: Dict[str, np.ndarray]):
    o = np.mean([base_pts[f] for f in ("index", "middle", "ring", "pinky")], axis=0)
    x = base_pts["index"] - base_pts["pinky"]
    x = x / np.linalg.norm(x)
    y = base_pts["middle"] - o
    y = y - x * (y @ x)
    y = y / np.linalg.norm(y)
    return o, np.column_stack([x, y, np.cross(x, y)])


def _huber_w(w: float, r_norm: float) -> float:
    return w if r_norm <= HUBER_DELTA else w * float(np.sqrt(HUBER_DELTA / r_norm))


class KPHandRetargeter:

    def __init__(self, hand_type: str, config_name: str = "base_hand",
                 urdf_root=None, keypoints: Optional[Dict[str, List[str]]] = None,
                 w_tip: float = 2.0, w_shape: Optional[float] = None,
                 w_pair: float = 6.0, w_snap: float = 160.0, w_sep: float = 80.0,
                 iters_per_call: int = 8, cold_iters: int = 60):
        if config_name not in CONFIG_REGISTRY:
            raise ValueError(
                f"Unknown robot_config '{config_name}'. Available: {list(CONFIG_REGISTRY.keys())}"
            )
        config = CONFIG_REGISTRY[config_name](urdf_root=urdf_root)

        self.hand_type = hand_type.lower()
        mroot = getattr(config, "_models_root", None)
        fk_urdf = (os.path.join(mroot, "base_hand", "urdf", f"{self.hand_type}.urdf")
                   if mroot else None)
        self.fk = HandSphericalFK(self.hand_type, urdf_path=fk_urdf)

        self.model = pin.buildModelFromUrdf(config._get_urdf_path(self.hand_type))
        self.data = self.model.createData()
        self._lo = np.where(np.isfinite(self.model.lowerPositionLimit),
                            self.model.lowerPositionLimit, -np.pi)
        self._hi = np.where(np.isfinite(self.model.upperPositionLimit),
                            self.model.upperPositionLimit, np.pi)

        self.keypoints = keypoints if keypoints is not None else self._auto_keypoints()
        self._fids = {f: [self.model.getFrameId(n, pin.FrameType.BODY)
                          for n in self.keypoints[f]] for f in FINGERS}

        cols = {}
        for f in FINGERS:
            jid = int(self.model.frames[self._fids[f][-1]].parent)
            cols[f] = [int(i) for i in self.model.supports[jid] if i > 0]
        common = set.intersection(*[set(v) for v in cols.values()])
        self._cols = {f: np.array([self.model.joints[i].idx_v for i in v if i not in common],
                                  dtype=int) for f, v in cols.items()}
        self._allc = sorted({int(i) for c in self._cols.values() for i in c})
        self._gidx = {v: k for k, v in enumerate(self._allc)}

        self.w_tip = float(w_tip)
        self.w_shape = float(config._KP_SHAPE_WEIGHT if w_shape is None else w_shape)
        self.w_pair = float(w_pair)
        self.w_snap = float(w_snap)
        self.w_sep = float(w_sep)
        self._iters = int(iters_per_call)
        self._cold_iters = int(cold_iters)
        self._cold_shape = bool(config._KP_COLD_SHAPE)

        neutral_quats = np.tile(np.array([0.0, 0.0, 0.0, 1.0]),
                                (1 + len(SENSED_JOINTS), 1))
        hp0 = self._human_points(neutral_quats)
        ho, hR = _palm_frame({f: hp0[HUMAN_KEYPOINTS[f][0]] for f in FINGERS})
        h_len = float(np.linalg.norm(hR.T @ (hp0["middle_tip"] - ho)))

        self._fk_robot(pin.neutral(self.model))
        self._r_origin, self._r_frame = _palm_frame(
            {f: self._pos(self._fids[f][0]) for f in FINGERS})
        r_len = float(np.linalg.norm(
            self._r_frame.T @ (self._pos(self._fids["middle"][-1]) - self._r_origin)))
        self.scale = r_len / h_len

        tgt0 = self._targets(hp0)
        self._off, self._seg_len = {}, {}
        for f in FINGERS:
            rc = [self._pos(i) for i in self._fids[f]]
            self._off[f] = [_rot_between(tgt0[f][k + 1] - tgt0[f][k], rc[k + 1] - rc[k])
                            for k in range(len(rc) - 1)]
            self._seg_len[f] = [float(np.linalg.norm(rc[k + 1] - rc[k]))
                                for k in range(len(rc) - 1)]

        fixed = set(config.get_fixed_joint_names(self.hand_type))
        self._out = [(self.model.names[j], self.model.joints[j].idx_q)
                     for j in range(1, self.model.njoints)
                     if self.model.names[j] not in fixed]
        self.joint_names = [n for n, _ in self._out]

        self._q = pin.neutral(self.model)
        self._cold = True

    def _auto_keypoints(self) -> Dict[str, List[str]]:
        m = self.model
        kids: Dict[int, List[int]] = {}
        for j in range(1, m.njoints):
            kids.setdefault(int(m.parents[j]), []).append(j)
        palm = max(kids, key=lambda k: len(kids[k]))
        self._fk_robot(pin.neutral(m))

        def chain(j):
            out = [j]
            while j in kids and len(kids[j]) == 1:
                j = kids[j][0]
                out.append(j)
            return out

        chains = {c: chain(c) for c in kids[palm]}
        if len(chains) != len(FINGERS):
            raise ValueError(
                f"팜 조인트에서 손가락 체인 {len(chains)}개를 찾았다(5개 필요) — "
                "keypoints 를 명시하라")
        base = {c: self.data.oMi[c].translation.copy() for c in chains}
        med = np.median(np.array(list(base.values())), axis=0)
        thumb = max(chains, key=lambda c: np.linalg.norm(base[c] - med))
        rest = [c for c in chains if c != thumb]
        lat = int(np.argmax([np.ptp([base[c][i] for c in rest]) for i in range(3)]))
        sgn = np.sign(base[thumb][lat] - np.mean([base[c][lat] for c in rest])) or 1.0
        rest.sort(key=lambda c: -sgn * base[c][lat])

        out: Dict[str, List[str]] = {}
        for f, c in zip(FINGERS, [thumb] + rest):
            joints = chains[c]
            links = []
            for j in joints:
                bodies = [fr.name for fr in m.frames if fr.type == pin.FrameType.BODY
                          and int(fr.parent) == j and "_sensor_" not in fr.name]
                if bodies:
                    links.append(bodies[0])
            last = joints[-1]
            cands = [fr.name for fr in m.frames if fr.type == pin.FrameType.BODY
                     and int(fr.parent) == last and "_sensor_" not in fr.name]
            tip = max(cands, key=lambda n: np.linalg.norm(
                self._pos(m.getFrameId(n, pin.FrameType.BODY))
                - self.data.oMi[last].translation))
            mid = [n for n in links if n != tip][-3:]
            out[f] = mid + [tip]
        return out

    def _fk_robot(self, q: np.ndarray) -> None:
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)

    def _pos(self, fid: int) -> np.ndarray:
        return self.data.oMf[fid].translation.copy()

    def _human_points(self, sensor_quats: np.ndarray) -> Dict[str, np.ndarray]:
        pts = self.fk.compute_positions(sensor_quats)
        return {n: pts[JOINT_INDEX[n]] for f in FINGERS for n in HUMAN_KEYPOINTS[f]}

    def _targets(self, hp: Dict[str, np.ndarray]) -> Dict[str, np.ndarray]:
        o, R = _palm_frame({f: hp[HUMAN_KEYPOINTS[f][0]] for f in FINGERS})
        return {f: np.array([self._r_frame @ (self.scale * (R.T @ (hp[n] - o)))
                             + self._r_origin for n in HUMAN_KEYPOINTS[f]])
                for f in FINGERS}

    def _pair_rows(self, T: Dict[str, np.ndarray], snap: bool):
        pair_tgt, engaged = {}, []
        for a, b in PAIRS:
            v = T[a][-1] - T[b][-1]
            dist = float(np.linalg.norm(v))
            u = v / max(dist, 1e-9)
            if snap and dist < SNAP_ENGAGE:
                al = float(np.clip((SNAP_ENGAGE - dist) / (SNAP_ENGAGE - SNAP_FULL),
                                   0.0, 1.0))
                mag = (1 - al) * dist + al * SNAP_CONTACT
                pair_tgt[(a, b)] = (mag * u, self.w_pair + al * (self.w_snap - self.w_pair))
                if al >= 1.0:
                    engaged.append(b)
            else:
                pair_tgt[(a, b)] = (v, self.w_pair)
        sep = []
        for a, b in SEP_PAIRS:
            if a in engaged and b in engaged:
                v = T[a][-1] - T[b][-1]
                dist = float(np.linalg.norm(v))
                if dist < SEP_MIN:
                    sep.append((a, b, SEP_MIN * v / max(dist, 1e-9)))
        return pair_tgt, sep

    def _solve(self, T: Dict[str, np.ndarray], q: np.ndarray, iters: int,
               w_tip: float, w_shape: float, w_pair: float, snap: bool) -> np.ndarray:
        pair_tgt, sep = self._pair_rows(T, snap) if w_pair > 0 else ({}, [])
        n = len(self._allc)
        for _ in range(iters):
            self._fk_robot(q)
            J, P = {}, {}
            for f in FINGERS:
                for k, fid in enumerate(self._fids[f]):
                    Jf = pin.computeFrameJacobian(self.model, self.data, q, fid,
                                                  pin.LOCAL_WORLD_ALIGNED)[:3]
                    Jg = np.zeros((3, n))
                    for ci in self._cols[f]:
                        Jg[:, self._gidx[int(ci)]] = Jf[:, ci]
                    J[(f, k)] = Jg
                    P[(f, k)] = self._pos(fid)
            rows, res = [], []
            for f in FINGERS:
                last = len(self._fids[f]) - 1
                if w_shape > 0:
                    for k in range(last):
                        seg = self._off[f][k] @ (T[f][k + 1] - T[f][k])
                        seg *= self._seg_len[f][k] / max(np.linalg.norm(seg), 1e-9)
                        r = seg - (P[(f, k + 1)] - P[(f, k)])
                        w = _huber_w(w_shape, float(np.linalg.norm(r)))
                        rows.append(w * (J[(f, k + 1)] - J[(f, k)]))
                        res.append(w * r)
                r = T[f][-1] - P[(f, last)]
                w = _huber_w(w_tip, float(np.linalg.norm(r)))
                rows.append(w * J[(f, last)])
                res.append(w * r)
            for (a, b), (vt, wp) in pair_tgt.items():
                la, lb = len(self._fids[a]) - 1, len(self._fids[b]) - 1
                r = vt - (P[(a, la)] - P[(b, lb)])
                w = _huber_w(wp, float(np.linalg.norm(r)))
                rows.append(w * (J[(a, la)] - J[(b, lb)]))
                res.append(w * r)
            for a, b, vt in sep:
                la, lb = len(self._fids[a]) - 1, len(self._fids[b]) - 1
                r = vt - (P[(a, la)] - P[(b, lb)])
                rows.append(self.w_sep * (J[(a, la)] - J[(b, lb)]))
                res.append(self.w_sep * r)
            A = np.vstack(rows)
            b = np.concatenate(res)
            lam2 = 0.02 * float(b @ b) + 1e-4
            dq_g = np.linalg.solve(A.T @ A + lam2 * np.eye(n), A.T @ b)
            dq = np.zeros(self.model.nv)
            for ci, k in self._gidx.items():
                dq[ci] = dq_g[k]
            step = float(np.linalg.norm(dq))
            if step > DQ_MAX_STEP:
                dq *= DQ_MAX_STEP / step
            q = np.clip(q + dq, self._lo, self._hi)
        return q

    def compute(self, sensor_quats: np.ndarray) -> np.ndarray:
        hp = self._human_points(np.asarray(sensor_quats, dtype=float))
        T = self._targets(hp)
        if self._cold:
            if self._cold_shape:
                self._q = self._solve(T, self._q, self._cold_iters,
                                      w_tip=0.5, w_shape=1.0, w_pair=0.0, snap=False)
            self._cold = False
        self._q = self._solve(T, self._q, self._iters, w_tip=self.w_tip,
                              w_shape=self.w_shape, w_pair=self.w_pair, snap=True)
        return np.array([self._q[iq] for _, iq in self._out])

    def reset(self) -> None:
        self._q = pin.neutral(self.model)
        self._cold = True
