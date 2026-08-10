from __future__ import annotations

import os
from typing import Dict, List, Optional

import numpy as np
import pinocchio as pin


from .hand_configs import CONFIG_REGISTRY
from .human_fk import FINGERS, HumanHandFK
from .human_fk import palm_frame_from_fingers as _palm_frame
from .human_fk import rot_between as _rot_between

PAIRS = tuple(("thumb", f) for f in ("index", "middle", "ring", "pinky"))
HUBER_DELTA = 0.02
DQ_MAX_STEP = 0.4
LIMIT_MARGIN = 0.10


def _huber_w(w: float, r_norm: float) -> float:
    return w if r_norm <= HUBER_DELTA else w * float(np.sqrt(HUBER_DELTA / r_norm))


class KPHandRetargeter:

    def __init__(self, hand_type: str, config_name: str = "base_hand",
                 urdf_root=None, keypoints: Optional[Dict[str, List[str]]] = None,
                 w_tip: float = 6.0, w_shape: Optional[float] = None,
                 thumb_offset: Optional[float] = None, k_limit: float = 0.3,
                 k_smooth: float = 0.25,
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
        self.fk = HumanHandFK(self.hand_type, urdf_path=fk_urdf)
        self.human_joint_names = self.fk.joint_names

        self.urdf_path = config._get_urdf_path(self.hand_type)
        self.model = pin.buildModelFromUrdf(self.urdf_path)
        self.data = self.model.createData()
        self._lo = np.where(np.isfinite(self.model.lowerPositionLimit),
                            self.model.lowerPositionLimit, -np.pi)
        self._hi = np.where(np.isfinite(self.model.upperPositionLimit),
                            self.model.upperPositionLimit, np.pi)

        self.keypoints = (keypoints if keypoints is not None
                          else self._sensor_keypoints() or self._auto_keypoints())
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
        self._vidx = {self.model.joints[j].idx_v: self.model.joints[j].idx_q
                      for j in range(1, self.model.njoints)}

        self.w_tip = float(w_tip)
        self.w_shape = float(config._KP_SHAPE_WEIGHT if w_shape is None else w_shape)
        self.k_limit = float(k_limit)
        self.k_smooth = float(k_smooth)
        self._iters = int(iters_per_call)
        self._cold_iters = int(cold_iters)
        self._cold_shape = bool(config._KP_COLD_SHAPE)

        self._palm_fid = self._pick_palm_frame()
        hp0 = self._human_points(self._neutral_human())
        self._h_origin, self._h_frame = _palm_frame(hp0)
        h_len = float(np.linalg.norm(
            self._h_frame.T @ (hp0["middle"][-1] - self._h_origin)))

        self._fk_robot(pin.neutral(self.model))
        self._r_origin, self._r_frame = _palm_frame(
            {f: np.array([self._pos(i) for i in self._fids[f]]) for f in FINGERS})
        r_len = float(np.linalg.norm(
            self._r_frame.T @ (self._pos(self._fids["middle"][-1]) - self._r_origin)))
        self.scale = r_len / h_len
        self.thumb_offset = float(config._KP_THUMB_OFFSET
                                  if thumb_offset is None else thumb_offset)
        self._t_off = np.zeros(3)

        tgt0 = self._targets(hp0)
        self._off, self._seg_len = {}, {}
        for f in FINGERS:
            rc = [self._pos(i) for i in self._fids[f]]
            self._off[f] = [_rot_between(tgt0[f][k + 1] - tgt0[f][k], rc[k + 1] - rc[k])
                            for k in range(len(rc) - 1)]
            self._seg_len[f] = [float(np.linalg.norm(rc[k + 1] - rc[k]))
                                for k in range(len(rc) - 1)]
        if self.thumb_offset != 0.0:
            self._t_off = self.thumb_offset * (
                self._pos(self._fids["thumb"][0]) - tgt0["thumb"][0])

        fixed = set(config.get_fixed_joint_names(self.hand_type))
        self._out = [(self.model.names[j], self.model.joints[j].idx_q)
                     for j in range(1, self.model.njoints)
                     if self.model.names[j] not in fixed]
        self.joint_names = [n for n, _ in self._out]

        self._q = pin.neutral(self.model)
        self._cold = True
        self._cold_prev: Optional[np.ndarray] = None
        self._last_targets: Optional[Dict[str, np.ndarray]] = None



    def _limit_push(self, q: np.ndarray) -> Optional[np.ndarray]:
        g = np.zeros(len(self._allc))
        hit = False
        for ci, k in self._gidx.items():
            iq = self._vidx[ci]
            lo, hi = self._lo[iq], self._hi[iq]
            if not np.isfinite(lo) or not np.isfinite(hi) or hi - lo < 1e-9:
                continue
            margin = min(LIMIT_MARGIN, 0.25 * (hi - lo))
            if q[iq] < lo + margin:
                g[k] = (lo + margin) - q[iq]
                hit = True
            elif q[iq] > hi - margin:
                g[k] = (hi - margin) - q[iq]
                hit = True
        return g if hit else None

    def _pick_palm_frame(self) -> int:
        m = self.model
        name = self.hand_type + "_sensor_dorsum"
        if m.existFrame(name, pin.FrameType.BODY):
            return m.getFrameId(name, pin.FrameType.BODY)
        common = set.intersection(*[
            {int(j) for j in m.supports[int(m.frames[self._fids[f][-1]].parent)] if j > 0}
            for f in FINGERS])
        palm_joint = max(common) if common else 0
        for fr in m.frames:
            if (fr.type == pin.FrameType.BODY and int(fr.parent) == palm_joint
                    and "_sensor_" not in fr.name):
                return m.getFrameId(fr.name, pin.FrameType.BODY)
        raise ValueError(
            f"{self.urdf_path}: 팜 기준 프레임을 못 찾았다"
            f" ({name} 도 없고 조인트 {palm_joint} 에 링크도 없다)")

    def _sensor_keypoints(self) -> Optional[Dict[str, List[str]]]:
        m = self.model
        pfx = self.hand_type + "_"
        tips = {f: pfx + f"sensor_{f}_distal" for f in FINGERS}
        if not all(m.existFrame(n, pin.FrameType.BODY) for n in tips.values()):
            return None
        out: Dict[str, List[str]] = {}
        for f, tip in tips.items():
            jid = int(m.frames[m.getFrameId(tip, pin.FrameType.BODY)].parent)
            chain = [int(j) for j in m.supports[jid] if j > 0]
            links = []
            for j in chain:
                bodies = [fr.name for fr in m.frames
                          if fr.type == pin.FrameType.BODY and int(fr.parent) == j
                          and "_sensor_" not in fr.name]
                if bodies:
                    links.append(bodies[0])
            if len(links) < 3:
                return None
            out[f] = links[-3:] + [tip]
        return out

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

    def _neutral_human(self):
        return {n: 0.0 for n in self.human_joint_names}

    def _human_points(self, source) -> Dict[str, np.ndarray]:
        return self.fk.points(source)

    def _targets(self, hp: Dict[str, np.ndarray]) -> Dict[str, np.ndarray]:
        o, R = self._h_origin, self._h_frame
        out = {f: np.array([self._r_frame @ (self.scale * (R.T @ (p - o)))
                            + self._r_origin for p in hp[f]])
               for f in FINGERS}
        out["thumb"] = out["thumb"] + self._t_off
        return out



    def _solve(self, T: Dict[str, np.ndarray], q: np.ndarray, iters: int,
               w_tip: float, w_shape: float,
               q_prev: Optional[np.ndarray] = None) -> np.ndarray:
        n = len(self._allc)
        prev_g = None
        if q_prev is not None and self.k_smooth > 0.0:
            prev_g = np.array([q_prev[self._vidx[ci]] for ci in self._allc])
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
                if w_tip > 0:
                    r = T[f][-1] - P[(f, last)]
                    w = _huber_w(w_tip, float(np.linalg.norm(r)))
                    rows.append(w * J[(f, last)])
                    res.append(w * r)
            if prev_g is not None:
                cur_g = np.array([q[self._vidx[ci]] for ci in self._allc])
                rows.append(self.k_smooth * np.eye(n))
                res.append(self.k_smooth * (prev_g - cur_g))
            if self.k_limit > 0.0:
                g = self._limit_push(q)
                if g is not None:
                    rows.append(self.k_limit * np.eye(n))
                    res.append(self.k_limit * g)
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

    def human_to_robot(self) -> np.ndarray:
        o, R = self._h_origin, self._h_frame
        A = self._r_frame @ R.T
        T = np.eye(4)
        T[:3, :3] = A
        T[:3, 3] = self._r_origin - A @ o
        return T

    def current_q(self) -> np.ndarray:
        return np.array([self._q[iq] for _, iq in self._out])

    def last_targets(self) -> Optional[Dict[str, np.ndarray]]:
        return self._last_targets

    def achieved_points(self) -> Dict[str, np.ndarray]:
        self._fk_robot(self._q)
        return {f: np.array([self._pos(i) for i in ids])
                for f, ids in self._fids.items()}

    def contact_pairs(self) -> Dict[str, float]:
        self._fk_robot(self._q)
        out = {}
        for a, b in PAIRS:
            out[f"{a}-{b}"] = float(np.linalg.norm(
                self._pos(self._fids[a][-1]) - self._pos(self._fids[b][-1])))
        return out

    def tip_error(self) -> float:
        if self._last_targets is None:
            return float("nan")
        self._fk_robot(self._q)
        return float(np.mean([
            np.linalg.norm(self._pos(self._fids[f][-1]) - self._last_targets[f][-1])
            for f in FINGERS]))

    def target_contact_pairs(self) -> Dict[str, float]:
        if self._last_targets is None:
            return {}
        T = self._last_targets
        return {f"{a}-{b}": float(np.linalg.norm(T[a][-1] - T[b][-1]))
                for a, b in PAIRS}

    def compute(self, human_input) -> np.ndarray:
        T = self._targets(self._human_points(human_input))
        self._last_targets = T
        if self._cold:
            if self._cold_shape:
                self._q = self._solve(T, self._q, self._cold_iters,
                                      w_tip=0.5, w_shape=1.0)
            self._cold = False
        q_prev = None if self._cold_prev is None else self._cold_prev.copy()
        self._q = self._solve(T, self._q, self._iters, w_tip=self.w_tip,
                              w_shape=self.w_shape, q_prev=q_prev)
        self._cold_prev = self._q.copy()
        return np.array([self._q[iq] for _, iq in self._out])

    def reset(self) -> None:
        self._q = pin.neutral(self.model)
        self._cold = True
        self._cold_prev = None
        self._last_targets = None
