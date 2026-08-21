#!/usr/bin/env python3
import argparse
import os
import time
from concurrent.futures import ProcessPoolExecutor

import numpy as np
import pinocchio as pin

from whatslab.solvers.hand.human_fk import BONE_LINKS, FINGERS, HumanHandFK

KEYPOINT_ORDER = ["wrist"] + [f"{f}_{j}" for f in FINGERS
                              for j in ("a", "b", "c", "tip")]
N_KP = len(KEYPOINT_ORDER)
REF_KP = 1 + 4 * FINGERS.index("middle") + 3
SEGMENTS = [(0 if k == 0 else 1 + 4 * i + k - 1, 1 + 4 * i + k)
            for i in range(len(FINGERS)) for k in range(4)]


class Retargeter:

    def __init__(self, side, w_bone=1.0, iters=30, tol=1e-9, free=False,
                 urdf_path=None, fix_scale=False, scale_const=None):
        self.fk = HumanHandFK(side, urdf_path=urdf_path)
        m = self.fk.model
        self.model, self.data = m, m.createData()
        self.names = list(self.fk.joint_names)
        self.iv = np.array([m.joints[m.getJointId(n)].idx_v for n in self.names])
        self.iq = np.array([self.fk._idx_q[n] for n in self.names])
        self.lim_lo = m.lowerPositionLimit[self.iq].copy()
        self.lim_hi = m.upperPositionLimit[self.iq].copy()
        self.free = bool(free)
        if self.free:
            span = self.lim_hi - self.lim_lo
            self.lo = self.lim_lo - 1.5 * span
            self.hi = self.lim_hi + 1.5 * span
        else:
            self.lo, self.hi = self.lim_lo, self.lim_hi
        self.fids = [self.fk._palm_fid] + [self.fk._fids[n] for f in FINGERS
                                           for n in BONE_LINKS[f]]
        self.w_bone = float(w_bone)
        self.iters = int(iters)
        self.tol = float(tol)
        self.nq = len(self.names)
        self.fix_scale = bool(fix_scale)
        self.scale_const = None if scale_const is None else float(scale_const)
        mid = FINGERS.index("middle")
        self.ref_seg = [(0 if k == 0 else 1 + 4 * mid + k - 1, 1 + 4 * mid + k)
                        for k in range(4)]
        P0 = self.keypoints_of(0.5 * (self.lo + self.hi))
        self.ref_len = float(sum(np.linalg.norm(P0[b] - P0[a])
                                 for a, b in self.ref_seg))

    def _points_and_jac(self, q):
        pin.computeJointJacobians(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        P = np.empty((N_KP, 3))
        J = np.empty((N_KP, 3, self.nq))
        for k, fid in enumerate(self.fids):
            P[k] = self.data.oMf[fid].translation
            Jf = pin.getFrameJacobian(self.model, self.data, fid,
                                      pin.LOCAL_WORLD_ALIGNED)[:3]
            J[k] = Jf[:, self.iv]
        return P, J

    def _dirs_and_jac(self, P, J):
        a = np.array([s[0] for s in SEGMENTS])
        b = np.array([s[1] for s in SEGMENTS])
        v = P[b] - P[a]
        n = np.linalg.norm(v, axis=1, keepdims=True)
        n = np.maximum(n, 1e-9)
        d = v / n
        Jv = J[b] - J[a]
        Jd = (Jv - d[:, :, None] * np.einsum("sk,skj->sj", d, Jv)[:, None, :]) / n[:, :, None]
        return d, Jd

    def _q_full(self, qn):
        q = pin.neutral(self.model)
        q[self.iq] = qn
        return q

    def solve(self, kp, q0=None, R0=None, t0=None, s0=None):
        x = np.asarray(kp, dtype=float)
        a_i = np.array([a for a, _ in SEGMENTS])
        b_i = np.array([b for _, b in SEGMENTS])
        qn = 0.5 * (self.lo + self.hi) if q0 is None else q0.copy()
        P, J = self._points_and_jac(self._q_full(qn))
        if self.scale_const is not None:
            s = self.scale_const
        elif self.fix_scale:
            lt = float(sum(np.linalg.norm(x[b] - x[a])
                           for a, b in self.ref_seg))
            s = self.ref_len / max(lt, 1e-9)
        elif s0 is None:
            lt = np.linalg.norm(x[REF_KP] - x[0])
            s = np.linalg.norm(P[REF_KP] - P[0]) / max(lt, 1e-9)
        else:
            s = float(s0)
        R = np.eye(3) if R0 is None else R0.copy()
        t = (P.mean(0) - s * (x @ R.T).mean(0)) if t0 is None else t0.copy()
        lam = 1e-6
        NWF = 6 if (self.fix_scale or self.scale_const is not None) else 7

        def ev(qn, R, t, s):
            P, J = self._points_and_jac(self._q_full(qn))
            y = s * (x @ R.T) + t
            d, Jd = self._dirs_and_jac(P, J)
            vt = y[b_i] - y[a_i]
            nt = np.maximum(np.linalg.norm(vt, axis=1, keepdims=True), 1e-9)
            dt = vt / nt
            r = np.concatenate([(P - y).ravel(), self.w_bone * (d - dt).ravel()])
            return r, P, J, d, Jd, y, nt, dt

        r, P, J, d, Jd, y, nt, dt = ev(qn, R, t, s)
        cost = r @ r
        NW = NWF
        for _ in range(self.iters):
            Jq = np.vstack([J.reshape(N_KP * 3, self.nq),
                            self.w_bone * Jd.reshape(len(SEGMENTS) * 3, self.nq)])
            Jw = np.zeros((r.size, NW))
            yc = y - t
            for k in range(N_KP):
                Jw[3 * k:3 * k + 3, :3] = pin.skew(yc[k])
                Jw[3 * k:3 * k + 3, 3:6] = -np.eye(3)
                if NW > 6:
                    Jw[3 * k:3 * k + 3, 6] = -yc[k]
            off = N_KP * 3
            for si in range(len(SEGMENTS)):
                Pj = (np.eye(3) - np.outer(dt[si], dt[si])) / nt[si, 0]
                Jw[off + 3 * si:off + 3 * si + 3, :3] = \
                    self.w_bone * Pj @ pin.skew(yc[b_i[si]] - yc[a_i[si]])
            Jf = np.hstack([Jq, Jw])
            H = Jf.T @ Jf
            A = H + lam * np.diag(np.maximum(np.diag(H), 1e-12))
            try:
                step = np.linalg.solve(A, -(Jf.T @ r))
            except np.linalg.LinAlgError:
                break
            qn2 = np.clip(qn + step[:self.nq], self.lo, self.hi)
            R2 = pin.exp3(step[self.nq:self.nq + 3]) @ R
            t2 = t + step[self.nq + 3:self.nq + 6]
            s2 = s if NW == 6 else s * float(np.exp(step[self.nq + 6]))
            out = ev(qn2, R2, t2, s2)
            if out[0] @ out[0] >= cost:
                lam *= 8.0
                if lam > 1e6:
                    break
                continue
            gain = cost - out[0] @ out[0]
            qn, R, t, s = qn2, R2, t2, s2
            r, P, J, d, Jd, y, nt, dt = out
            cost = r @ r
            lam = max(lam * 0.4, 1e-9)
            if gain < self.tol * max(cost, 1e-12):
                break
        rms = float(np.sqrt(r[:N_KP * 3] @ r[:N_KP * 3] / N_KP))
        return qn, R, t, s, rms

    def keypoints_of(self, qn):
        P, _ = self._points_and_jac(self._q_full(qn))
        return P


def _run_block(args):
    side, w_bone, iters, block, free, urdf_path, scale_const = args
    rt = Retargeter(side, w_bone=w_bone, iters=iters, free=free,
                    urdf_path=urdf_path, scale_const=scale_const)
    q = R = t = sc = None
    out, res = [], []
    for kp in block:
        q, R, t, sc, rms = rt.solve(kp, q, R, t, sc)
        out.append(q.copy()); res.append(rms)
    return np.array(out), np.array(res)


def retarget_all(side, kps, w_bone, iters, workers, block=200, free=False,
                 urdf_path=None, scale_const=None):
    if workers <= 1:
        return _run_block((side, w_bone, iters, kps, free, urdf_path,
                           scale_const))
    blocks = [kps[i:i + block] for i in range(0, len(kps), block)]
    with ProcessPoolExecutor(max_workers=workers) as ex:
        parts = list(ex.map(_run_block,
                            [(side, w_bone, iters, b, free, urdf_path,
                              scale_const) for b in blocks]))
    return (np.concatenate([p[0] for p in parts]),
            np.concatenate([p[1] for p in parts]))


def jitter_skeleton(kp, rng, frac):
    out = kp.copy()
    for i in range(len(FINGERS)):
        cur = kp[0].copy()
        for k in range(4):
            a = 0 if k == 0 else 1 + 4 * i + k - 1
            b = 1 + 4 * i + k
            v = (kp[b] - kp[a]) * (1.0 + rng.uniform(-frac, frac))
            cur = cur + v
            out[b] = cur
    return out


def selftest(side, n, seed, w_bone, iters, skel, check_jac=False):
    rt = Retargeter(side, w_bone=w_bone, iters=iters)
    rng = np.random.default_rng(seed)
    e, dq, ms = [], [], []
    for _ in range(n):
        qt = rt.lo + rng.uniform(0.05, 0.95, rt.nq) * (rt.hi - rt.lo)
        kp = rt.keypoints_of(qt)
        if skel > 0.0:
            kp = jitter_skeleton(kp, rng, skel)
        kp = (kp * rng.uniform(0.75, 1.35)) @ pin.exp3(rng.normal(0, 1.0, 3)).T \
            + rng.normal(0, 0.3, 3)
        t0 = time.perf_counter()
        q, _R, _t, _s, rms = rt.solve(kp)
        ms.append((time.perf_counter() - t0) * 1e3)
        lref = np.linalg.norm(rt.keypoints_of(qt)[REF_KP] - rt.keypoints_of(qt)[0])
        e.append(rms / lref * 100.0)
        dq.append(np.degrees(np.abs(q - qt)).max())
    return np.array(e), np.array(dq), np.array(ms)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--side", default="left", choices=("left", "right"))
    ap.add_argument("--dataset", default=None,
                    help="npz — key 'keypoints' (T,21,3), 순서는 KEYPOINT_ORDER")
    ap.add_argument("--out", default=None)
    ap.add_argument("--w-bone", type=float, default=1.0)
    ap.add_argument("--iters", type=int, default=30)
    ap.add_argument("--workers", type=int, default=max(1, (os.cpu_count() or 2) - 2))
    ap.add_argument("--block", type=int, default=200)
    ap.add_argument("--selftest", type=int, default=0)
    ap.add_argument("--skeleton-jitter", type=float, default=0.0)
    ap.add_argument("--free", action="store_true",
                    help="리타게팅 중 관절한계를 풀어 원래 자세를 그대로 받는다")
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()

    if args.selftest:
        e, d, ms = selftest(args.side, args.selftest, args.seed, args.w_bone,
                            args.iters, args.skeleton_jitter)
        print("왕복 검증 %d프레임 (임의 회전·평행이동·스케일 0.75~1.35, 골격지터 ±%.0f%%)"
              % (args.selftest, args.skeleton_jitter * 100))
        print("  키포인트 RMS   mean %.3f%% p95 %.3f%%" % (e.mean(), np.percentile(e, 95)))
        print("  관절 최대오차  mean %.2f도 p95 %.2f도" % (d.mean(), np.percentile(d, 95)))
        print("  프레임당       %.2f ms" % ms.mean())
        return

    if not args.dataset or not args.out:
        raise SystemExit("--dataset 과 --out (또는 --selftest N)")
    kps = np.load(args.dataset)["keypoints"].astype(float)
    if kps.shape[1:] != (N_KP, 3):
        raise SystemExit("keypoints 는 (T,%d,3): %s" % (N_KP, kps.shape))
    t0 = time.perf_counter()
    q, res = retarget_all(args.side, kps, args.w_bone, args.iters,
                          args.workers, args.block, args.free)
    dt = time.perf_counter() - t0
    np.savez(args.out, q=q, joint_names=np.array(Retargeter(args.side).names),
             residual=res)
    print("%s: %d프레임 %.1fs (%.2f ms/frame, worker %d) 잔차 mean %.2fmm p95 %.2fmm"
          % (args.out, len(q), dt, dt / len(q) * 1e3, args.workers,
             res.mean() * 1e3, np.percentile(res, 95) * 1e3))


if __name__ == "__main__":
    main()
