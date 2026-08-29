#!/usr/bin/env python3
"""손목 기준 손끝 센서 회전만으로 손 관절각을 푸는 DLS IK (소프트 한계만 포함)."""
import argparse
import os
import time

import numpy as np
import pinocchio as pin

FING = ("thumb", "index", "middle", "ring", "pinky")
LIMIT_MARGIN_FRAC = 0.05
W_PENALTY = 1.0
LM_LAMBDA0 = 1e-4


class RotIK:

    def __init__(self, side="left", urdf_path=None, steps=40, tol=1e-10):
        if urdf_path is None:
            urdf_path = os.path.join(os.environ["WHATSLAB_MODELS_ROOT"],
                                     "base_hand", "urdf", "%s.urdf" % side)
        self.model = pin.buildModelFromUrdf(urdf_path)
        self.data = self.model.createData()
        m = self.model
        self.names = [m.names[j] for j in range(1, m.njoints)
                      if m.joints[j].nq > 0]
        self.iq = np.array([m.joints[m.getJointId(n)].idx_q
                            for n in self.names])
        self.iv = np.array([m.joints[m.getJointId(n)].idx_v
                            for n in self.names])
        self.lo = m.lowerPositionLimit[self.iq].copy()
        self.hi = m.upperPositionLimit[self.iq].copy()
        self.dorsum = m.getFrameId("%s_sensor_dorsum" % side)
        self.tips = [m.getFrameId("%s_sensor_%s_distal" % (side, f))
                     for f in FING]
        self.nq = len(self.names)
        self.steps = int(steps)
        self.tol = float(tol)
        self.margin = np.maximum(LIMIT_MARGIN_FRAC * (self.hi - self.lo), 1e-9)

    def q_full(self, th):
        q = pin.neutral(self.model)
        q[self.iq] = th
        return q

    def neutral_theta(self):
        return 0.5 * (self.lo + self.hi)

    def rel_rot(self, th):
        pin.forwardKinematics(self.model, self.data, self.q_full(th))
        pin.updateFramePlacements(self.model, self.data)
        Rd = self.data.oMf[self.dorsum].rotation
        return np.array([Rd.T @ self.data.oMf[t].rotation for t in self.tips])

    def _limit_res(self, th):
        rho = np.zeros(self.nq)
        drho = np.zeros(self.nq)
        for d, sign in ((th - (self.hi - self.margin),  1.0),
                        ((self.lo + self.margin) - th, -1.0)):
            ramp = (d > 0) & (d < self.margin)
            past = d >= self.margin
            rho[ramp] += d[ramp] ** 2 / (2 * self.margin[ramp])
            drho[ramp] += sign * d[ramp] / self.margin[ramp]
            rho[past] += d[past] - 0.5 * self.margin[past]
            drho[past] += sign
        w = np.sqrt(W_PENALTY)
        return w * rho, w * np.diag(drho)

    def _rot_res(self, th, tgt, need_jac=True):
        q = self.q_full(th)
        pin.computeJointJacobians(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        Rd = self.data.oMf[self.dorsum].rotation
        Jd = pin.getFrameJacobian(self.model, self.data, self.dorsum,
                                  pin.LOCAL)[3:6][:, self.iv]
        r = np.empty(15)
        J = np.empty((15, self.nq)) if need_jac else None
        for i, t in enumerate(self.tips):
            A = Rd.T @ self.data.oMf[t].rotation
            E = tgt[i].T @ A
            r[3 * i:3 * i + 3] = pin.log3(E)
            if not need_jac:
                continue
            Jt = pin.getFrameJacobian(self.model, self.data, t,
                                      pin.LOCAL)[3:6][:, self.iv]
            J[3 * i:3 * i + 3] = np.asarray(pin.Jlog3(E)) @ (Jt - A.T @ Jd)
        return r, J

    def solve(self, tgt, th0=None):
        th = self.neutral_theta() if th0 is None else np.clip(
            th0.copy(), self.lo, self.hi)
        rr, Jr = self._rot_res(th, tgt)
        rl, Jl = self._limit_res(th)
        r = np.concatenate([rr, rl])
        cost = r @ r
        lam = LM_LAMBDA0
        for _ in range(self.steps):
            J = np.vstack([Jr, Jl])
            H = J.T @ J
            A = H + lam * np.diag(np.maximum(np.diag(H), 1e-8))
            try:
                step = np.linalg.solve(A, -(J.T @ r))
            except np.linalg.LinAlgError:
                break
            th2 = th + step
            rr2, Jr2 = self._rot_res(th2, tgt)
            rl2, Jl2 = self._limit_res(th2)
            r2 = np.concatenate([rr2, rl2])
            c2 = r2 @ r2
            if c2 >= cost:
                lam *= 8.0
                if lam > 1e8:
                    break
                continue
            gain = cost - c2
            th, r, rr, Jr, rl, Jl, cost = th2, r2, rr2, Jr2, rl2, Jl2, c2
            lam = max(lam * 0.3, 1e-10)
            if gain < self.tol * max(cost, 1e-12):
                break
        return th, float(np.linalg.norm(rr) / np.sqrt(5))


def sample_poses(ik, n, rng, mode="mix"):
    lo, hi = ik.lo, ik.hi
    out = np.empty((n, ik.nq))
    k = 0
    while k < n:
        m = rng.integers(0, 3) if mode == "mix" else 0
        if m == 0:
            th = lo + rng.uniform(0.05, 0.95, ik.nq) * (hi - lo)
        elif m == 1:
            f = rng.uniform(0.0, 1.0)
            th = lo + (f + rng.normal(0, 0.08, ik.nq)) * (hi - lo)
        else:
            th = 0.5 * (lo + hi) + rng.normal(0, 0.15, ik.nq) * (hi - lo)
        out[k] = np.clip(th, lo, hi)
        k += 1
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--side", default="left")
    ap.add_argument("--n", type=int, default=200)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--eps-deg", type=float, default=0.05)
    a = ap.parse_args()
    ik = RotIK(a.side)
    rng = np.random.default_rng(a.seed)
    th_true = sample_poses(ik, a.n, rng)
    res, dq, ms = [], [], []
    for th in th_true:
        tgt = ik.rel_rot(th)
        t0 = time.perf_counter()
        th_hat, rr = ik.solve(tgt)
        ms.append((time.perf_counter() - t0) * 1e3)
        res.append(np.degrees(rr))
        dq.append(np.degrees(np.abs(th_hat - th)))
    res = np.array(res); dq = np.array(dq); ms = np.array(ms)
    ok = res < a.eps_deg
    print("샘플 %d  회전잔차 mean %.4f° p95 %.4f°  수렴(<%.2f°) %.1f%%  %.1f ms/f"
          % (a.n, res.mean(), np.percentile(res, 95), a.eps_deg,
             100 * ok.mean(), ms.mean()))
    print("\n%-24s %10s %10s" % ("관절", "복원오차[°]", "가동폭[°]"))
    span = np.degrees(ik.hi - ik.lo)
    d = dq[ok] if ok.any() else dq
    for i in np.argsort(-d.mean(0)):
        print("%-24s %10.2f %10.1f" % (ik.names[i], d[:, i].mean(), span[i]))


if __name__ == "__main__":
    main()
