#!/usr/bin/env python3
"""DLS 가 오차 0 으로 수렴하는 (센서회전, 관절각) 쌍만 모아 학습 데이터를 만든다."""
import argparse
import os
import time
from concurrent.futures import ProcessPoolExecutor

import numpy as np

import hand_rot_ik as hri

_G = {}


def _init(side, steps, eps):
    _G["ik"] = hri.RotIK(side, steps=steps)
    _G["eps"] = eps


def _block(args):
    n, seed = args
    ik, eps = _G["ik"], _G["eps"]
    rng = np.random.default_rng(seed)
    th = hri.sample_poses(ik, n, rng)
    R = np.empty((n, 5, 3, 3), dtype=np.float32)
    G = np.empty((n, ik.nq), dtype=np.float32)
    keep = np.zeros(n, dtype=bool)
    for i in range(n):
        tgt = ik.rel_rot(th[i])
        hat, rr = ik.solve(tgt)
        if np.degrees(rr) < eps:
            R[i] = tgt
            G[i] = hat
            keep[i] = True
    return R[keep], G[keep], th[keep], int(keep.sum()), n


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--side", default="left")
    ap.add_argument("--n", type=int, default=200000)
    ap.add_argument("--block", type=int, default=2000)
    ap.add_argument("--workers", type=int, default=14)
    ap.add_argument("--steps", type=int, default=40)
    ap.add_argument("--eps-deg", type=float, default=0.05)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    nb = (a.n + a.block - 1) // a.block
    tasks = [(a.block, a.seed * 100000 + i) for i in range(nb)]
    t0 = time.time()
    Rs, Gs, Ts, kept, tot = [], [], [], 0, 0
    with ProcessPoolExecutor(max_workers=a.workers, initializer=_init,
                             initargs=(a.side, a.steps, a.eps_deg)) as ex:
        for j, (R, G, T, k, n) in enumerate(ex.map(_block, tasks)):
            Rs.append(R); Gs.append(G); Ts.append(T)
            kept += k; tot += n
            if (j + 1) % 10 == 0:
                print("  %d/%d 블록  채택 %d/%d (%.1f%%)  %.0fs"
                      % (j + 1, nb, kept, tot, 100 * kept / tot,
                         time.time() - t0), flush=True)
    R = np.concatenate(Rs); G = np.concatenate(Gs); T = np.concatenate(Ts)
    ik = hri.RotIK(a.side)
    np.savez_compressed(a.out, R=R, q_gt=G.astype(np.float32),
                        q_true=T.astype(np.float32),
                        names=np.array(ik.names),
                        lo=ik.lo.astype(np.float32),
                        hi=ik.hi.astype(np.float32))
    d = np.degrees(np.abs(G - T))
    print("채택 %d/%d (%.1f%%)  %.0fs  -> %s" % (
        kept, tot, 100 * kept / tot, time.time() - t0, a.out))
    print("GT vs 원본 q 차이: mean %.2f° p95 %.2f°  (널스페이스 몫)"
          % (d.mean(), np.percentile(d, 95)))


if __name__ == "__main__":
    main()
