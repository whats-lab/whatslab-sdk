#!/usr/bin/env python3
import argparse
import glob
import os

import numpy as np

OSIM_RANGE = {
    "cmc_flexion": (-45, 15), "cmc_abduction": (-25, 25),
    "mp_flexion": (-40, 45), "ip_flexion": (-25, 75),
    "4cmc_flexion": (0, 11.6),
    "2mcp_abduction": (-15, 15), "2mcp_flexion": (-45, 90),
    "2pm_flexion": (0, 100), "2md_flexion": (0, 80),
    "3mcp_abduction": (-15, 15), "3mcp_flexion": (-45, 90),
    "3pm_flexion": (0, 100), "3md_flexion": (0, 80),
    "4mcp_abduction": (-15, 15), "4mcp_flexion": (-45, 90),
    "4pm_flexion": (0, 100), "4md_flexion": (0, 80),
    "5mcp_abduction": (-15, 15), "5mcp_flexion": (-45, 90),
    "5pm_flexion": (0, 100), "5md_flexion": (0, 80),
}
ANAT_MARGIN = 15.0
VEL_MAX = 900.0


def load(d):
    out = {}
    for p in sorted(glob.glob(os.path.join(d, "*.npz"))):
        z = np.load(p, allow_pickle=True)
        tag = os.path.basename(p)[:-4]
        out[tag] = (z["t"], z["q"], [str(x) for x in z["names"]])
    return out


def frame_flags(t, q, names):
    n, m = q.shape
    bad_range = np.zeros((n, m), bool)
    for j, nm in enumerate(names):
        r = OSIM_RANGE.get(nm)
        if r is None:
            continue
        bad_range[:, j] = (q[:, j] < r[0] - ANAT_MARGIN) | \
                          (q[:, j] > r[1] + ANAT_MARGIN)
    dt = np.median(np.diff(t)) if len(t) > 2 else 0.01
    v = np.abs(np.diff(q, axis=0, prepend=q[:1])) / max(dt, 1e-6)
    bad_vel = v > VEL_MAX
    bad_nan = ~np.isfinite(q)
    return bad_range, bad_vel, bad_nan


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", required=True)
    ap.add_argument("--out", default=None)
    args = ap.parse_args()
    D = load(args.dir)
    if not D:
        raise SystemExit("데이터 없음: " + args.dir)
    names = D[list(D)[0]][2]
    nsub = len({k.split("_")[0] for k in D})
    print("파일 %d개, 피험자 %d명, 관절 %d개" % (len(D), nsub, len(names)))

    tot = np.zeros(len(names))
    bad = np.zeros(len(names))
    lo_all = {n: [] for n in names}
    hi_all = {n: [] for n in names}
    per_file = {}
    for tag, (t, q, nm) in D.items():
        br, bv, bn = frame_flags(t, q, nm)
        b = br | bv | bn
        tot += len(q)
        bad += b.sum(0)
        keep = ~b.any(1)
        per_file[tag] = (len(q), int(keep.sum()))
        for j, n in enumerate(nm):
            good = q[~b[:, j], j]
            if len(good):
                lo_all[n].append(np.percentile(good, 0.5))
                hi_all[n].append(np.percentile(good, 99.5))

    print("\n%-16s %13s %8s   %s" % ("관절", ".osim range", "이탈%",
                                    "정제후 개인 0.5/99.5% (mean±sd)"))
    rows = []
    for j, n in enumerate(names):
        r = OSIM_RANGE.get(n)
        lo, hi = np.array(lo_all[n]), np.array(hi_all[n])
        rows.append((n, r, lo, hi, 100.0 * bad[j] / max(tot[j], 1)))
        print("%-16s %13s %7.1f%%   [%+6.1f±%4.1f, %+6.1f±%4.1f]  n=%d"
              % (n, ("[%+.0f,%+.0f]" % r) if r else "-",
                 100.0 * bad[j] / max(tot[j], 1),
                 lo.mean() if len(lo) else np.nan, lo.std() if len(lo) else 0,
                 hi.mean() if len(hi) else np.nan, hi.std() if len(hi) else 0,
                 len(lo)))

    ok = np.array([v[1] for v in per_file.values()])
    n_all = np.array([v[0] for v in per_file.values()])
    print("\n프레임 전체 %d, 전관절 동시 유효 %d (%.1f%%)"
          % (n_all.sum(), ok.sum(), 100.0 * ok.sum() / n_all.sum()))
    worst = sorted(per_file.items(), key=lambda kv: kv[1][1] / max(kv[1][0], 1))
    print("\n유효율 최저 파일 8개:")
    for k, (a, b) in worst[:8]:
        print("   %-34s %5d/%5d = %5.1f%%" % (k, b, a, 100.0 * b / max(a, 1)))
    if args.out:
        np.savez(args.out, names=np.array(names),
                 lo=np.array([np.array(lo_all[n]) if len(lo_all[n]) else np.array([np.nan])
                              for n in names], dtype=object),
                 hi=np.array([np.array(hi_all[n]) if len(hi_all[n]) else np.array([np.nan])
                              for n in names], dtype=object),
                 badfrac=bad / np.maximum(tot, 1), allow_pickle=True)
        print("\n저장:", args.out)


if __name__ == "__main__":
    main()
