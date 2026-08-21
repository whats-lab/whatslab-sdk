#!/usr/bin/env python3
import argparse
import glob
import importlib.util
import os

import numpy as np
import pinocchio as pin

from whatslab.solvers.hand.human_fk import HumanHandFK

bc = importlib.util.module_from_spec(importlib.util.spec_from_file_location(
    "bc", os.path.join(os.path.dirname(os.path.abspath(__file__)), "bham_clean.py")))
bc.__loader__.exec_module(bc)

MAP = {"index_mcp_flex": "2mcp_flexion", "index_mcp_abd": "2mcp_abduction",
       "index_pip": "2pm_flexion", "index_dip": "2md_flexion",
       "middle_mcp_flex": "3mcp_flexion", "middle_mcp_abd": "3mcp_abduction",
       "middle_pip": "3pm_flexion", "middle_dip": "3md_flexion",
       "ring_mcp_flex": "4mcp_flexion", "ring_mcp_abd": "4mcp_abduction",
       "ring_pip": "4pm_flexion", "ring_dip": "4md_flexion",
       "pinky_mcp_flex": "5mcp_flexion", "pinky_mcp_abd": "5mcp_abduction",
       "pinky_pip": "5pm_flexion", "pinky_dip": "5md_flexion",
       "thumb_cmc_flex": "cmc_flexion", "thumb_cmc_abd": "cmc_abduction",
       "thumb_mcp_flex": "mp_flexion", "thumb_ip_flex": "ip_flexion"}
BHAM_AXIS = {
    "2mcp_flexion": (0.980, 0.026, -0.198), "2mcp_abduction": (0.215, 0.192, 0.958),
    "2pm_flexion": (0.976, 0.127, -0.175), "2md_flexion": (0.986, 0.147, 0.075),
    "3mcp_flexion": (0.999, 0.047, -0.010), "3mcp_abduction": (-0.021, 0.119, 0.993),
    "3pm_flexion": (0.993, -0.062, 0.102), "3md_flexion": (0.995, 0.036, 0.097),
    "4mcp_flexion": (0.997, -0.065, -0.030), "4mcp_abduction": (-0.022, 0.144, 0.989),
    "4pm_flexion": (0.995, -0.035, 0.092), "4md_flexion": (0.953, -0.140, 0.270),
    "5mcp_flexion": (0.976, -0.214, -0.035), "5mcp_abduction": (-0.025, 0.216, 0.976),
    "5pm_flexion": (0.928, -0.328, 0.176), "5md_flexion": (0.893, -0.427, 0.145)}
GRADE = {"thumb": "A", "index": "A", "ring": "B", "pinky": "B", "middle": "C"}
KEEP = ["B030", "B027", "B029", "B013", "B019", "B006", "B028", "B024",
        "B009", "B014", "B010", "B005", "B020", "B004"]


def envelope(dirs):
    acc = {}
    nf = 0
    for d in dirs:
        for p in sorted(glob.glob(os.path.join(d, "*.npz"))):
            if os.path.basename(p).split("_")[0] not in KEEP:
                continue
            z = np.load(p, allow_pickle=True)
            t, q, nm = z["t"], z["q"], [str(x) for x in z["names"]]
            br, bv, bn = bc.frame_flags(t, q, nm)
            k = ~(br | bv | bn).any(1)
            if k.sum() < 50:
                continue
            nf += int(k.sum())
            sub = os.path.basename(p).split("_")[0]
            for j, n in enumerate(nm):
                acc.setdefault(n, {}).setdefault(sub, []).append(q[k, j])
    out = {}
    for n, per in acc.items():
        los, his = [], []
        for v in per.values():
            a = np.concatenate(v)
            los.append(np.percentile(a, 0.5)); his.append(np.percentile(a, 99.5))
        out[n] = (float(np.mean(los)), float(np.std(los)),
                  float(np.mean(his)), float(np.std(his)), len(los))
    return out, nf


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dirs", nargs="+", required=True)
    ap.add_argument("--side", default="left")
    ap.add_argument("--sd", type=float, default=1.0)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    env, nf = envelope(args.dirs)
    print("정제 유효 프레임 %d, 피험자 %d명" % (nf, len(KEEP)))

    fk = HumanHandFK(args.side)
    m, d = fk.model, fk.data
    q0 = pin.neutral(m)
    pin.computeJointJacobians(m, d, q0)
    pin.updateFramePlacements(m, d)
    Rd = d.oMf[fk._palm_fid].rotation

    lim_out, ax_out, names_l, names_a = [], [], [], []
    print("\n%-18s %2s %15s %17s  %s" % ("관절", "급", "현재", "제안", "근거"))
    for short, b in MAP.items():
        jn = args.side + "_" + short
        grp = short.split("_")[0]
        g = GRADE[grp]
        lo0 = np.degrees(m.lowerPositionLimit[fk._idx_q[jn]])
        hi0 = np.degrees(m.upperPositionLimit[fk._idx_q[jn]])
        osr = bc.OSIM_RANGE.get(b)
        e = env.get(b)
        if g == "C" or e is None:
            print("%-18s %2s [%+6.0f,%+6.0f] %17s  변경 없음(근거 부족)"
                  % (short, g, lo0, hi0, "-"))
            continue
        mlo = e[0] - args.sd * e[1]
        mhi = e[2] + args.sd * e[3]
        if g == "A":
            lo = min(lo0, mlo, osr[0]) if osr else min(lo0, mlo)
            hi = max(hi0, mhi, osr[1]) if osr else max(hi0, mhi)
        else:
            lo = min(lo0, mlo)
            hi = max(hi0, mhi)
        lo = float(5.0 * np.floor(lo / 5.0))
        hi = float(5.0 * np.ceil(hi / 5.0))
        lim_out.append((lo, hi)); names_l.append(short)
        src = "측정[%+.0f,%+.0f] n=%d" % (mlo, mhi, e[4])
        print("%-18s %2s [%+6.0f,%+6.0f] [%+7.0f,%+7.0f]  %s"
              % (short, g, lo0, hi0, lo, hi, src))

        if b in BHAM_AXIS and grp != "thumb":
            j = m.getJointId(jn)
            v = np.zeros(m.nv); v[m.joints[j].idx_v] = 1.0
            w = pin.getJointJacobian(m, d, j, pin.LOCAL_WORLD_ALIGNED)[3:6] @ v
            cur_d = Rd.T @ w / max(np.linalg.norm(w), 1e-12)
            tgt = np.array(BHAM_AXIS[b], float)
            tgt /= np.linalg.norm(tgt)
            if tgt @ cur_d < 0:
                tgt = -tgt
            Rj = d.oMi[j].rotation
            loc = Rj.T @ (Rd @ tgt)
            loc /= np.linalg.norm(loc)
            ax_out.append(loc); names_a.append(short)

    np.savez(args.out + "_limits.npz", names=np.array(names_l),
             lo=np.array([a for a, _ in lim_out]),
             hi=np.array([b for _, b in lim_out]))
    np.savez(args.out + "_axes.npz", names=np.array(names_a),
             xyz=np.array(ax_out))
    print("\n한계 %d개 → %s_limits.npz" % (len(names_l), args.out))
    print("축 %d개(손가락만, 부호 보존) → %s_axes.npz" % (len(names_a), args.out))


if __name__ == "__main__":
    main()
