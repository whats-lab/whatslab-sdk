#!/usr/bin/env python3
import argparse
import os
import shutil
import xml.etree.ElementTree as ET

import numpy as np
import pinocchio as pin

from whatslab.solvers.hand.human_fk import HumanHandFK

BHAM = {
    "index_mcp_flex": "2mcp_flexion", "index_mcp_abd": "2mcp_abduction",
    "index_pip": "2pm_flexion", "index_dip": "2md_flexion",
    "middle_mcp_flex": "3mcp_flexion", "middle_mcp_abd": "3mcp_abduction",
    "middle_pip": "3pm_flexion", "middle_dip": "3md_flexion",
    "ring_mcp_flex": "4mcp_flexion", "ring_mcp_abd": "4mcp_abduction",
    "ring_pip": "4pm_flexion", "ring_dip": "4md_flexion",
    "pinky_mcp_flex": "5mcp_flexion", "pinky_mcp_abd": "5mcp_abduction",
    "pinky_pip": "5pm_flexion", "pinky_dip": "5md_flexion",
    "thumb_cmc_flex": "cmc_flexion", "thumb_cmc_abd": "cmc_abduction",
    "thumb_mcp_flex": "mp_flexion", "thumb_ip_flex": "ip_flexion",
}
BHAM_AXIS = {
    "2mcp_flexion": (0.980, 0.026, -0.198), "2mcp_abduction": (0.215, 0.192, 0.958),
    "2pm_flexion": (0.976, 0.127, -0.175), "2md_flexion": (0.986, 0.147, 0.075),
    "3mcp_flexion": (0.999, 0.047, -0.010), "3mcp_abduction": (-0.021, 0.119, 0.993),
    "3pm_flexion": (0.993, -0.062, 0.102), "3md_flexion": (0.995, 0.036, 0.097),
    "4mcp_flexion": (0.997, -0.065, -0.030), "4mcp_abduction": (-0.022, 0.144, 0.989),
    "4pm_flexion": (0.995, -0.035, 0.092), "4md_flexion": (0.953, -0.140, 0.270),
    "5mcp_flexion": (0.976, -0.214, -0.035), "5mcp_abduction": (-0.025, 0.216, 0.976),
    "5pm_flexion": (0.928, -0.328, 0.176), "5md_flexion": (0.893, -0.427, 0.145),
    "cmc_flexion": (0.042, 0.665, -0.745), "cmc_abduction": (0.496, 0.732, 0.468),
    "mp_flexion": (0.084, 0.203, -0.975), "ip_flexion": (0.050, 0.480, -0.876),
}


def our_axes(side):
    fk = HumanHandFK(side)
    m, d = fk.model, fk.data
    q = pin.neutral(m)
    pin.computeJointJacobians(m, d, q)
    pin.updateFramePlacements(m, d)
    Rd = d.oMf[fk._palm_fid].rotation
    out, lim = {}, {}
    for n in fk.joint_names:
        j = m.getJointId(n)
        v = np.zeros(m.nv)
        v[m.joints[j].idx_v] = 1.0
        w = pin.getJointJacobian(m, d, j, pin.LOCAL_WORLD_ALIGNED)[3:6] @ v
        nn = np.linalg.norm(w)
        short = n.replace(side + "_", "")
        out[short] = (Rd.T @ w) / nn if nn > 1e-9 else None
        lim[short] = (np.degrees(m.lowerPositionLimit[fk._idx_q[n]]),
                      np.degrees(m.upperPositionLimit[fk._idx_q[n]]))
    return fk, out, lim, Rd


def load_env(path):
    z = np.load(path)
    nm = [str(x) for x in z["names"]]
    return {n: dict(lo_mean=z["lo_mean"][i], lo_sd=z["lo_sd"][i],
                    hi_mean=z["hi_mean"][i], hi_sd=z["hi_sd"][i],
                    g_lo=z["g_lo"][i], g_hi=z["g_hi"][i])
            for i, n in enumerate(nm)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--env", required=True)
    ap.add_argument("--side", default="left")
    ap.add_argument("--sd", type=float, default=1.0,
                    help="개인최대가동 mean ± sd*SD 를 새 한계로")
    args = ap.parse_args()

    env = load_env(args.env)
    fk, ax, lim, Rd = our_axes(args.side)

    print("=== 관절축: 우리 URDF vs BHaM (dorsum 프레임, 부호무관 각도)")
    print("%-18s %8s   %-26s %-26s" % ("관절", "차이", "우리", "BHaM"))
    for k, b in BHAM.items():
        if k not in ax or ax[k] is None or b not in BHAM_AXIS:
            continue
        u = ax[k]
        v = np.array(BHAM_AXIS[b]); v /= np.linalg.norm(v)
        a = np.degrees(np.arccos(min(1.0, abs(u @ v))))
        print("%-18s %7.1f째  [%+.3f %+.3f %+.3f]  [%+.3f %+.3f %+.3f]"
              % (k, a, *u, *v))

    print("\n=== 관절한계 수정안 (개인최대가동 mean ± %.1f*sd, 5도 반올림)" % args.sd)
    print("%-18s %15s %17s %10s" % ("관절", "현재", "제안", "변화"))
    prop = {}
    for k, b in BHAM.items():
        if b not in env or k not in lim:
            continue
        e = env[b]
        lo = e["lo_mean"] - args.sd * e["lo_sd"]
        hi = e["hi_mean"] + args.sd * e["hi_sd"]
        lo = 5.0 * np.floor(lo / 5.0)
        hi = 5.0 * np.ceil(hi / 5.0)
        cur = lim[k]
        prop[k] = (lo, hi)
        print("%-18s [%+6.0f,%+6.0f] [%+7.0f,%+7.0f] %+6.0f/%+6.0f"
              % (k, cur[0], cur[1], lo, hi, lo - cur[0], hi - cur[1]))
    np.savez(os.path.splitext(args.env)[0] + "_proposal.npz",
             names=np.array(list(prop)),
             lo=np.array([prop[k][0] for k in prop]),
             hi=np.array([prop[k][1] for k in prop]))
    print("\n저장:", os.path.splitext(args.env)[0] + "_proposal.npz")


if __name__ == "__main__":
    main()
