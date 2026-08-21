import argparse

import numpy as np

LG_PIP = np.array([0.951737167, 0.91851675, 0.928536917, 0.8512485, 1.3543025])
LG_MCP_f = np.array([0.5947195, 0.83472675, 0.565198583, 0.86111525, 0.8560715])
LG_MCP_a = np.array([-0.418572417, 0.249356273, 0.207084273, -0.1545799,
                     -0.005201221, 0.040710132, -0.002326209, 0.007637062,
                     -0.157234591, 0.001796508, 0.422011364, 0.005946804,
                     -0.006997746, -0.116095218, 0.000932049, 0.302371273,
                     0.002152977, -0.005472833])
LG_CMC5 = 0.160296182
LG_CMC1 = np.array([0.789672, 0.625676, -0.248543545, 0.022100016])
LG_abd = LG_MCP_a[:3]
LG_rel = LG_MCP_a[3:].reshape(3, 5)
DIP_COEF = ((0.87, 25.27), (0.79, 18.33), (0.73, 20.54), (0.84, 12.42))

OUR = ["index_mcp_flex", "index_mcp_abd", "index_pip", "index_dip",
       "middle_mcp_flex", "middle_mcp_abd", "middle_pip", "middle_dip",
       "pinky_mcp_flex", "pinky_mcp_abd", "pinky_pip", "pinky_dip",
       "ring_mcp_flex", "ring_mcp_abd", "ring_pip", "ring_dip",
       "thumb_cmc_flex", "thumb_cmc_abd", "thumb_mcp_abd", "thumb_mcp_flex",
       "thumb_ip_flex"]


def read_glove(path):
    return np.loadtxt(path, skiprows=2, usecols=range(24))


def to_angles(d, ref):
    c = lambda k: d[:, k - 1] - ref[k - 1]
    mcp2, mcp3, mcp4, mcp5 = c(6), c(10), c(14), c(18)
    abd2, abd4, abd5 = c(13), c(17), c(21)

    def rel(row, x, y):
        g = LG_rel[row]
        return g[0] * x + g[1] * x ** 2 + g[2] * y + g[3] * y ** 2 + g[4] * x * y

    a = {}
    a["CMC1_f"] = c(2) * LG_CMC1[0] + c(5) * LG_CMC1[1]
    a["CMC1_a"] = c(5) * LG_CMC1[2] + c(2) * LG_CMC1[3]
    a["MCP1"] = c(3) * LG_MCP_f[0]
    a["IP1"] = c(4) * LG_PIP[0]
    a["MCP2_f"] = mcp2 * LG_MCP_f[1]
    a["MCP2_a"] = abd2 * LG_abd[0] + rel(0, mcp2, mcp3)
    a["PIP2"] = c(7) * LG_PIP[1]
    a["MCP3_f"] = mcp3 * LG_MCP_f[2]
    a["MCP3_a"] = np.zeros(len(d))
    a["PIP3"] = c(11) * LG_PIP[2]
    a["MCP4_f"] = mcp4 * LG_MCP_f[3]
    a["MCP4_a"] = -(abd4 * LG_abd[1] + rel(1, mcp3, mcp4))
    a["PIP4"] = c(15) * LG_PIP[3]
    a["MCP5_f"] = mcp5 * LG_MCP_f[4]
    a["MCP5_a"] = -(abd5 * LG_abd[2] + rel(2, mcp4, mcp5))
    a["PIP5"] = c(19) * LG_PIP[4]
    a["PalmarArch"] = c(22) * LG_CMC5
    for k, (fn, (g, off)) in enumerate(zip(("PIP2", "PIP3", "PIP4", "PIP5"),
                                           DIP_COEF)):
        a["DIP%d" % (k + 2)] = np.maximum(0.0, g * a[fn] - off)
    return a


def to_urdf_q(a, n):
    q = np.zeros((n, len(OUR)))
    src = {
        "index_mcp_flex": "MCP2_f", "index_mcp_abd": "MCP2_a",
        "index_pip": "PIP2", "index_dip": "DIP2",
        "middle_mcp_flex": "MCP3_f", "middle_mcp_abd": "MCP3_a",
        "middle_pip": "PIP3", "middle_dip": "DIP3",
        "ring_mcp_flex": "MCP4_f", "ring_mcp_abd": "MCP4_a",
        "ring_pip": "PIP4", "ring_dip": "DIP4",
        "pinky_mcp_flex": "MCP5_f", "pinky_mcp_abd": "MCP5_a",
        "pinky_pip": "PIP5", "pinky_dip": "DIP5",
        "thumb_cmc_flex": "CMC1_f", "thumb_cmc_abd": "CMC1_a",
        "thumb_mcp_flex": "MCP1", "thumb_ip_flex": "IP1",
    }
    for i, name in enumerate(OUR):
        k = src.get(name)
        if k is not None:
            q[:, i] = np.deg2rad(a[k])
    return q


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--trial", required=True)
    ap.add_argument("--ref", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--side", default="left")
    args = ap.parse_args()

    d = read_glove(args.trial)
    ref = read_glove(args.ref).mean(axis=0)
    a = to_angles(d, ref)
    q = to_urdf_q(a, len(d))
    t = d[:, 0]

    import sys
    sys.path.insert(0, "/home/whatslab09/whatslab-sdk/src")
    from whatslab.solvers.hand.human_fk import HumanHandFK
    fk = HumanHandFK(args.side)
    names = [n.replace(args.side + "_", "") for n in fk.joint_names]
    assert names == OUR, (names, OUR)
    lo = np.array([fk.model.lowerPositionLimit[fk._idx_q[n]] for n in fk.joint_names])
    hi = np.array([fk.model.upperPositionLimit[fk._idx_q[n]] for n in fk.joint_names])
    n_out = int((q < lo - 1e-9).sum() + (q > hi + 1e-9).sum())
    q_cl = np.clip(q, lo, hi)

    np.savez(args.out, q=q_cl, q_raw=q, t=t,
             joint_names=np.array(fk.joint_names))
    print("%s  %d프레임 %.2f초 (%.0f Hz)" % (args.out, len(q), t[-1] - t[0],
                                          1.0 / np.median(np.diff(t))))
    print("한계 초과 샘플 %d / %d (%.2f%%) → clip" % (n_out, q.size,
                                                100.0 * n_out / q.size))
    print("\n%-18s %8s %8s %8s   %s" % ("관절", "min", "max", "std", "출처"))
    srcmap = {"middle_mcp_abd": "0 고정(기준손가락)", "thumb_mcp_abd": "미측정 → 0"}
    for i, nm in enumerate(OUR):
        v = np.degrees(q_cl[:, i])
        print("%-18s %+8.1f %+8.1f %8.2f   %s"
              % (nm, v.min(), v.max(), v.std(), srcmap.get(nm, "")))


if __name__ == "__main__":
    main()
