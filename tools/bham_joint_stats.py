#!/usr/bin/env python3
import argparse
import glob
import os
import xml.etree.ElementTree as ET

import numpy as np

HAND_JOINTS = ("CMC1a", "CMC1b", "MCP", "IP", "CMC4", "CMC5",
               "_2MCP", "_2prox-midph_b", "_2mid-distph",
               "_3MCP", "_3prox-midph_b", "_3mid-distph",
               "_4MCP", "_4prox-midph_b", "_4mid-distph",
               "_5MCP", "_5prox-midph_b", "_5mid-distph")
OURS = {
    "2mcp_flexion": "index_mcp_flex", "2mcp_abduction": "index_mcp_abd",
    "2pm_flexion": "index_pip", "2md_flexion": "index_dip",
    "3mcp_flexion": "middle_mcp_flex", "3mcp_abduction": "middle_mcp_abd",
    "3pm_flexion": "middle_pip", "3md_flexion": "middle_dip",
    "4mcp_flexion": "ring_mcp_flex", "4mcp_abduction": "ring_mcp_abd",
    "4pm_flexion": "ring_pip", "4md_flexion": "ring_dip",
    "5mcp_flexion": "pinky_mcp_flex", "5mcp_abduction": "pinky_mcp_abd",
    "5pm_flexion": "pinky_pip", "5md_flexion": "pinky_dip",
    "cmc_flexion": "thumb_cmc_flex", "cmc_abduction": "thumb_cmc_abd",
    "mp_flexion": "thumb_mcp_flex", "ip_flexion": "thumb_ip_flex",
}
SANE = np.deg2rad(400.0)


def parse(path):
    root = ET.parse(path).getroot()
    out = {}
    for j in root.iter():
        if not j.tag.endswith("Joint") or j.get("name") not in HAND_JOINTS:
            continue
        jn = j.get("name")
        rng, clamped = {}, {}
        for c in j.iter("Coordinate"):
            lo, hi = [float(x) for x in c.find("range").text.split()]
            rng[c.get("name")] = (lo, hi)
            clamped[c.get("name")] = (c.findtext("clamped") or "").strip() == "true"
        axes = {}
        for ta in j.iter("TransformAxis"):
            co = (ta.findtext("coordinates") or "").strip()
            ax = ta.findtext("axis")
            if co and ax:
                axes[co] = np.array([float(x) for x in ax.split()])
        tr = {}
        for f in j.iter("PhysicalOffsetFrame"):
            v = f.findtext("translation")
            if v:
                tr[f.get("name")] = np.array([float(x) for x in v.split()])
        for co in rng:
            out[co] = {"joint": jn, "range": rng[co], "clamped": clamped[co],
                       "axis": axes.get(co), "offsets": tr}
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", required=True)
    ap.add_argument("--side", default="left")
    args = ap.parse_args()

    files = sorted(glob.glob(os.path.join(args.dir, "*.osim")))
    subj = {}
    for f in files:
        tag = os.path.basename(f).replace("BHaM_", "").replace("_Scaled.osim", "")
        subj[tag] = parse(f)
    print("모델 %d개: %s" % (len(subj), ", ".join(sorted(subj)[:6]) + " ..."))

    coords = sorted({c for v in subj.values() for c in v}, key=lambda c: OURS.get(c, "zz" + c))
    from whatslab.solvers.hand.human_fk import HumanHandFK
    fk = HumanHandFK(args.side)
    ourlim = {}
    for n in fk.joint_names:
        s = n.replace(args.side + "_", "")
        ourlim[s] = (np.degrees(fk.model.lowerPositionLimit[fk._idx_q[n]]),
                     np.degrees(fk.model.upperPositionLimit[fk._idx_q[n]]))

    print("\n%-18s %-14s %7s %19s %19s %6s" % (
        "BHaM coordinate", "our joint", "clamp", "BHaM lo (mean±sd)",
        "BHaM hi (mean±sd)", "우리"))
    for c in coords:
        los, his, cl = [], [], 0
        for v in subj.values():
            if c not in v:
                continue
            lo, hi = v[c]["range"]
            if abs(lo) > SANE or abs(hi) > SANE:
                continue
            los.append(np.degrees(lo)); his.append(np.degrees(hi))
            cl += bool(v[c]["clamped"])
        if not los:
            print("%-18s %-14s   범위 표기가 전부 비정상(무제한)" % (c, OURS.get(c, "-")))
            continue
        o = ourlim.get(OURS.get(c, ""), None)
        print("%-18s %-14s %3d/%-3d %8.1f ± %-6.1f %8.1f ± %-6.1f %s"
              % (c, OURS.get(c, "-"), cl, len(los), np.mean(los), np.std(los),
                 np.mean(his), np.std(his),
                 ("[%+.0f,%+.0f]" % o) if o else "-"))

    print("\n=== 관절축 (BHaM, 피험자 간 편차)")
    print("%-18s %-14s %26s %9s" % ("coordinate", "our joint", "축 mean", "편차도"))
    for c in coords:
        A = [v[c]["axis"] for v in subj.values() if c in v and v[c]["axis"] is not None]
        if not A:
            continue
        A = np.array([a / np.linalg.norm(a) for a in A])
        m = A.mean(0); m /= np.linalg.norm(m)
        sp = np.degrees(np.arccos(np.clip(A @ m, -1, 1)))
        print("%-18s %-14s  [%+.3f %+.3f %+.3f] %8.2f"
              % (c, OURS.get(c, "-"), *m, sp.max()))


if __name__ == "__main__":
    main()
