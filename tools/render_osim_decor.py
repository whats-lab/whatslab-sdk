#!/usr/bin/env python3
import argparse
import json
import os
import xml.etree.ElementTree as ET

import numpy as np

HAND_BONES = ("proximal_row", "capitate", "trapezium", "trapezoid", "hamate",
              "lunate", "PISIFORM", "SCAPHOID", "TRIQUETRAL", "1mc", "thumbprox",
              "thumbdist", "2mc", "3mc", "4mc", "5mc") + tuple(
    "%d%s" % (i, s) for i in (2, 3, 4, 5) for s in ("proxph", "midph", "distph"))


def read_vtp(path):
    r = ET.parse(path).getroot()
    pc = r.find(".//Piece")
    V = None
    for da in pc.find("Points").iter("DataArray"):
        if da.get("NumberOfComponents") == "3":
            V = np.fromstring(da.text, sep=" ").reshape(-1, 3)
            break
    conn = offs = None
    P = pc.find("Polys")
    if P is not None:
        for da in P.iter("DataArray"):
            if da.get("Name") == "connectivity":
                conn = np.fromstring(da.text, sep=" ").astype(int)
            elif da.get("Name") == "offsets":
                offs = np.fromstring(da.text, sep=" ").astype(int)
    F = []
    if conn is not None and offs is not None:
        s = 0
        for e in offs:
            idx = conn[s:e]
            for k in range(1, len(idx) - 1):
                F.append([idx[0], idx[k], idx[k + 1]])
            s = e
    return V, (np.array(F, dtype=int) if F else None)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--decor", required=True)
    ap.add_argument("--geom", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--hand-only", action="store_true")
    ap.add_argument("--azim", type=float, default=0.0)
    ap.add_argument("--elev", type=float, default=5.0)
    ap.add_argument("--maxfaces", type=int, default=6000)
    args = ap.parse_args()

    d = json.load(open(args.decor))
    mob = {int(k): v for k, v in d["mobod"].items()}

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    fig = plt.figure(figsize=(6.4, 6.4), facecolor="#111")
    ax = fig.add_subplot(111, projection="3d", facecolor="#111")
    allp, used = [], 0
    for it in d["decor"]:
        base = os.path.basename(it["mesh"])
        stem = base.replace(".vtp", "").replace("_new", "")
        if args.hand_only and not any(h in stem or stem in h for h in HAND_BONES):
            continue
        p = os.path.join(args.geom, base)
        if not os.path.exists(p):
            continue
        V, F = read_vtp(p)
        if V is None:
            continue
        Rg = np.array(mob[it["bodyId"]]["R"])
        pg = np.array(mob[it["bodyId"]]["p"])
        Rl = np.array(it["R"])
        pl = np.array(it["p"])
        V = V * np.array(it["scale"])
        V = (Rl @ V.T).T + pl
        V = (Rg @ V.T).T + pg
        allp.append(V); used += 1
        if F is not None and len(F):
            step = max(1, len(F) // args.maxfaces)
            ax.add_collection3d(Poly3DCollection(
                V[F[::step]], facecolor="#dcdce0", edgecolor="none"))
    A = np.vstack(allp)
    c = A.mean(0)
    r = 0.55 * float((A.max(0) - A.min(0)).max())
    ax.set_xlim(c[0] - r, c[0] + r); ax.set_ylim(c[1] - r, c[1] + r)
    ax.set_zlim(c[2] - r, c[2] + r)
    ax.set_box_aspect((1, 1, 1)); ax.set_axis_off()
    ax.view_init(elev=args.elev, azim=args.azim)
    fig.tight_layout()
    fig.savefig(args.out, dpi=120, facecolor="#111")
    print("%s  meshes=%d verts=%d" % (args.out, used, len(A)))


if __name__ == "__main__":
    main()
