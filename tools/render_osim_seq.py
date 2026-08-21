#!/usr/bin/env python3
import argparse
import json
import os
import xml.etree.ElementTree as ET

import imageio.v2 as imageio
import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

HAND = ("proximal_row", "capitate", "trapezium", "trapezoid", "hamate", "lunate",
        "PISIFORM", "SCAPHOID", "TRIQUETRAL", "1mc", "thumbprox", "thumbdist",
        "2mc", "3mc", "4mc", "5mc") + tuple(
    "%d%s" % (i, s) for i in (2, 3, 4, 5) for s in ("proxph", "midph", "distph"))


def read_vtp(path):
    pc = ET.parse(path).getroot().find(".//Piece")
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
            i = conn[s:e]
            for k in range(1, len(i) - 1):
                F.append([i[0], i[k], i[k + 1]])
            s = e
    return V, np.array(F, dtype=int)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seq", required=True)
    ap.add_argument("--geom", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--strip", default=None)
    ap.add_argument("--hand-only", action="store_true")
    ap.add_argument("--azim", type=float, default=0.0)
    ap.add_argument("--elev", type=float, default=5.0)
    ap.add_argument("--maxfaces", type=int, default=1200)
    args = ap.parse_args()

    d = json.load(open(args.seq))
    fps, seq = d["fps"], d["seq"]
    parts = []
    for it in d["decor"]:
        base = os.path.basename(it["mesh"])
        stem = base.replace(".vtp", "").replace("_new", "")
        if args.hand_only and not any(h in stem or stem in h for h in HAND):
            continue
        p = os.path.join(args.geom, base)
        if not os.path.exists(p):
            continue
        V, F = read_vtp(p)
        if V is None or not len(F):
            continue
        V = V * np.array(it["scale"])
        V = (np.array(it["R"]) @ V.T).T + np.array(it["p"])
        step = max(1, len(F) // args.maxfaces)
        parts.append((it["bodyId"], V, F[::step]))
    print("메쉬 %d개, 삼각형 %d" % (len(parts), sum(len(f) for _, _, f in parts)))

    def world(fr):
        out = []
        for bid, V, F in parts:
            m = fr[str(bid)] if str(bid) in fr else fr[bid]
            W = (np.array(m["R"]) @ V.T).T + np.array(m["p"])
            out.append((W, F))
        return out

    A = np.vstack([w for w, _ in world(seq[0])])
    allw = np.vstack([np.vstack([w for w, _ in world(f)]) for f in seq[::10]])
    c = allw.mean(0)
    r = 0.60 * float((allw.max(0) - allw.min(0)).max())

    fig = plt.figure(figsize=(5.2, 5.2), facecolor="#111")
    ax = fig.add_subplot(111, projection="3d", facecolor="#111")
    frames = []
    for k, fr in enumerate(seq):
        ax.clear()
        for W, F in world(fr):
            ax.add_collection3d(Poly3DCollection(W[F], facecolor="#dcdce0",
                                                 edgecolor="none"))
        ax.set_xlim(c[0] - r, c[0] + r); ax.set_ylim(c[1] - r, c[1] + r)
        ax.set_zlim(c[2] - r, c[2] + r)
        ax.set_box_aspect((1, 1, 1)); ax.set_axis_off()
        ax.view_init(elev=args.elev, azim=args.azim)
        ax.set_title("OpenSim FK   t=%.2fs" % (k / fps), color="w", fontsize=10)
        fig.canvas.draw()
        frames.append(np.asarray(fig.canvas.buffer_rgba())[:, :, :3].copy())
    imageio.mimwrite(args.out, frames, fps=fps, quality=8)
    print("%s  %d프레임" % (args.out, len(frames)))
    if args.strip:
        sel = np.linspace(0, len(frames) - 1, 6).astype(int)
        imageio.imwrite(args.strip, np.hstack([frames[i] for i in sel]))
        print(args.strip)


if __name__ == "__main__":
    main()
