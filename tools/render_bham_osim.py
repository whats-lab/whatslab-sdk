#!/usr/bin/env python3
import argparse
import base64
import os
import xml.etree.ElementTree as ET
import zlib

import numpy as np


def rpy(v):
    a, b, c = v
    ca, sa, cb, sb, cc, sc = (np.cos(a), np.sin(a), np.cos(b),
                              np.sin(b), np.cos(c), np.sin(c))
    Rx = np.array([[1, 0, 0], [0, ca, -sa], [0, sa, ca]])
    Ry = np.array([[cb, 0, sb], [0, 1, 0], [-sb, 0, cb]])
    Rz = np.array([[cc, -sc, 0], [sc, cc, 0], [0, 0, 1]])
    return Rx @ Ry @ Rz


def T(R, p):
    M = np.eye(4)
    M[:3, :3] = R
    M[:3, 3] = p
    return M


def vec(node, tag, n=3):
    t = node.findtext(tag)
    return np.array([float(x) for x in t.split()]) if t else np.zeros(n)


class Osim:

    def __init__(self, path):
        r = ET.parse(path).getroot()
        self.joints, self.bodies = {}, {}
        for b in r.iter("Body"):
            g = []
            for m in b.iter("Mesh"):
                g.append((m.findtext("mesh_file").strip(),
                          vec(m, "scale_factors") if m.findtext("scale_factors")
                          else np.ones(3)))
            self.bodies[b.get("name")] = g
        for j in r.iter():
            if not j.tag.endswith("Joint"):
                continue
            pf = (j.findtext("socket_parent_frame") or "").split("/")[-1]
            cf = (j.findtext("socket_child_frame") or "").split("/")[-1]
            local = {}
            for f in j.iter("PhysicalOffsetFrame"):
                local[f.get("name")] = {
                    "parent": (f.findtext("socket_parent") or "").split("/")[-1],
                    "t": vec(f, "translation"), "o": vec(f, "orientation")}
            axes, coords = [], []
            for ta in j.iter("TransformAxis"):
                co = (ta.findtext("coordinates") or "").strip()
                ax = vec(ta, "axis")
                if co:
                    axes.append((co, ax))
            for c in j.iter("Coordinate"):
                coords.append((c.get("name"),
                               float(c.findtext("default_value") or 0.0)))
            fp = local.get(pf, {"parent": pf, "t": np.zeros(3), "o": np.zeros(3)})
            fc = local.get(cf, {"parent": cf, "t": np.zeros(3), "o": np.zeros(3)})
            self.joints[j.get("name")] = {
                "fp": fp, "fc": fc, "axes": axes, "coords": dict(coords),
                "parent": fp["parent"], "child": fc["parent"]}

    def fk(self, qmap=None):
        qmap = qmap or {}
        pose = {}
        ground = {j["parent"] for j in self.joints.values()} - {
            j["child"] for j in self.joints.values()}
        root = "ground" if "ground" in ground else (sorted(ground)[0] if ground
                                                    else None)
        pose[root] = np.eye(4)
        todo = dict(self.joints)
        for _ in range(len(todo) + 2):
            for name, j in list(todo.items()):
                if j["parent"] not in pose:
                    continue
                fp, fc = j["fp"], j["fc"]
                Tp = T(rpy(fp["o"]), fp["t"])
                Tc = T(rpy(fc["o"]), fc["t"])
                Rj = np.eye(3)
                for co, ax in j["axes"]:
                    ang = qmap.get(co, j["coords"].get(co, 0.0))
                    n = np.linalg.norm(ax)
                    if n < 1e-9 or abs(ang) < 1e-12:
                        continue
                    a = ax / n
                    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]],
                                  [-a[1], a[0], 0]])
                    Rj = Rj @ (np.eye(3) + np.sin(ang) * K
                               + (1 - np.cos(ang)) * K @ K)
                pose[j["child"]] = pose[j["parent"]] @ Tp @ T(Rj, np.zeros(3)) \
                    @ np.linalg.inv(Tc)
                del todo[name]
        return pose


def read_vtp(path):
    r = ET.parse(path).getroot()
    pc = r.find(".//Piece")
    pts = None
    for da in pc.find("Points").iter("DataArray"):
        if da.get("NumberOfComponents") == "3":
            pts = decode(da).reshape(-1, 3)
            break
    polys, offs = None, None
    P = pc.find("Polys")
    if P is not None:
        for da in P.iter("DataArray"):
            nm = da.get("Name")
            if nm == "connectivity":
                polys = decode(da, int)
            elif nm == "offsets":
                offs = decode(da, int)
    faces = []
    if polys is not None and offs is not None:
        s = 0
        for e in offs:
            idx = polys[s:e]
            for k in range(1, len(idx) - 1):
                faces.append([idx[0], idx[k], idx[k + 1]])
            s = e
    return pts, np.array(faces, dtype=int) if faces else None


def decode(da, dt=float):
    nt = {"Float32": np.float32, "Float64": np.float64,
          "Int32": np.int32, "Int64": np.int64}.get(da.get("type"), np.float32)
    txt = da.text or ""
    if (da.get("format") or "ascii") == "ascii":
        return np.fromstring(txt, sep=" ").astype(nt)
    t = "".join(txt.split())
    hlen = 8 * (3 + int(np.frombuffer(
        base64.b64decode(t[:32] + "==")[:8], np.int64)[0]))
    b64h = ((hlen + 2) // 3) * 4
    hdr = base64.b64decode(t[:b64h] + "===")[:hlen]
    nb = int(np.frombuffer(hdr[:8], np.int64)[0])
    sizes = np.frombuffer(hdr[24:24 + 8 * nb], np.int64)
    body = base64.b64decode(t[b64h:] + "===")
    out, off = [], 0
    for sz in sizes:
        out.append(zlib.decompress(body[off:off + int(sz)]))
        off += int(sz)
    return np.frombuffer(b"".join(out), nt)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--osim", required=True)
    ap.add_argument("--geom", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--pose", default=None,
                    help="coord=deg,... 예: 2mcp_flexion=60,2pm_flexion=80")
    ap.add_argument("--transforms", default=None,
                    help="OpenSim API 가 낸 body 변환 json (정본). 주면 자체 FK 대신 이걸 쓴다")
    ap.add_argument("--hand-only", action="store_true")
    ap.add_argument("--azim", type=float, default=-62.0)
    ap.add_argument("--elev", type=float, default=18.0)
    args = ap.parse_args()

    o = Osim(args.osim)
    if args.transforms:
        import json
        ref = json.load(open(args.transforms))["bodies"]
        pose = {}
        for b, v in ref.items():
            M = np.eye(4)
            M[:3, :3] = np.array(v["R"])
            M[:3, 3] = np.array(v["p"])
            pose[b] = M
        src = "OpenSim API"
    else:
        qmap = {}
        if args.pose:
            for kv in args.pose.split(","):
                k, v = kv.split("=")
                qmap[k] = np.deg2rad(float(v))
        pose = o.fk(qmap)
        src = "자체 FK"

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    fig = plt.figure(figsize=(6.4, 6.4), facecolor="#111")
    ax = fig.add_subplot(111, projection="3d", facecolor="#111")
    HAND = ("proximal_row", "capitate", "trapezium", "trapezoid", "hamate",
            "firstmc", "proximal_thumb", "distal_thumb", "secondmc", "thirdmc",
            "fourthmc", "fifthmc") + tuple(
        "%d%s" % (i, s) for i in (2, 3, 4, 5)
        for s in ("proxph", "midph", "distph"))
    allp = []
    for body, meshes in o.bodies.items():
        if body not in pose:
            continue
        if args.hand_only and body not in HAND:
            continue
        for mf, sc in meshes:
            p = os.path.join(args.geom, os.path.basename(mf))
            if not os.path.exists(p):
                continue
            V, F = read_vtp(p)
            if V is None:
                continue
            V = V * sc
            V = (pose[body][:3, :3] @ V.T).T + pose[body][:3, 3]
            allp.append(V)
            if F is not None and len(F):
                step = max(1, len(F) // 4000)
                ax.add_collection3d(Poly3DCollection(
                    V[F[::step]], facecolor="#d8d8dc", edgecolor="none",
                    alpha=1.0))
    A = np.vstack(allp)
    c = A.mean(0)
    r = 0.55 * float((A.max(0) - A.min(0)).max())
    ax.set_xlim(c[0] - r, c[0] + r); ax.set_ylim(c[1] - r, c[1] + r)
    ax.set_zlim(c[2] - r, c[2] + r)
    ax.set_box_aspect((1, 1, 1)); ax.set_axis_off()
    ax.view_init(elev=args.elev, azim=args.azim)
    fig.tight_layout()
    fig.savefig(args.out, dpi=120, facecolor="#111")
    print("%s  bodies=%d  verts=%d  기하출처=%s" % (args.out, len(allp), len(A), src))


if __name__ == "__main__":
    main()
