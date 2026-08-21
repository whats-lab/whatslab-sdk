#!/usr/bin/env python3
import argparse
import os
import re

import imageio.v2 as imageio
import mujoco
import numpy as np

from whatslab.paths import models_root

TMP = os.environ.get("WHATSLAB_RENDER_TMP", "/tmp/whatslab_render")
MJ_BLOCK = ('<mujoco><compiler balanceinertia="true" discardvisual="false"'
            ' fusestatic="false"/><visual><global offwidth="%d" offheight="%d"/>'
            '<headlight ambient="0.5 0.5 0.5" diffuse="0.6 0.6 0.6"/>'
            '</visual></mujoco>')


def abs_urdf(path, tag, w, h):
    os.makedirs(TMP, exist_ok=True)
    s = open(path).read()
    s = s.replace("package://dexhand_description/", models_root().rstrip("/") + "/")
    s = re.sub(r'<mass\s+value="([0-9.eE+-]+)"\s*/>',
               lambda m: '<mass value="%g"/>' % max(float(m.group(1)), 1e-3), s)
    for ax in ("ixx", "iyy", "izz"):
        s = re.sub(r'%s="([0-9.eE+-]+)"' % ax,
                   lambda m, a=ax: '%s="%g"' % (a, max(float(m.group(1)), 1e-6)), s)
    if "<mujoco>" not in s:
        s = re.sub(r"(<robot[^>]*>)", r"\1" + (MJ_BLOCK % (w, h)), s, count=1)
    out = os.path.join(TMP, "abs_%s.urdf" % tag)
    open(out, "w").write(s)
    return out


def lit_model(urdf, tag, w, h):
    m0 = mujoco.MjModel.from_xml_path(abs_urdf(urdf, tag, w, h))
    xml = os.path.join(TMP, "mjcf_%s.xml" % tag)
    mujoco.mj_saveLastXML(xml, m0)
    s = open(xml).read()
    add = ('<visual><global offwidth="%d" offheight="%d"/>'
           '<headlight ambient="0.6 0.6 0.6" diffuse="0.8 0.8 0.8"'
           ' specular="0.1 0.1 0.1"/></visual>' % (w, h))
    s = s.replace("<worldbody>",
                  add + '<worldbody><light pos="0.3 -0.3 0.6" dir="-1 1 -2"'
                  ' diffuse="0.9 0.9 0.9"/><light pos="-0.3 0.3 0.6"'
                  ' dir="1 -1 -2" diffuse="0.5 0.5 0.5"/>', 1)
    open(xml, "w").write(s)
    m = mujoco.MjModel.from_xml_path(xml)
    for g in range(m.ngeom):
        if m.geom_rgba[g][:3].max() < 0.45:
            m.geom_rgba[g][:3] = (0.72, 0.74, 0.78)
    return m


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--npz", required=True)
    ap.add_argument("--side", default="left")
    ap.add_argument("--out", required=True)
    ap.add_argument("--fps", type=int, default=25)
    ap.add_argument("--stride", type=int, default=4)
    ap.add_argument("--size", type=int, default=520)
    ap.add_argument("--azimuth", type=float, default=0.0)
    ap.add_argument("--elevation", type=float, default=0.0)
    ap.add_argument("--zoom", type=float, default=1.15)
    ap.add_argument("--strip", default=None, help="키프레임 스트립 png")
    ap.add_argument("--strip-n", type=int, default=8)
    args = ap.parse_args()

    d = np.load(args.npz)
    q = d["q"]
    names = [str(n) for n in d["joint_names"]]
    urdf = os.path.join(models_root(), "base_hand", "urdf", "%s.urdf" % args.side)
    W = H = args.size
    model = lit_model(urdf, "human_%s" % args.side, W, H)
    data = mujoco.MjData(model)
    ren = mujoco.Renderer(model, H, W)

    adr, miss = {}, []
    for i, n in enumerate(names):
        j = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)
        if j < 0:
            miss.append(n)
        else:
            adr[i] = model.jnt_qposadr[j]
    if miss:
        print("[warn] MuJoCo 에 없는 관절 %d개: %s" % (len(miss), miss[:4]))

    mujoco.mj_forward(model, data)
    r = model.geom_rbound[:, None]
    lo = (data.geom_xpos - r).min(axis=0)
    hi = (data.geom_xpos + r).max(axis=0)
    cam = mujoco.MjvCamera()
    cam.lookat[:] = 0.5 * (lo + hi)
    cam.distance = args.zoom * float(np.linalg.norm(hi - lo))
    cam.azimuth, cam.elevation = args.azimuth, args.elevation

    idx = range(0, len(q), args.stride)
    frames = []
    for k in idx:
        for i, a in adr.items():
            data.qpos[a] = q[k, i]
        mujoco.mj_forward(model, data)
        ren.update_scene(data, camera=cam)
        frames.append(ren.render())
    imageio.mimwrite(args.out, frames, fps=args.fps, quality=8)
    print("%s  %d프레임 (원본 %d, stride %d) %dx%d"
          % (args.out, len(frames), len(q), args.stride, W, H))

    if args.strip:
        sel = np.linspace(0, len(frames) - 1, args.strip_n).astype(int)
        imageio.imwrite(args.strip, np.hstack([frames[i] for i in sel]))
        print("%s  키프레임 %d장" % (args.strip, len(sel)))


if __name__ == "__main__":
    main()
