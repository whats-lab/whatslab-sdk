#!/usr/bin/env python3
"""실측 q(npz)를 사람 손 + 로봇 손 나란히 MuJoCo 로 렌더해 영상으로 낸다.

MuJoCo 는 URDF 의 package:// 를 못 풀고, 사람 손 URDF 는 질량이 0 이라 로드를
거부하며(mjMINVAL), URDF 안의 <mujoco> 블록은 <visual> 을 무시한다. 그래서
경로 치환 + 최소 질량·관성 주입 + MJCF 변환 후 조명 삽입을 거친다.

카메라는 각 손의 팜 프레임(kv.origin/kv.rot/l_ref)에서 유도하므로 모든 뷰가 같은
방향에서 손바닥을 본다. orca 는 URDF 상 손가락이 아래로 나와 화면에서 180도 돌린다.
"""
import argparse
import os
import re

import imageio.v2 as imageio
import mujoco
import numpy as np

from whatslab.paths import models_root
from whatslab.solvers.hand.hand_configs import CONFIG_REGISTRY
from whatslab.solvers.hand.net_retargeter import NetHandRetargeter

TMP = os.environ.get("WHATSLAB_RENDER_TMP", "/tmp/whatslab_render")
W, H = 480, 480
ROLL180 = ("orca_hand",)


MJ_BLOCK = ('<mujoco><compiler balanceinertia="true" '
            'discardvisual="false" fusestatic="false"/>'
            '<visual><global offwidth="%d" offheight="%d"/>'
            '<headlight ambient="0.5 0.5 0.5" diffuse="0.6 0.6 0.6"/>'
            '</visual></mujoco>')


def abs_urdf(path, tag):
    os.makedirs(TMP, exist_ok=True)
    s = open(path).read()
    s = re.sub(r"package://dexhand_description/", models_root().rstrip("/") + "/", s)
    s = re.sub(r'<mass\s+value="([0-9.eE+-]+)"\s*/>',
               lambda m: '<mass value="%g"/>' % max(float(m.group(1)), 1e-3), s)
    s = re.sub(r'ixx="([0-9.eE+-]+)"',
               lambda m: 'ixx="%g"' % max(float(m.group(1)), 1e-6), s)
    s = re.sub(r'iyy="([0-9.eE+-]+)"',
               lambda m: 'iyy="%g"' % max(float(m.group(1)), 1e-6), s)
    s = re.sub(r'izz="([0-9.eE+-]+)"',
               lambda m: 'izz="%g"' % max(float(m.group(1)), 1e-6), s)
    if "<mujoco>" not in s:
        s = re.sub(r"(<robot[^>]*>)", r"\1" + (MJ_BLOCK % (W, H)), s, count=1)
    out = os.path.join(TMP, "abs_%s.urdf" % tag)
    open(out, "w").write(s)
    return out


def lit_model(urdf, tag):
    m0 = mujoco.MjModel.from_xml_path(abs_urdf(urdf, tag))
    xml = os.path.join(TMP, "mjcf_%s.xml" % tag)
    mujoco.mj_saveLastXML(xml, m0)
    s = open(xml).read()
    add = ('<visual><global offwidth="%d" offheight="%d"/>'
           '<headlight ambient="0.6 0.6 0.6" diffuse="0.8 0.8 0.8"'
           ' specular="0.1 0.1 0.1"/></visual>' % (W, H))
    s = s.replace("<worldbody>",
                  add + '<worldbody><light pos="0.3 -0.3 0.6" dir="-1 1 -2"'
                  ' diffuse="0.9 0.9 0.9"/>'
                  '<light pos="-0.3 0.3 0.6" dir="1 -1 -2"'
                  ' diffuse="0.5 0.5 0.5"/>', 1)
    open(xml, "w").write(s)
    m = mujoco.MjModel.from_xml_path(xml)
    for g in range(m.ngeom):
        if m.geom_rgba[g][:3].max() < 0.45:
            m.geom_rgba[g][:3] = (0.72, 0.74, 0.78)
    return m


def cam_from_palm(origin, rot, l_ref, flip):
    n = np.asarray(rot)[:, 2] * (-1.0 if flip else 1.0)
    n = n / np.linalg.norm(n)
    look = np.asarray(origin) + np.asarray(rot)[:, 1] * (0.62 * l_ref)
    az = float(np.degrees(np.arctan2(n[1], n[0])))
    el = float(np.degrees(np.arcsin(np.clip(n[2], -1.0, 1.0))))
    return look, az, el, 3.1 * l_ref


class View:

    def __init__(self, urdf, tag, palm=None, roll180=False):
        self.roll180 = bool(roll180)
        self.model = lit_model(urdf, tag)
        self.data = mujoco.MjData(self.model)
        self.ren = mujoco.Renderer(self.model, H, W)
        self.qadr = {}
        for j in range(self.model.njnt):
            n = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, j)
            self.qadr[n] = int(self.model.jnt_qposadr[j])
        mujoco.mj_forward(self.model, self.data)
        self.cam = mujoco.MjvCamera()
        self.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        if palm is None:
            p = self.data.geom_xpos
            self.cam.lookat[:] = p.mean(axis=0)
            self.cam.distance = 1.75 * float(np.ptp(p, axis=0).max() + 1e-3)
            self.cam.azimuth, self.cam.elevation = 140.0, -20.0
        else:
            look, az, el, dist = palm
            self.cam.lookat[:] = look
            self.cam.azimuth, self.cam.elevation = az, el
            self.cam.distance = dist
        self.opt = mujoco.MjvOption()
        mujoco.mjv_defaultOption(self.opt)

    def frame(self, q_named):
        self.data.qpos[:] = 0.0
        for n, v in q_named.items():
            if n in self.qadr:
                self.data.qpos[self.qadr[n]] = float(v)
        mujoco.mj_forward(self.model, self.data)
        self.ren.update_scene(self.data, self.cam, self.opt)
        img = self.ren.render()
        return img[::-1, ::-1] if self.roll180 else img


def label(img, text):
    from PIL import Image, ImageDraw
    im = Image.fromarray(img)
    ImageDraw.Draw(im).text((12, 12), text, fill=(255, 230, 60))
    return np.asarray(im)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--npz", required=True)
    ap.add_argument("--pairs", nargs="+", required=True)
    ap.add_argument("--side", default="left")
    ap.add_argument("--out", required=True)
    ap.add_argument("--stride", type=int, default=4)
    ap.add_argument("--fps", type=int, default=25)
    ap.add_argument("--max-frames", type=int, default=400)
    ap.add_argument("--flip", action="store_true")
    a = ap.parse_args()

    rec = np.load(a.npz, allow_pickle=False)
    names = [str(n) for n in rec["joint_names"]]
    Q = np.asarray(rec["q"], dtype=float)[::a.stride][:a.max_frames]
    print("프레임 %d (stride %d)" % (len(Q), a.stride), flush=True)

    human_urdf = os.path.join(models_root(), "base_hand", "urdf",
                              "%s.urdf" % a.side)
    views = []
    for spec in a.pairs:
        tag, cfg, ck = spec.split("=")
        r = NetHandRetargeter(a.side, cfg, checkpoint=ck)
        u = CONFIG_REGISTRY[cfg]()._get_urdf_path(a.side)
        pl = cam_from_palm(r.kv.origin, r.kv.rot, r.kv.l_ref, a.flip)
        views.append((tag, View(u, tag, pl, cfg in ROLL180), r))
        if not views[:1] or views[0][0] != "human":
            hp = cam_from_palm(r.hkv.origin, r.hkv.rot, r.hkv.l_ref, a.flip)
            views.insert(0, ("human", View(human_urdf, "human", hp), None))

    frames = []
    for i, q in enumerate(Q):
        row = []
        for tag, v, r in views:
            if r is None:
                row.append(label(v.frame(dict(zip(names, q))), tag))
            else:
                ang = dict(zip(names, q))
                qa = r.compute(ang)
                row.append(label(v.frame(dict(zip(r.joint_names, qa))), tag))
        frames.append(np.concatenate(row, axis=1))
        if i % 25 == 0:
            print("  %d/%d" % (i, len(Q)), flush=True)
    imageio.mimsave(a.out, frames, fps=a.fps, quality=8)
    print("저장", a.out, len(frames), "프레임")


if __name__ == "__main__":
    main()
