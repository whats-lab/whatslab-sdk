#!/usr/bin/env python3
"""손 리타게팅 백엔드를 목표 구성과 무관한 물리 지표로 비교한다 (글러브 캘리브 덤프 기준)."""
import argparse
import json
import time

import numpy as np
import torch

from whatslab.solvers.hand import HandRetargeter, KPHandRetargeter
from whatslab.solvers.hand.net_retargeter import NetHandRetargeter
from whatslab.solvers.hand.hand_configs import CONFIG_REGISTRY
from whatslab.solvers.hand.human_fk import (FINGERS, HumanHandFK, palm_frame,
                                            palm_frame_from_fingers)

SPREAD_PAIRS = (("index", "middle"), ("middle", "ring"), ("ring", "pinky"))


def theta_blocks(names):
    out = {}
    for i, n in enumerate(names):
        out.setdefault(n.split("_")[0], []).append(i)
    return out


def real_thetas(dump_path, profile_dir, steps):
    d = json.load(open(dump_path))
    prof = json.load(open("%s/%s/%s.json" % (profile_dir, d["hand_side"].lower(),
                                             d["profile"])))
    names = prof["theta"]
    hi = np.asarray(d["theta_hi"], dtype=float)
    lo = np.asarray(d["theta_lo"], dtype=float)
    rows = [np.zeros(len(names))]
    for th in d["pinch_thetas"]:
        th = np.asarray(th, dtype=float)
        rows += [th * k / (steps - 1) for k in range(1, steps)]
    for kind in ("flex", "abd"):
        sel = [i for i, n in enumerate(names) if n.endswith("_" + kind)]
        a, b = np.zeros(len(names)), np.zeros(len(names))
        for i in sel:
            a[i], b[i] = (0.0, hi[i]) if kind == "flex" else (lo[i], hi[i])
        rows += [a + (b - a) * k / (steps - 1) for k in range(1, steps)]
    return names, np.asarray(rows)


def theta_expander(dump_path, profile_dir):
    d = json.load(open(dump_path))
    side = d["hand_side"].lower()
    prof = json.load(open("%s/%s/%s.json" % (profile_dir, side, d["profile"])))
    names, scale = prof["theta"], d["finger_scale"]

    def expand(theta):
        th = {n: float(v) for n, v in zip(names, theta)}
        out = {}
        for c in prof["coupling"]:
            s = scale.get(c["scale_key"], 1.0) if c["scale_key"] else 1.0
            out[c["joint"]] = out.get(c["joint"], 0.0) + c["coeff"] * th[c["theta"]] * s
        return out

    flat = np.zeros(len(names))
    fist = np.zeros(len(names))
    hi = np.asarray(d["theta_hi"], dtype=float)
    for i, n in enumerate(names):
        if n.endswith("_flex"):
            fist[i] = hi[i]
    return expand, flat, fist


def load_poses(dump_path, profile_dir, steps, kinds=("pinch",)):
    d = json.load(open(dump_path))
    side = d["hand_side"].lower()
    prof = json.load(open("%s/%s/%s.json" % (profile_dir, side, d["profile"])))
    names, scale = prof["theta"], d["finger_scale"]

    def expand(theta):
        th = {n: float(v) for n, v in zip(names, theta)}
        if len(th) != len(names):
            raise ValueError(
                "덤프 theta %d 개와 프로파일 theta %d 개가 다르다 — 프로파일이 바뀐 뒤의"
                " 낡은 덤프다" % (len(theta), len(names)))
        out = {}
        for c in prof["coupling"]:
            s = scale.get(c["scale_key"], 1.0) if c["scale_key"] else 1.0
            out[c["joint"]] = out.get(c["joint"], 0.0) + c["coeff"] * th[c["theta"]] * s
        return out

    trajs = {}
    if "pinch" in kinds:
        for f, th in zip(d["pinch_finger_names"], d["pinch_thetas"]):
            th = np.asarray(th, dtype=float)
            trajs[f] = [expand(th * (k + 1) / steps) for k in range(steps)]
    lo = np.asarray(d["theta_lo"], dtype=float)
    hi = np.asarray(d["theta_hi"], dtype=float)
    for kind in ("flex", "abd"):
        if kind not in kinds:
            continue
        sel = [i for i, n in enumerate(names) if n.endswith("_" + kind)]
        if not sel:
            raise ValueError("프로파일 theta 에 _%s 가 없다: %s" % (kind, names))
        a, b = np.zeros(len(names)), np.zeros(len(names))
        for i in sel:
            a[i], b[i] = (0.0, hi[i]) if kind == "flex" else (lo[i], hi[i])
        trajs[kind] = [expand(a + (b - a) * (k + 1) / steps) for k in range(steps)]
    return side, trajs


LMC_MIN_MM = 0.5
THREADS = 1


def bone_axes(pts):
    out = {}
    for f in FINGERS:
        z = pts[f][-1] - pts[f][-2]
        nz = float(np.linalg.norm(z))
        if nz < 1e-9:
            raise ValueError("%s 마지막 뼈 길이가 0 이다" % f)
        out[f] = z / nz
    return out


def align_axis(w, src, dst):
    k = np.cross(src, dst)
    c = float(src @ dst)
    s2 = float(k @ k)
    if s2 < 1e-12:
        return w if c > 0.0 else -w
    kw = np.cross(k, w)
    return w + kw + np.cross(k, kw) * (1.0 - c) / s2


def motion_consistency(h_prev, h_cur, r_prev, r_cur, h_local, r_local):
    gmc, lmc = [], []
    for f in FINGERS:
        u = h_cur[f] - h_prev[f]
        if float(np.linalg.norm(u)) * 1e3 < LMC_MIN_MM:
            continue
        v = r_cur[f] - r_prev[f]
        nv = float(np.linalg.norm(v))
        if nv < 1e-9:
            gmc.append(-1.0)
            lmc.append(-1.0)
            continue
        un, vn = u / float(np.linalg.norm(u)), v / nv
        gmc.append(float(un @ vn))
        lmc.append(float(un @ align_axis(vn, r_local[f], h_local[f])))
    return gmc, lmc


class Fair:

    def __init__(self, side):
        self.fk = HumanHandFK(side)
        hp0 = self.fk.neutral_points()
        self._h0 = palm_frame({f: hp0[f][0] for f in FINGERS}, hp0["palm"])

    def local_tips(self, angles):
        hp = self.fk.points(angles)
        o, R = self._h0
        return {f: R.T @ (hp[f][-1] - o) for f in FINGERS}

    def chain_local(self, angles):
        hp = self.fk.points(angles)
        o, R = self._h0
        return {f: np.array([R.T @ (p - o) for p in hp[f]]) for f in FINGERS}

    def score(self, angles, tips, bones, r_R, r_len):
        hp = self.fk.points(angles)
        o, R = palm_frame({f: hp[f][0] for f in FINGERS}, hp["palm"])
        s = r_len / float(np.linalg.norm(R.T @ (hp["middle"][-1] - o)))
        ang, distal = [], []
        for f in FINGERS:
            b = bones[f][-len(hp[f]):]
            for k in range(len(b) - 1):
                u = R.T @ (hp[f][k + 1] - hp[f][k])
                v = r_R.T @ (b[k + 1] - b[k])
                nu, nv = np.linalg.norm(u), np.linalg.norm(v)
                if nu > 1e-9 and nv > 1e-9:
                    a = np.degrees(np.arccos(np.clip(u @ v / (nu * nv), -1, 1)))
                    ang.append(a)
                    if k == len(b) - 2:
                        distal.append(a)
        contact = {}
        for f in FINGERS:
            if f == "thumb":
                continue
            h = float(np.linalg.norm(hp["thumb"][-1] - hp[f][-1])) * s
            contact[f] = (float(np.linalg.norm(tips["thumb"] - tips[f])) - h) * 1e3
        spread = [(float(np.linalg.norm(tips[a] - tips[b]))
                   - float(np.linalg.norm(hp[a][-1] - hp[b][-1])) * s) * 1e3
                  for a, b in SPREAD_PAIRS]
        return (float(np.mean(ang)), contact, float(np.mean(np.abs(spread))),
                float(np.mean(distal)) if distal else float("nan"))


def measure(engine, tips_of, bones_of, frame_of, side, trajs):
    fair = Fair(side)
    r_o, r_R, r_len = frame_of()
    acc = {k: [] for k in ("shape", "distal", "spread", "dq", "ms", "pinch", "open",
                           "gmc", "lmc")}
    for f, traj in trajs.items():
        engine.reset()
        prev = prev_h = prev_r = None
        for i, ang in enumerate(traj):
            t0 = time.perf_counter()
            q = engine.compute(ang)
            acc["ms"].append((time.perf_counter() - t0) * 1e3)
            if prev is not None:
                acc["dq"].append(float(np.abs(q - prev).max()))
            prev = q
            tips = tips_of()
            bones = bones_of()
            sh, con, spr, dis = fair.score(ang, tips, bones, r_R, r_len)
            acc["shape"].append(sh)
            acc["distal"].append(dis)
            acc["spread"].append(spr)
            h_chain = fair.chain_local(ang)
            r_chain = {g: np.array([r_R.T @ (p - r_o) for p in bones[g]])
                       for g in FINGERS}
            cur_h = {g: h_chain[g][-1] for g in FINGERS}
            cur_r = {g: r_chain[g][-1] for g in FINGERS}
            if prev_h is not None:
                g_, l_ = motion_consistency(prev_h, cur_h, prev_r, cur_r,
                                            bone_axes(h_chain),
                                            bone_axes(r_chain))
                acc["gmc"] += g_
                acc["lmc"] += l_
            prev_h, prev_r = cur_h, cur_r
            if f in FINGERS and i == len(traj) - 1:
                acc["pinch"].append(abs(con[f]))
            if i == 0:
                acc["open"].append(np.mean([abs(v) for v in con.values()]))
    gmc = np.asarray(acc["gmc"])
    lmc = np.asarray(acc["lmc"])
    pinch = np.mean(acc["pinch"]) if acc["pinch"] else float("nan")
    return (np.mean(acc["shape"]), np.mean(acc["distal"]),
            np.mean(acc["spread"]), pinch,
            np.mean(acc["open"]), 100.0 * gmc.mean(), 100.0 * lmc.mean(),
            100.0 * (lmc.mean() - gmc.mean()),
            np.percentile(acc["dq"], 95), np.mean(acc["ms"]))


def kp_probe(kp):
    import pinocchio as pin

    def tips():
        kp._fk_robot(kp._q)
        return {f: kp._pos(kp._fids[f][-1]) for f in FINGERS}

    def bones():
        kp._fk_robot(kp._q)
        return {f: np.array([kp._pos(i) for i in kp._fids[f]]) for f in FINGERS}

    def frame():
        kp._fk_robot(pin.neutral(kp.model))
        pts = {f: np.array([kp._pos(i) for i in kp._fids[f]]) for f in FINGERS}
        o, R = palm_frame_from_fingers(pts)
        return o, R, float(np.linalg.norm(R.T @ (pts["middle"][-1] - o)))
    return tips, bones, frame


def net_probe(r):
    import pinocchio as pin

    def pos(fid):
        return r.data.oMf[fid].translation.copy()

    def fk():
        pin.forwardKinematics(r.model, r.data, r._q)
        pin.updateFramePlacements(r.model, r.data)

    def tips():
        fk()
        return {f: pos(r.kv.fids[f][-1]) for f in FINGERS}

    def bones():
        fk()
        return {f: np.array([pos(i) for i in r.kv.fids[f]]) for f in FINGERS}

    def frame():
        pin.forwardKinematics(r.model, r.data, pin.neutral(r.model))
        pin.updateFramePlacements(r.model, r.data)
        pts = {f: np.array([pos(i) for i in r.kv.fids[f]]) for f in FINGERS}
        o, R = palm_frame_from_fingers(pts)
        return o, R, float(np.linalg.norm(R.T @ (pts["middle"][-1] - o)))
    return tips, bones, frame


def dex_probe(cfg, side):
    dx = HandRetargeter(side, cfg)
    rob = dx._seq_stage1.optimizer.robot
    chain = {f: fc.links for f, fc in
             zip(FINGERS, CONFIG_REGISTRY[cfg]()._get_fingers(side))}

    def lp(n):
        return rob.get_link_pose(rob.get_link_index(n))[:3, 3].copy()

    def frame():
        rob.compute_forward_kinematics(rob.q0)
        pts = {f: np.array([lp(n) for n in chain[f][1:]]) for f in FINGERS}
        o, R = palm_frame_from_fingers(pts)
        return o, R, float(np.linalg.norm(R.T @ (pts["middle"][-1] - o)))

    class Wrap:
        def compute(self, ang):
            q = dx.compute(ang)
            rob.compute_forward_kinematics(
                dx._two_stage_retarget(dx.last_human_positions + dx._r_origin))
            return q

        def reset(self):
            pass
    return (Wrap(), lambda: {f: lp(chain[f][-1]) for f in FINGERS},
            lambda: {f: np.array([lp(n) for n in chain[f][1:]]) for f in FINGERS},
            frame)


def main():
    torch.set_num_threads(THREADS)
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--dump", required=True, help="글러브 캘리브 덤프 json")
    ap.add_argument("--profiles", required=True, help="profiles/ 디렉토리")
    ap.add_argument("--configs", nargs="+", default=["orca_hand", "robotis_hx5_d20"])
    ap.add_argument("--backends", nargs="+", default=["kp", "dex"],
                    choices=["kp", "dex", "net"])
    ap.add_argument("--net-checkpoint", default=None)
    ap.add_argument("--steps", type=int, default=20)
    ap.add_argument("--traj", nargs="+", default=["pinch"],
                    choices=["pinch", "flex", "abd"],
                    help="pinch=실측 핀치 램프 / flex=굽힘 전용 / abd=벌림 전용")
    args = ap.parse_args()

    side, trajs = load_poses(args.dump, args.profiles, args.steps, tuple(args.traj))
    print("side=%s  궤적 %s (%d개) x %d프레임" % (
        side, ",".join(args.traj), len(trajs), args.steps))
    print("%-26s %7s %7s %7s %9s %9s %6s %6s %6s %7s %6s" % (
        "설정", "형상°", "말단°", "벌림mm", "핀치접촉", "펴짐접촉", "GMC", "LMC",
        "격차", "|dq|p95", "ms"))
    for cfg in args.configs:
        for be in args.backends:
            if be == "kp":
                kp = KPHandRetargeter(side, cfg)
                r = measure(kp, *kp_probe(kp), side, trajs)
            elif be == "net":
                if args.net_checkpoint is None:
                    ap.error("--backends net 은 --net-checkpoint 가 필요하다")
                eng = NetHandRetargeter(side, cfg, checkpoint=args.net_checkpoint)
                r = measure(eng, *net_probe(eng), side, trajs)
            else:
                eng, t, b, fr = dex_probe(cfg, side)
                r = measure(eng, t, b, fr, side, trajs)
            print("%-26s %7.1f %7.1f %7.1f %9.1f %9.1f %6.1f %6.1f %+6.1f %7.3f"
                  " %6.2f" % ("%s %s" % (cfg, be), *r))


if __name__ == "__main__":
    main()
