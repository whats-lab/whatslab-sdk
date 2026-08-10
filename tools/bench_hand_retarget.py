#!/usr/bin/env python3
"""손 리타게팅 백엔드를 목표 구성과 무관한 물리 지표로 비교한다 (글러브 캘리브 덤프 기준)."""
import argparse
import json
import time

import numpy as np

from whatslab.solvers.hand import HandRetargeter, KPHandRetargeter
from whatslab.solvers.hand.hand_configs import CONFIG_REGISTRY
from whatslab.solvers.hand.human_fk import FINGERS, HumanHandFK, palm_frame

SPREAD_PAIRS = (("index", "middle"), ("middle", "ring"), ("ring", "pinky"))


def load_poses(dump_path, profile_dir, steps):
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
    for f, th in zip(d["pinch_finger_names"], d["pinch_thetas"]):
        th = np.asarray(th, dtype=float)
        trajs[f] = [expand(th * (k + 1) / steps) for k in range(steps)]
    return side, trajs


LMC_MIN_MM = 0.5


class Fair:

    def __init__(self, side):
        self.fk = HumanHandFK(side)
        hp0 = self.fk.neutral_points()
        self._h0 = palm_frame({f: hp0[f][0] for f in FINGERS}, hp0["palm"])

    def local_tips(self, angles):
        hp = self.fk.points(angles)
        o, R = self._h0
        return {f: R.T @ (hp[f][-1] - o) for f in FINGERS}

    def score(self, angles, tips, bones, r_R, r_len):
        hp = self.fk.points(angles)
        o, R = palm_frame({f: hp[f][0] for f in FINGERS}, hp["palm"])
        s = r_len / float(np.linalg.norm(R.T @ (hp["middle"][-1] - o)))
        ang = []
        for f in FINGERS:
            b = bones[f][-len(hp[f]):]
            for k in range(len(b) - 1):
                u = R.T @ (hp[f][k + 1] - hp[f][k])
                v = r_R.T @ (b[k + 1] - b[k])
                nu, nv = np.linalg.norm(u), np.linalg.norm(v)
                if nu > 1e-9 and nv > 1e-9:
                    ang.append(np.degrees(np.arccos(np.clip(u @ v / (nu * nv), -1, 1))))
        contact = {}
        for f in FINGERS:
            if f == "thumb":
                continue
            h = float(np.linalg.norm(hp["thumb"][-1] - hp[f][-1])) * s
            contact[f] = (float(np.linalg.norm(tips["thumb"] - tips[f])) - h) * 1e3
        spread = [(float(np.linalg.norm(tips[a] - tips[b]))
                   - float(np.linalg.norm(hp[a][-1] - hp[b][-1])) * s) * 1e3
                  for a, b in SPREAD_PAIRS]
        return float(np.mean(ang)), contact, float(np.mean(np.abs(spread)))


def measure(engine, tips_of, bones_of, frame_of, side, trajs):
    fair = Fair(side)
    r_o, r_R, r_len = frame_of()
    acc = {k: [] for k in ("shape", "spread", "dq", "ms", "pinch", "open", "lmc")}
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
            sh, con, spr = fair.score(ang, tips, bones_of(), r_R, r_len)
            acc["shape"].append(sh)
            acc["spread"].append(spr)
            cur_h = fair.local_tips(ang)
            cur_r = {g: r_R.T @ (tips[g] - r_o) for g in FINGERS}
            if prev_h is not None:
                for g in FINGERS:
                    u = cur_h[g] - prev_h[g]
                    if float(np.linalg.norm(u)) * 1e3 < LMC_MIN_MM:
                        continue
                    v = cur_r[g] - prev_r[g]
                    nv = float(np.linalg.norm(v))
                    if nv < 1e-9:
                        acc["lmc"].append(-1.0)
                        continue
                    acc["lmc"].append(float(u @ v / (np.linalg.norm(u) * nv)))
            prev_h, prev_r = cur_h, cur_r
            if i == len(traj) - 1:
                acc["pinch"].append(abs(con[f]))
            if i == 0:
                acc["open"].append(np.mean([abs(v) for v in con.values()]))
    lmc = np.asarray(acc["lmc"])
    return (np.mean(acc["shape"]), np.mean(acc["spread"]), np.mean(acc["pinch"]),
            np.mean(acc["open"]), 100.0 * np.mean(lmc > 0.0), 100.0 * lmc.mean(),
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
        o, R = palm_frame({f: kp._pos(kp._fids[f][0]) for f in FINGERS},
                          kp._pos(kp._palm_fid))
        return o, R, float(np.linalg.norm(
            R.T @ (kp._pos(kp._fids["middle"][-1]) - o)))
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
        o, R = palm_frame({f: lp(chain[f][1]) for f in FINGERS},
                          lp(chain["index"][0]))
        return o, R, float(np.linalg.norm(R.T @ (lp(chain["middle"][-1]) - o)))

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
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--dump", required=True, help="글러브 캘리브 덤프 json")
    ap.add_argument("--profiles", required=True, help="profiles/ 디렉토리")
    ap.add_argument("--configs", nargs="+", default=["orca_hand", "robotis_hx5_d20"])
    ap.add_argument("--backends", nargs="+", default=["kp", "dex"])
    ap.add_argument("--steps", type=int, default=20)
    args = ap.parse_args()

    side, trajs = load_poses(args.dump, args.profiles, args.steps)
    print("side=%s  핀치 %d개 x %d프레임" % (side, len(trajs), args.steps))
    print("%-26s %8s %8s %10s %10s %7s %6s %8s %7s" % (
        "설정", "형상°", "벌림mm", "핀치접촉", "펴짐접촉", "LMC%", "cos", "|dq|p95",
        "ms"))
    for cfg in args.configs:
        for be in args.backends:
            if be == "kp":
                kp = KPHandRetargeter(side, cfg)
                r = measure(kp, *kp_probe(kp), side, trajs)
            else:
                eng, t, b, fr = dex_probe(cfg, side)
                r = measure(eng, t, b, fr, side, trajs)
            print("%-26s %8.1f %8.1f %10.1f %10.1f %7.1f %6.1f %8.3f %7.2f" % (
                "%s %s" % (cfg, be), *r))


if __name__ == "__main__":
    main()
