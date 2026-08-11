#!/usr/bin/env python3
"""좌우 URDF 가 참 미러인지 관절별로 잰다 (좌우 통합 모델의 수용 기준)."""
import argparse

import numpy as np
import pinocchio as pin

from whatslab.solvers.hand.hand_configs import CONFIG_REGISTRY
from whatslab.solvers.hand.human_fk import FINGERS, HumanHandFK, palm_frame_from_fingers
from whatslab.solvers.hand.keyvector import human_chains, sensor_chains

MIRROR = np.diag([1.0, 1.0, -1.0])
PROBE_DEG = (5.0, 15.0, 25.0)
TOL_MM = 2.0


class Rig:

    def __init__(self, model, data, chains):
        self.model = model
        self.data = data
        self.fids = {f: [model.getFrameId(n, pin.FrameType.BODY) for n in chains[f]]
                     for f in FINGERS}
        self.names = [model.names[j] for j in range(1, model.njoints)
                      if model.joints[j].nq > 0]
        self.idx_q = {model.names[j]: int(model.joints[j].idx_q)
                      for j in range(1, model.njoints) if model.joints[j].nq > 0}
        self.frame = palm_frame_from_fingers(self.points(pin.neutral(model)))

    def points(self, q):
        pin.forwardKinematics(self.model, self.data, np.asarray(q, dtype=float))
        pin.updateFramePlacements(self.model, self.data)
        return {f: np.array([self.data.oMf[i].translation.copy()
                             for i in self.fids[f]]) for f in FINGERS}

    def local(self, q):
        pts = self.points(q)
        o, rot = self.frame
        return {f: np.array([rot.T @ (p - o) for p in pts[f]]) for f in FINGERS}

    def probe(self, name, deg):
        q = pin.neutral(self.model)
        q[self.idx_q[name]] = np.deg2rad(deg)
        return self.local(q)


def human_rig(side):
    fk = HumanHandFK(side)
    return Rig(fk.model, fk.data, human_chains(fk))


def robot_rig(side, cfg):
    config = CONFIG_REGISTRY[cfg]()
    model = pin.buildModelFromUrdf(config._get_urdf_path(side))
    return Rig(model, model.createData(), sensor_chains(model, side))


def err_mm(left, right):
    return max(float(np.abs(left[f] - right[f] @ MIRROR).max()) for f in FINGERS) * 1e3


def delta(cur, base):
    return {f: cur[f] - base[f] for f in FINGERS}


def report(label, left, right):
    lb = left.local(pin.neutral(left.model))
    rb = right.local(pin.neutral(right.model))
    neutral = err_mm(lb, rb)
    if len(left.names) != len(right.names):
        print("%-16s 관절 수 불일치 L=%d R=%d — 통합 불가"
              % (label, len(left.names), len(right.names)))
        return False
    rows = []
    for a, b in zip(left.names, right.names):
        worst = max(err_mm(delta(left.probe(a, d), lb), delta(right.probe(b, d), rb))
                    for d in PROBE_DEG)
        rows.append((worst, a))
    rows.sort(reverse=True)
    bad = [r for r in rows if r[0] > TOL_MM]
    print("%-16s 중립형상 %6.2fmm   축 미러오차>%.0fmm %2d/%-2d   최악 %7.2fmm"
          % (label, neutral, TOL_MM, len(bad), len(rows), rows[0][0]))
    for w, a in bad[:6]:
        print("      %-44s %7.2fmm" % (a[-44:], w))
    return not bad and neutral <= TOL_MM


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--configs", nargs="+",
                    default=["human", "orca_hand", "robotis_hx5_d20"])
    args = ap.parse_args()
    ok = {}
    for cfg in args.configs:
        if cfg == "human":
            ok[cfg] = report(cfg, human_rig("left"), human_rig("right"))
        else:
            ok[cfg] = report(cfg, robot_rig("left", cfg), robot_rig("right", cfg))
    print()
    for cfg, passed in ok.items():
        print("%-16s 좌우 통합 게이트 %s" % (cfg, "통과" if passed else "미통과"))
    return 0 if all(ok.values()) else 1


if __name__ == "__main__":
    raise SystemExit(main())
