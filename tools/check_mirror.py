#!/usr/bin/env python3
"""좌우 URDF 가 참 미러인지 잰다 — 좌우 통합(mirror_to) 을 쓸지의 판정 기준.

판정은 전범위 무작위 자세의 미러 오차로 한다. 관절을 하나씩 조금 흔드는 축 검사는
통과해도 전범위에서 틀어지는 사례가 있었다(실측 67~102mm). 실제 결과와도 전범위
쪽이 맞는다 — orca 는 0.9mm 라 오른손 지문추종이 전용모델 24.02 대 미러 24.49mm 로
차이가 2% 뿐이고, robotis 는 14mm 라 30.50 대 49.65mm 로 63% 악화된다.
"""
import argparse

import numpy as np
import pinocchio as pin

from whatslab.solvers.hand.hand_configs import CONFIG_REGISTRY
from whatslab.solvers.hand.human_fk import FINGERS, HumanHandFK, palm_frame_from_fingers
from whatslab.solvers.hand.keyvector import human_chains, sensor_chains

MIRROR = np.diag([1.0, 1.0, -1.0])
PROBE_DEG = (5.0, 15.0, 25.0)
TOL_MM = 2.0
FULL_TOL_MM = 3.0
FULL_N = 500
LIMIT_TOL_DEG = 0.5


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

    def limits(self):
        lo = np.array([self.model.lowerPositionLimit[self.idx_q[n]]
                       for n in self.names])
        hi = np.array([self.model.upperPositionLimit[self.idx_q[n]]
                       for n in self.names])
        return lo, hi

    def at_unit(self, u):
        lo, hi = self.limits()
        q = pin.neutral(self.model)
        for k, n in enumerate(self.names):
            q[self.idx_q[n]] = lo[k] + (u[k] + 1.0) * 0.5 * (hi[k] - lo[k])
        return self.local(q)

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


def limit_conventions(left, right):
    llo, lhi = left.limits()
    rlo, rhi = right.limits()
    same = np.abs(rlo - llo) + np.abs(rhi - lhi)
    flip = np.abs(rlo + lhi) + np.abs(rhi + llo)
    tol = np.deg2rad(LIMIT_TOL_DEG)
    kind = np.where(same <= tol, 0, np.where(flip <= tol, 1, 2))
    return kind, [left.names[i] for i in np.flatnonzero(kind == 2)]


def full_range(left, right, seed=0):
    rng = np.random.default_rng(seed)
    u = rng.uniform(-1.0, 1.0, (FULL_N, len(left.names)))
    e = []
    for k in range(FULL_N):
        lo = left.at_unit(u[k])
        ro = right.at_unit(u[k])
        e.append(np.mean([np.abs(lo[f][-2:] - ro[f][-2:] @ MIRROR).mean()
                          for f in FINGERS]))
    return np.array(e) * 1e3


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
    kind, odd = limit_conventions(left, right)
    e = full_range(left, right)
    print("%-16s 중립형상 %6.2fmm   축흔들기>%.0fmm %2d/%-2d   전범위 mean %6.2f"
          " p95 %7.2f mm   한계규약 동일 %d / 반전 %d / 불일치 %d"
          % (label, neutral, TOL_MM, len(bad), len(rows), e.mean(),
             np.percentile(e, 95), int((kind == 0).sum()), int((kind == 1).sum()),
             int((kind == 2).sum())))
    for w, a in bad[:4]:
        print("      축흔들기 %-40s %7.2fmm" % (a[-40:], w))
    for a in odd[:4]:
        print("      한계규약 불일치 %-40s" % a[-40:])
    return e.mean() <= FULL_TOL_MM and not odd


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
        if cfg == "human":
            print("%-16s 참고 — mirror_to 를 쓰는 모든 손이 공통으로 지는 부담이다"
                  " (사람 좌우 URDF 자체의 미러 오차)" % cfg)
            continue
        print("%-16s 좌우 통합 게이트 %s — %s"
              % (cfg, "통과" if passed else "미통과",
                 "왼손 모델 하나 + mirror_to 로 양손" if passed
                 else "side 별 체크포인트 필수"))
    return 0 if all(v for k, v in ok.items() if k != "human") else 1


if __name__ == "__main__":
    raise SystemExit(main())
