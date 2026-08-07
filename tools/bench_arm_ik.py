#!/usr/bin/env python3
"""팔 IK 벤치 — 정확도 / 연속성 / 비용을 한 번에 재는 고정 기준.

IK 를 손볼 때마다 임시 스크립트로 재면 비교가 안 된다. 이 도구가 기준선이다.

    python tools/bench_arm_ik.py --rig rigs/nero_orca_right.yaml
    python tools/bench_arm_ik.py --rig … --set solver.backend=dls
    python tools/bench_arm_ik.py --rig … --traj reach          # 궤적 종류
    python tools/bench_arm_ik.py --rig … --episode ~/data/ep0  # 기록 리플레이(있으면)

지표
----
정확도  : 클램프 후 베이스 목표 대비 pos[mm] / ori[deg] (mean·p95·max)
연속성  : 프레임 간 |Δq|[rad] (mean·p95·max) + 임계 초과 횟수 → EE 순간이동 지표
비용    : 프레임당 solve 시간[ms] (mean·p95·max) + 60Hz 예산(16.7ms) 초과 프레임 수

**도달 불가 구간 주의** — 추종 오차가 크다고 솔버 탓이 아닐 수 있다. `--floor` 를
주면 최악 프레임에서 무차별 재시작으로 '도달 가능 하한'을 구해 판정까지 낸다.
"""
from __future__ import annotations

import argparse
import time

import numpy as np
from scipy.spatial.transform import Rotation

BUDGET_MS = 1000.0 / 60.0
JUMP_TOL = 0.6


def traj_wave(n: int):
    for i in range(n):
        t = i / n
        T = np.eye(4)
        T[:3, 3] = [0.55 + 0.15 * np.sin(2 * np.pi * t),
                    0.25 * np.sin(4 * np.pi * t), 0.15]
        T[:3, :3] = Rotation.from_euler(
            "xyz", [1.2 * np.sin(2 * np.pi * t), 0.8 * np.sin(3 * np.pi * t),
                    1.5 * np.sin(1.5 * np.pi * t)]).as_matrix()
        yield T


def traj_reach(n: int):
    for i, T in enumerate(traj_wave(n)):
        T = T.copy()
        T[:3, 3] *= 1.0 + 1.2 * max(0.0, (i / n) - 0.4)
        yield T


def traj_slow(n: int):
    T0 = np.eye(4)
    T0[:3, :3] = Rotation.from_euler("xyz", [0.0, -np.pi / 2, 0.0]).as_matrix()
    for i in range(n):
        t = i / n
        T = T0.copy()
        T[:3, 3] = [0.5 + 0.1 * np.sin(2 * np.pi * t), 0.1 * np.sin(2 * np.pi * t), 0.2]
        yield T


def traj_fk(n: int, robot=None):
    s = robot.solver
    lo, hi = s._lo, s._hi
    mid = 0.5 * (lo + hi)
    amp = 0.25 * (hi - lo)
    phase = np.linspace(0.0, 2.0, s.nq)
    for i in range(n):
        t = i / n
        q = mid + amp * np.sin(2 * np.pi * (t + phase))
        yield robot.to_canonical(s.fk(q))


def traj_overshoot(n: int, robot=None):
    s = robot.solver
    rm = robot.rig.solver.reach_max or 1.0
    lo, hi = s._lo, s._hi
    qa = lo + (hi - lo) * 0.35
    qb = lo + (hi - lo) * 0.65
    Ta, Tb = robot.to_canonical(s.fk(qa)), robot.to_canonical(s.fk(qb))
    for i in range(n):
        t = i / (n - 1)
        u = 2.0 * t if t <= 0.5 else 2.0 * (1.0 - t)
        T = Ta.copy()
        T[:3, 3] = (1 - u) * Ta[:3, 3] + u * Tb[:3, 3]
        T[:3, :3] = Tb[:3, :3] if u > 0.5 else Ta[:3, :3]
        push = max(0.0, 1.0 - abs(u - 0.5) / 0.2)
        if push > 0.0:
            d = T[:3, 3]
            nrm = float(np.linalg.norm(d)) or 1e-9
            T[:3, 3] = d * (1.0 + push * (1.6 * rm / nrm - 1.0))
        yield T


TRAJ = {"wave": traj_wave, "reach": traj_reach, "slow": traj_slow, "fk": traj_fk,
        "overshoot": traj_overshoot}
NEEDS_ROBOT = {"fk", "overshoot"}


def _clamped_base_target(robot, T_canonical):
    T_b = robot.to_base(np.asarray(T_canonical, dtype=float))
    rm = robot.rig.solver.reach_max
    if rm:
        n = float(np.linalg.norm(T_b[:3, 3]))
        if n > rm:
            T_b = T_b.copy()
            T_b[:3, 3] *= rm / n
    return T_b


def run(robot, ik, targets):
    qs, ts, pes, oes = [], [], [], []
    for T in targets:
        t0 = time.perf_counter()
        q = np.asarray(ik.solve(T), dtype=float)
        ts.append((time.perf_counter() - t0) * 1e3)
        qs.append(q)
        pe, oe = robot.solver.pose_error(q, _clamped_base_target(robot, T))
        pes.append(pe)
        oes.append(oe)
    return np.array(qs), np.array(ts), np.array(pes), np.array(oes)


def report(label, qs, ts, pes, oes):
    dq = np.linalg.norm(np.diff(qs, axis=0), axis=1)
    def s(a, f=1.0):
        return f"mean {a.mean()*f:7.2f}  p95 {np.percentile(a,95)*f:7.2f}  max {a.max()*f:7.2f}"
    print(f"\n=== {label} ({len(qs)} 프레임) ===")
    print(f"정확도 pos[mm] : {s(pes, 1000)}")
    print(f"      ori[deg] : {s(np.degrees(oes))}")
    print(f"연속성 |Δq|[rad]: {s(dq)}   >{JUMP_TOL} 초과 {int((dq>JUMP_TOL).sum())}회")
    print(f"비용   [ms]     : {s(ts)}   >{BUDGET_MS:.1f}ms 초과 {int((ts>BUDGET_MS).sum())}/{len(ts)}")
    return dict(pos=pes.mean(), jump=dq.max(), over=int((dq > JUMP_TOL).sum()),
                ms=ts.mean(), ms_max=ts.max())


def floor_check(robot, targets, pes, restarts=200, k=5):
    s = robot.solver
    lo = np.where(np.isfinite(s.model.lowerPositionLimit), s.model.lowerPositionLimit, -np.pi)
    hi = np.where(np.isfinite(s.model.upperPositionLimit), s.model.upperPositionLimit, np.pi)
    rng = np.random.default_rng(0)
    worst = np.argsort(pes)[-k:][::-1]
    print(f"\n--- 도달 가능 하한 (무차별 {restarts}회) ---")
    print(f"{'frame':>6} {'측정':>10} {'하한':>10}   판정")
    for i in worst:
        T_b = _clamped_base_target(robot, targets[int(i)])
        best = np.inf
        for _ in range(restarts):
            q0 = lo + (hi - lo) * rng.random(s.nq)
            s.init_data = q0
            s.history_data = q0
            try:
                q = s.converge(T_b, q0)
            except Exception:
                continue
            p, o = s.pose_error(q, T_b)
            best = min(best, p + 0.1 * o)
        verdict = "솔버 실패" if best < pes[i] - 0.005 else "도달 불가(하한)"
        print(f"{int(i):>6} {pes[i]*1000:>9.1f}mm {best*1000:>9.1f}mm   {verdict}")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--rig", default="rigs/nero_orca_right.yaml")
    ap.add_argument("--traj", default="wave", choices=sorted(TRAJ))
    ap.add_argument("--frames", type=int, default=300)
    ap.add_argument("--set", action="append", default=[],
                    help="rig 값 덮어쓰기 (예: solver.backend=dls, solver.w_ori=5)")
    ap.add_argument("--floor", action="store_true", help="최악 프레임 도달 하한까지 판정")
    args = ap.parse_args()

    from whatslab.teleop.ik import RobotArmIK
    from whatslab.robot import RobotModel, load_rig

    rig = load_rig(args.rig)
    for kv in args.set:
        path, _, val = kv.partition("=")
        obj = rig
        parts = path.split(".")
        for p in parts[:-1]:
            obj = getattr(obj, p)
        if not hasattr(obj, parts[-1]):
            raise SystemExit(f"--set: 없는 필드 {path!r}")
        for cast in (int, float):
            try:
                setattr(obj, parts[-1], cast(val))
                break
            except ValueError:
                continue
        else:
            setattr(obj, parts[-1], val)
    robot = RobotModel(rig)
    ik = RobotArmIK(robot)

    targets = list(TRAJ[args.traj](args.frames, robot) if args.traj in NEEDS_ROBOT
                   else TRAJ[args.traj](args.frames))
    label = f"{rig.name} / {args.traj} / backend={rig.solver.backend}" \
            + (f" / {' '.join(args.set)}" if args.set else "")
    qs, ts, pes, oes = run(robot, ik, targets)
    report(label, qs, ts, pes, oes)
    if args.floor:
        floor_check(robot, targets, pes)


if __name__ == "__main__":
    main()
