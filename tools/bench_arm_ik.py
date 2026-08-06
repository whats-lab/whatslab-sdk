#!/usr/bin/env python3
"""팔 IK 벤치 — 정확도 / 연속성 / 비용을 한 번에 재는 고정 기준.

IK 를 손볼 때마다 임시 스크립트로 재면 비교가 안 된다. 이 도구가 기준선이다.

    python tools/bench_arm_ik.py --rig rigs/nero_orca_right.yaml
    python tools/bench_arm_ik.py --rig … --set solver.backend=dls
    python tools/bench_arm_ik.py --rig … --traj reach          # 궤적 종류
    python tools/bench_arm_ik.py --rig … --traj walk --seeds 40  # 다중 시드 판정
    python tools/bench_arm_ik.py --rig … --episode ~/data/ep0  # 기록 리플레이(있으면)

지표
----
정확도  : 클램프 후 베이스 목표 대비 pos[mm] / ori[deg] (mean·p95·max)
연속성  : 프레임 간 |Δq|[rad] (mean·p95·max) + 임계 초과 횟수 → EE 순간이동 지표
비용    : 프레임당 solve 시간[ms] (mean·p95·max) + 60Hz 예산(16.7ms) 초과 프레임 수

**단일 시도로 판정하지 말 것** — `walk` 궤적은 시드별 평균 오차가 3~150mm 로
흩어진다. `--traj walk --seeds 40` 의 평균±표준오차로만 비교하고, 두 설정을
비교할 땐 `--dump` 로 시드별 값을 받아 대응차(paired)를 본다.

**도달 불가 구간 주의** — 추종 오차가 크다고 솔버 탓이 아닐 수 있다. `--floor` 를
주면 최악 프레임에서 무차별 재시작으로 '도달 가능 하한'을 구해 판정까지 낸다.
"""
from __future__ import annotations

import argparse
import time

import numpy as np
from scipy.spatial.transform import Rotation

BUDGET_MS = 1000.0 / 60.0        # 60Hz 프레임 예산
JUMP_TOL = 0.6                   # [rad] 이 이상 튀면 순간이동으로 본다


# ------------------------------------------------------------------ 궤적
def traj_wave(n: int):
    """도달범위 안에서 매끄럽게 움직이되 **자세는 임의로** 굴린다(컨트롤러 유사)."""
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
    """reach_max 밖까지 뻗어 클램프가 지속되는 구간(사람이 팔을 뻗는 상황)."""
    for i, T in enumerate(traj_wave(n)):
        T = T.copy()
        T[:3, 3] *= 1.0 + 1.2 * max(0.0, (i / n) - 0.4)
        yield T


def traj_slow(n: int):
    """느린 위치 이동 + 고정 자세 — 솔버의 정상상태 정확도만 본다."""
    T0 = np.eye(4)
    T0[:3, :3] = Rotation.from_euler("xyz", [0.0, -np.pi / 2, 0.0]).as_matrix()
    for i in range(n):
        t = i / n
        T = T0.copy()
        T[:3, 3] = [0.5 + 0.1 * np.sin(2 * np.pi * t), 0.1 * np.sin(2 * np.pi * t), 0.2]
        yield T


def traj_fk(n: int, robot=None):
    """**도달 가능이 보장된** 궤적 — 유효 q 를 매끄럽게 흔들고 FK 로 목표를 만든다.

    오차 하한이 정확히 0 이므로, 남는 오차는 전부 솔버 탓이다. 합성 좌표로 만든
    궤적(wave/slow)은 도달 불가 구간을 지나 솔버 품질과 도달성을 섞어버린다 —
    **정확도 판정은 이 궤적으로 한다.**
    """
    s = robot.solver
    lo, hi = s._lo, s._hi
    mid = 0.5 * (lo + hi)
    amp = 0.25 * (hi - lo)
    phase = np.linspace(0.0, 2.0, s.nq)      # 관절마다 위상 달리 → 자세도 함께 변함
    for i in range(n):
        t = i / n
        q = mid + amp * np.sin(2 * np.pi * (t + phase))
        yield robot.to_canonical(s.fk(q))


def traj_overshoot(n: int, robot=None):
    """A → (도달 한계 밖) → B → 되돌아오기. 클램프 진입/이탈에서의 연속성 검사.

    사람이 팔을 너무 멀리 뻗었다 당기는 상황. 목표가 워크스페이스를 벗어나는 구간에서는
    reach 클램프가 방향만 남기므로, 진입·이탈 순간에 해가 튈 수 있다(불연속 취약점).
    A/B 는 도달 가능하도록 FK 로 잡고, 중간을 한계 밖으로 밀어낸다.
    """
    s = robot.solver
    rm = robot.rig.solver.reach_max or 1.0
    lo, hi = s._lo, s._hi
    qa = lo + (hi - lo) * 0.35
    qb = lo + (hi - lo) * 0.65
    Ta, Tb = robot.to_canonical(s.fk(qa)), robot.to_canonical(s.fk(qb))
    for i in range(n):
        t = i / (n - 1)
        u = 2.0 * t if t <= 0.5 else 2.0 * (1.0 - t)      # A→B→A 왕복
        T = Ta.copy()
        T[:3, 3] = (1 - u) * Ta[:3, 3] + u * Tb[:3, 3]
        T[:3, :3] = Tb[:3, :3] if u > 0.5 else Ta[:3, :3]
        # 중간 구간(u 0.3~0.7)에서 한계의 1.6배까지 밀어낸다 → 클램프 진입/이탈
        push = max(0.0, 1.0 - abs(u - 0.5) / 0.2)         # 0→1→0
        if push > 0.0:
            d = T[:3, 3]
            nrm = float(np.linalg.norm(d)) or 1e-9
            T[:3, 3] = d * (1.0 + push * (1.6 * rm / nrm - 1.0))
        yield T


def traj_walk(n: int, robot=None, seed: int = 0):
    """사람 손목 유사 랜덤워크 — 위치·자세를 독립적으로 흔든다.

    시드마다 완전히 다른 궤적이 나오고 평균 오차가 3~150mm 로 흩어진다.
    **단일 시드로는 판정 불가** → `--seeds 40` 의 평균±표준오차로 비교하고,
    두 설정 비교는 시드를 맞춘 대응차(paired)로 본다.
    """
    rm = (robot.rig.solver.reach_max or 0.5) if robot is not None else 0.5
    rng = np.random.default_rng(seed)
    p = np.array([0.35, -0.20, 0.10])
    R = Rotation.identity()
    for _ in range(n):
        p = p + 0.010 * rng.standard_normal(3)
        r = float(np.linalg.norm(p))
        p *= np.clip(r, 0.45 * rm, 0.62 * rm) / r
        R = R * Rotation.from_rotvec(0.06 * rng.standard_normal(3))
        T = np.eye(4)
        T[:3, :3] = R.as_matrix()
        T[:3, 3] = p
        yield T


TRAJ = {"wave": traj_wave, "reach": traj_reach, "slow": traj_slow, "fk": traj_fk,
        "overshoot": traj_overshoot, "walk": traj_walk}
NEEDS_ROBOT = {"fk", "overshoot", "walk"}     # robot 인자를 받는 궤적
SEEDED = {"walk"}                             # seed 인자를 받는 궤적


# ------------------------------------------------------------------ 실행
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
    return dict(pos=pes.mean(), ori=oes.mean(), jump=dq.max(),
                over=int((dq > JUMP_TOL).sum()), ms=ts.mean(), ms_max=ts.max(),
                frames=len(qs))


def report_multi(label, runs, dump=None):
    """시드별 결과를 평균±표준오차로 집계 — 단일 시드 판정을 막는 것이 목적이다."""
    pos = np.array([r["pos"] for r in runs]) * 1000.0
    ori = np.degrees(np.array([r["ori"] for r in runs]))
    jump = np.array([r["jump"] for r in runs])
    over = int(sum(r["over"] for r in runs))
    ms = np.array([r["ms"] for r in runs])
    n = len(runs)
    score = pos + 100.0 * np.radians(ori)

    def pm(a, unit):
        se = a.std(ddof=1) / np.sqrt(n) if n > 1 else 0.0
        return f"{a.mean():7.2f} ± {se:5.2f} {unit}"

    print(f"\n=== {label} ({n} 시드 × {runs[0]['frames']} 프레임) ===")
    print(f"정확도 pos      : {pm(pos, 'mm')}   (시드 최대 {pos.max():.1f})")
    print(f"      ori      : {pm(ori, 'deg')}")
    print(f"score(pos+.1ori): {pm(score, 'mm')}")
    print(f"연속성 |Δq| max  : {jump.max():7.3f} rad   >{JUMP_TOL} 초과 {over}회")
    print(f"비용   [ms]     : {pm(ms, 'ms')}")
    print(f"시드별 pos[mm]  : {np.round(pos, 1).tolist()}")
    if dump:
        import json
        json.dump({"pos": pos.tolist(), "ori": ori.tolist(), "score": score.tolist()},
                  open(dump, "w"))
        print(f"(시드별 값 저장: {dump})")


def floor_check(robot, targets, pes, restarts=200, k=5):
    """최악 프레임의 도달 가능 하한 — 솔버 실패인가 도달 불가인가."""
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
    ap.add_argument("--seeds", type=int, default=1,
                    help="시드 개수 (walk 궤적). 2 이상이면 평균±표준오차로 집계 — "
                         "IK 변경 판정은 단일 시드로 하지 말 것")
    ap.add_argument("--dump", help="시드별 값을 json 으로 저장(대응차 비교용)")
    ap.add_argument("--set", action="append", default=[],
                    help="rig 값 덮어쓰기 (예: solver.backend=dls, solver.w_ori=5)")
    ap.add_argument("--floor", action="store_true", help="최악 프레임 도달 하한까지 판정")
    args = ap.parse_args()

    from whatslab.model.ik import RobotArmIK
    from whatslab.robot import RobotModel, load_rig

    rig = load_rig(args.rig)
    for kv in args.set:                       # solver.backend=dls 같은 점표기 덮어쓰기
        path, _, val = kv.partition("=")
        obj = rig
        parts = path.split(".")
        for p in parts[:-1]:
            obj = getattr(obj, p)
        if not hasattr(obj, parts[-1]):
            raise SystemExit(f"--set: 없는 필드 {path!r}")
        # 현재 값이 None 인 필드(max_iter/tol 등)에 문자열이 그대로 들어가면 솔버가
        # 예외를 삼키고 직전 자세를 붙들어 '그럴듯한 쓰레기 결과'가 나온다 → 명시 캐스팅.
        for cast in (int, float):
            try:
                setattr(obj, parts[-1], cast(val))
                break
            except ValueError:
                continue
        else:
            setattr(obj, parts[-1], val)      # 숫자로 안 되면 문자열(backend 등)
    label = f"{rig.name} / {args.traj} / backend={rig.solver.backend}" \
            + (f" / {' '.join(args.set)}" if args.set else "")
    if args.traj not in SEEDED and args.seeds > 1:
        raise SystemExit(f"--seeds 는 {sorted(SEEDED)} 궤적만 지원")

    runs, last = [], None
    for sd in range(max(1, args.seeds)):
        robot = RobotModel(rig)                  # 시드마다 솔버 상태를 새로 시작
        ik = RobotArmIK(robot)
        if args.traj in SEEDED:
            targets = list(TRAJ[args.traj](args.frames, robot, sd))
        elif args.traj in NEEDS_ROBOT:
            targets = list(TRAJ[args.traj](args.frames, robot))
        else:
            targets = list(TRAJ[args.traj](args.frames))
        qs, ts, pes, oes = run(robot, ik, targets)
        dq = np.linalg.norm(np.diff(qs, axis=0), axis=1)
        runs.append(dict(pos=pes.mean(), ori=oes.mean(), jump=float(dq.max()),
                         over=int((dq > JUMP_TOL).sum()), ms=ts.mean(),
                         ms_max=ts.max(), frames=len(qs)))
        last = (robot, targets, pes)

    if args.seeds > 1:
        report_multi(label, runs, args.dump)
    else:
        report(label, qs, ts, pes, oes)
    if args.floor and last is not None:
        floor_check(last[0], last[1], last[2])


if __name__ == "__main__":
    main()
