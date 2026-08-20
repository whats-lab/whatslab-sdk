#!/usr/bin/env python3
import argparse
import math
import time

import numpy as np

from whatslab.solvers import UniRetargeter

GESTURES = (
    ("curl", ("_mcp_flex", "_pip", "_dip", "_ip_flex"), 1.2),
    ("spread", ("_mcp_abd",), 0.35),
    ("thumb", ("thumb_cmc_flex", "thumb_cmc_abd", "thumb_mcp_flex"), 1.0),
)


def gesture_frame(names, phase):
    label, keys, amp = GESTURES[int(phase) % len(GESTURES)]
    ramp = amp * 0.5 * (1.0 - math.cos(2.0 * math.pi * (phase % 1.0)))
    return label, {n: (ramp if any(k in n for k in keys) else 0.0) for n in names}


def bench(rt, iters, warm, threads):
    rng = np.random.default_rng(0)
    frames = [{n: float(v) for n, v in zip(rt.human_joint_names, row)}
              for row in rng.uniform(-0.3, 0.9,
                                     (warm + iters, len(rt.human_joint_names)))]
    for f in frames[:warm]:
        rt.compute(f)
    t0 = time.perf_counter()
    for f in frames[warm:]:
        rt.compute(f)
    ms = (time.perf_counter() - t0) / iters * 1e3
    print(f"[bench] 프레임당 {ms:.3f} ms -> {1000.0 / ms:.0f} Hz (스레드 {threads})")
    print("[bench] 마지막 q(rad):",
          np.array2string(rt.compute(frames[-1]), precision=3))


def run_viz(rt, side, port, rate, gap, period_s):
    from whatslab.solvers.hand.human_fk import HumanHandFK
    from whatslab.viz import HandViz, human_upright_root

    fk = HumanHandFK(side)
    human = HandViz(fk.urdf_path, fk.joint_names, port=port,
                    root_path="/human_hand",
                    root_pose=human_upright_root(fk, offset=(0.0, -gap, 0.0)))
    human.start()
    robot = HandViz(rt.urdf_path, rt.joint_names, port=port,
                    root_path="/robot_hand")
    robot.start()
    print("[viz] 왼쪽 사람 손, 오른쪽 로봇 손 (메쉬 %s / %s)"
          % ("O" if human.mesh_mode else "X", "O" if robot.mesh_mode else "X"))
    print("[viz] 동작 " + " -> ".join(g[0] for g in GESTURES) + " 반복. Ctrl-C 종료.")

    dt, t0, last = 1.0 / rate, time.monotonic(), ""
    try:
        while True:
            phase = (time.monotonic() - t0) / period_s
            label, angles = gesture_frame(rt.human_joint_names, phase)
            q = rt.compute(angles)
            human.update(fk.q_from_named(angles))
            robot.update(q)
            if label != last:
                last = label
                print(f"[viz] {label}", flush=True)
            time.sleep(dt)
    except KeyboardInterrupt:
        print("\n[stop] 종료")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--robot", default="orca_hand",
                    help="orca_hand|robotis_hx5_d20|tesollo_dg5f|allegro_hand|base_hand")
    ap.add_argument("--hand", default="left", choices=("left", "right"))
    ap.add_argument("--onnx", default=None, help="기본은 assets/uni_all.onnx")
    ap.add_argument("--tables", default=None, help="기본은 assets/uni_tables.npz")
    ap.add_argument("--threads", type=int, default=1)
    ap.add_argument("--viz", action="store_true",
                    help="viser 로 사람/로봇 손을 나란히 띄우고 합성 동작을 흘린다")
    ap.add_argument("--port", type=int, default=8080, help="viser 포트")
    ap.add_argument("--viz-gap", type=float, default=0.25,
                    help="viz 에서 두 손을 띄워놓을 간격 (m)")
    ap.add_argument("--rate", type=float, default=30.0, help="viz 갱신 주기 (Hz)")
    ap.add_argument("--gesture-period", type=float, default=3.0,
                    help="동작 하나가 왕복하는 시간 (s)")
    ap.add_argument("--iters", type=int, default=500, help="bench 측정 프레임 수")
    ap.add_argument("--warm", type=int, default=50, help="bench 워밍업 프레임 수")
    args = ap.parse_args()

    rt = UniRetargeter(args.hand, args.robot, onnx_path=args.onnx,
                       tables_path=args.tables, threads=args.threads)
    print(f"[setup] {args.robot} [{args.hand}] "
          f"사람 관절 {len(rt.human_joint_names)}개 -> 로봇 관절 {len(rt.joint_names)}개")
    print(f"[setup] urdf={rt.urdf_path}")

    if args.viz:
        run_viz(rt, args.hand, args.port, args.rate, args.viz_gap,
                args.gesture_period)
    else:
        bench(rt, args.iters, args.warm, args.threads)


if __name__ == "__main__":
    main()
