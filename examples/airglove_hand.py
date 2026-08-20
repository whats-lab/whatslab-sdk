#!/usr/bin/env python3
import argparse
import os
import time

import numpy as np

from whatslab.teleop import HandModel


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default="orca_hand", help="로봇 손 리타게팅 config")
    ap.add_argument("--side", default="right", choices=["left", "right"])
    ap.add_argument("--urdf-root", default=os.environ.get("WHATSLAB_MODELS_ROOT"),
                    help="models 디렉토리 (미지정 시 WHATSLAB_MODELS_ROOT / 패키지 내장)")
    ap.add_argument("--rate", type=float, default=30.0, help="처리/출력 주기 (Hz)")
    ap.add_argument("--viz", action="store_true", help="viser 사람/로봇 손 URDF 메쉬")
    ap.add_argument("--viz-gap", type=float, default=0.25,
                    help="viz 에서 두 손을 띄워놓을 간격 (m)")
    args = ap.parse_args()

    src = args.urdf_root or "(패키지 내장 URDF)"
    print(f"[setup] config={args.config} side={args.side} models={src}",
          flush=True)
    m = HandModel(hand_config=args.config, side=args.side, urdf_root=args.urdf_root)
    ctrl = m.sides[args.side].retarget

    viz_human = viz_robot = None
    if args.viz:
        from whatslab.solvers.hand.human_fk import HumanHandFK
        from whatslab.viz import HandViz, human_upright_root
        eng = ctrl.engine
        human_fk = HumanHandFK(args.side)
        viz_human = HandViz(
            human_fk.urdf_path, human_fk.joint_names, root_path="/human_hand",
            root_pose=human_upright_root(human_fk,
                                         offset=(0.0, -args.viz_gap, 0.0)))
        viz_human.start()
        viz_robot = HandViz(eng.urdf_path, eng.joint_names,
                            root_path="/robot_hand")
        viz_robot.start()
        print("[viz] 왼쪽=사람 손 URDF, 오른쪽=로봇 손 (메쉬 %s / %s)"
              % ("O" if viz_human.mesh_mode else "X",
                 "O" if viz_robot.mesh_mode else "X"), flush=True)

    m.start()
    print("[run] 글러브 OSC 수신 대기. Ctrl-C 로 종료.", flush=True)
    print(f"[run] joint_names({len(ctrl.joint_names)}): {ctrl.joint_names}",
          flush=True)

    period, last_log, q_prev, dq_max = 1.0 / args.rate, 0.0, None, 0.0
    try:
        while True:
            data = m.get_data()[args.side]
            q = m.get_q()[args.side]
            have = all(n in q for n in ctrl.joint_names)
            tracked = bool(data.get("tracked")) and have
            now = time.monotonic()
            qv = np.array([q[n] for n in ctrl.joint_names]) if have else None
            if qv is not None:
                if q_prev is not None:
                    dq_max = max(dq_max, float(np.abs(qv - q_prev).max()))
                q_prev = qv
            if viz_human is not None and tracked:
                fingers = data.get("fingers")
                angles = fingers.hand.joint_angles if fingers is not None else None
                if angles:
                    viz_human.update(
                        human_fk.q_from_named(angles), timestamp=now)
                viz_robot.update(qv, timestamp=now)
            if now - last_log > 0.2:
                last_log = now
                if not tracked:
                    print("[no-signal] 글러브 /joint_angles 수신 없음", flush=True)
                elif dq_max < 1e-4:
                    print("[정지] 글러브는 붙었지만 손이 안 움직인다 —"
                          f" q = {np.round(qv, 3).tolist()}", flush=True)
                else:
                    print(f"[추적] |dq|max {dq_max:.3f}  "
                          f"q = {np.round(qv, 3).tolist()}", flush=True)
                dq_max = 0.0
            time.sleep(period)
    except KeyboardInterrupt:
        print("\n[stop] 종료")
    finally:
        m.stop()


if __name__ == "__main__":
    main()
