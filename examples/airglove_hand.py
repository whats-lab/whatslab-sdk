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
    print(f"[setup] config={args.config} side={args.side} models={src}")
    m = HandModel(hand_config=args.config, side=args.side, urdf_root=args.urdf_root)
    ctrl = m.sides[args.side].retarget

    viz_human = viz_robot = None
    if args.viz:
        from whatslab.viz import HumanHandViz, RobotHandViz
        eng = ctrl.engine
        T_human = np.eye(4)
        T_human[1, 3] = -args.viz_gap
        T_human = T_human @ eng.human_to_robot()
        viz_human = HumanHandViz(eng.fk, root_pose=T_human)
        viz_human.start()
        viz_robot = RobotHandViz(eng)
        viz_robot.start()
        print("[viz] viser: 왼쪽=사람 손 URDF, 오른쪽=로봇 손 (메쉬 %s / %s)" % (
            "O" if viz_human.mesh_mode else "X",
            "O" if viz_robot.mesh_mode else "X"))

    m.start()
    print(f"[run] AGA OSC 수신 대기. Ctrl-C 로 종료.")
    print(f"[run] joint_names({len(ctrl.joint_names)}): {ctrl.joint_names}")

    period, last_log = 1.0 / args.rate, 0.0
    try:
        while True:
            q = m.get_q()[args.side]
            have = all(n in q for n in ctrl.joint_names)
            tracked = bool(m.get_data()[args.side].get("tracked")) and have
            now = time.monotonic()
            qv = np.array([q[n] for n in ctrl.joint_names]) if have else None
            if viz_human is not None and tracked:
                fingers = m.get_data()[args.side].get("fingers")
                angles = fingers.hand.joint_angles if fingers is not None else None
                if angles:
                    viz_human.update(
                        ctrl.engine.fk.q_from_named(angles), timestamp=now)
                viz_robot.update(qv, timestamp=now)
            if now - last_log > 0.2:
                last_log = now
                if tracked:
                    print(f"[TRACKED] q = {np.round(qv, 3).tolist()}")
                else:
                    print("[no-signal] 글러브 /joint_angles 수신 없음", flush=True)
            time.sleep(period)
    except KeyboardInterrupt:
        print("\n[stop] 종료")
    finally:
        m.stop()


if __name__ == "__main__":
    main()
