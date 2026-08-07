#!/usr/bin/env python3
import argparse
import threading
import time

import numpy as np

from whatslab.teleop import GloveModel, QuestModel
from whatslab.robot import RobotModel


def _build_model(args, robot):
    if args.arm == "wrist":
        return QuestModel(robot)
    return GloveModel(robot)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--rig", default="rigs/nero_orca_right.yaml", help="rig config 경로")
    ap.add_argument("--side", default="right", choices=["left", "right"])
    ap.add_argument("--arm", default="controller", choices=["controller", "wrist"],
                    help="팔 소스: controller=Quest 컨트롤러(+글러브 손), wrist=Quest 핸드트래킹")
    ap.add_argument("--hand-config", default="orca_hand", help="손 리타게팅 config (hand 포함 rig)")
    ap.add_argument("--rate", type=float, default=60.0, help="폴링/출력 주기 (Hz)")
    ap.add_argument("--viz", action="store_true", help="viser: 팔+손 메쉬 + 목표(/target)·EE(/ee) 프레임")
    ap.add_argument("--port", type=int, default=8080, help="viser 포트")
    ap.add_argument("--no-safety", action="store_true",
                    help="관절 속도 rate-limit(SafetyFilter) 미적용 — 진단용")
    ap.add_argument("--diag", action="store_true",
                    help="단계별 계측 출력 — 입력/캘리브/클램프/IK오차/불연속을 한 줄로")
    ap.add_argument("--robot", action="store_true",
                    help="실물 전송 패널을 viser 에 띄운다 (--viz 필요, 기본 미연결)")
    ap.add_argument("--can", default="can0", help="nero CAN 채널")
    ap.add_argument("--speed", type=int, default=20, help="nero 속도 퍼센트 (텔레옵은 낮게)")
    args = ap.parse_args()

    robot = RobotModel(args.rig)
    model = _build_model(args, robot)

    print(f"[setup] arm_joints={robot.arm_joint_names}")

    if not args.no_safety:
        from robot_io import attach_safety
        sf = attach_safety(model, robot, args.rate)

    model.start()

    viz = None
    bridge = None
    if args.viz:
        from whatslab.viz import RobotArmViz
        viz = RobotArmViz(robot, port=args.port)
        viz.start()
        if args.robot:
            from robot_io import build_robot_panel
            bridge = build_robot_panel(model, robot, args)
        print(f"[viz] viser: http://localhost:{args.port}")
    elif args.robot:
        print("[robot] --robot 은 --viz 와 함께 써야 합니다(연결 버튼이 viser 패널에 있음)")

    def _calib_loop():
        while True:
            try:
                line = input()
            except (EOFError, KeyboardInterrupt):
                return
            if line.strip().lower() == "r":
                print("[calib] reach 측정 시작 — 팔을 최대 범위로 뻗으세요(8초)...", flush=True)
                r = model.calibrate_reach(persist=True)
            else:
                ok = model.calibrate_yaw()
                print("[calib] yaw " + ("완료(머리연동)" if ok else "실패 — HMD/자세 신호 확인"),
                      flush=True)
    threading.Thread(target=_calib_loop, daemon=True, name="calib").start()
    print("[calib] 기준 자세로 Enter → yaw 캘리브 | 'r'+Enter → reach 캘리브. Ctrl-C 종료.")

    period, last = 1.0 / args.rate, 0.0
    arm_names = list(robot.arm_joint_names)
    arm_set = set(arm_names)
    diag = None
    if args.diag:
        from robot_io import Diag
        diag = Diag(robot, model, args.side)
    try:
        while True:
            now = time.monotonic()
            q = model.get_q()
            right_q = q.get(args.side) or {}
            if diag is not None:
                raw = model._get_raw_target().get(args.side)
                q_arm_v = (np.array([right_q[n] for n in arm_names], dtype=float)
                           if all(n in right_q for n in arm_names) else None)
                diag.tick(raw, q_arm_v, now)
            if bridge is not None:
                bridge.send(right_q)
            has_arm_q = all(n in right_q for n in arm_names)
            if viz is not None and has_arm_q:
                arm_q = [right_q[n] for n in arm_names]
                hand_names = [k for k in right_q if k not in arm_set]
                hand_q = [right_q[n] for n in hand_names]
                viz.update(arm_q, target_pose=model.target.get(args.side),
                           hand_q=hand_q, hand_names=hand_names, timestamp=now)
            if now - last > 0.2:
                last = now
                arm_q = [round(right_q[n], 3) for n in arm_names] if has_arm_q else "--"
                tgt = "on" if model.target.get(args.side) is not None else "--"
                print(f"\r[q] arm={arm_q} hand={len(right_q) - len(arm_names)}j target={tgt}   ",
                      end="", flush=True)
            time.sleep(period)
    except KeyboardInterrupt:
        print("\n[stop] 종료")
    finally:
        model.stop()


if __name__ == "__main__":
    main()
