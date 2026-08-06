#!/usr/bin/env python3
"""Quest 컨트롤러(팔) + AirGlove(손) → nero(+orca) 텔레옵 실행 예제 (신 Model API).

whatslab 은 파이프라인을 소유하지 않는다 — 소비자(이 스크립트)가 **텔레옵 Model** 을
하나 만들고, 콜백 없이 폴링 루프에서 `get_q(side)` 만 당겨 쓴다. 좌표계/축 정합은
리시버 내부에서 끝나 있고(출력 = X-fwd/Z-up/RH 정준), Model 의 전처리는 yaw
캘리브(자세) + reach 캘리브(위치 스케일)뿐이다.

최종 UX = 로봇(rig) + 팔 소스 + 손 소스만 고르면 끝:
    --arm controller  손 = 글러브   → GloveModel  (컨트롤러 팔 + 글러브 손 + 햅틱)
    --arm wrist                      → QuestModel  (Quest 핸드트래킹: 손목→팔, 손가락→손)

실행:
    pip install -e ~/whatslab-sdk[receiver,arm,hand]
    # 컨트롤러(팔) + 글러브(손):
    python ~/whatslab-sdk/examples/quest_arm.py --rig rigs/nero_orca_right.yaml --side right
    # Quest 핸드트래킹 단독:
    python ~/whatslab-sdk/examples/quest_arm.py --rig rigs/nero_orca_right.yaml --arm wrist

    실행 중 Enter → yaw 캘리브(머리연동), 'r' + Enter → reach 캘리브(8초 뻗기).
"""
import argparse
import threading
import time

import numpy as np

from whatslab.model import GloveModel, QuestModel
from whatslab.robot import RobotModel, load_rig


def _build_model(args, robot):
    if args.arm == "wrist":
        return QuestModel(robot)                  # Quest 핸드트래킹: 손목→팔, 손가락→손
    return GloveModel(robot)                      # 컨트롤러→팔, 글러브→손 + 햅틱


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
    ap.add_argument("--no-calib", action="store_true",
                    help="캘리브 전처리(yaw W + reach 스케일 + 캘리브 원점 p0) 를 끄고 "
                         "리시버 좌표를 그대로 목표로 쓴다 — A/B 비교용")
    args = ap.parse_args()

    rig = load_rig(args.rig)
    if args.no_calib:
        rig.calibration.enabled = False
    robot = RobotModel(rig)
    model = _build_model(args, robot)

    print(f"[setup] rig={rig.name} arm={args.arm} hand={'on' if robot.has_hand else 'off'} "
          f"reach_max={rig.solver.reach_max} "
          f"calib={'on(W+scale+p0)' if rig.calibration.enabled else 'off(raw)'}")
    print(f"[setup] arm_joints={robot.arm_joint_names}")

    if not args.no_safety:      # rig max_joint_velocity 를 get_q 출력에 강제
        from robot_io import attach_safety
        sf = attach_safety(model, robot, args.rate)
        print(f"[safety] rate-limit {'on' if sf else 'off(설정 없음)'} "
              f"({rig.solver.max_joint_velocity} rad/s @ {args.rate:g}Hz)")

    model.start()

    viz = None
    bridge = None
    if args.viz:                          # 팔+손 메쉬 + 목표(/target)·EE(/ee) 프레임
        from whatslab.viz import RobotArmViz
        viz = RobotArmViz(robot, port=args.port)
        viz.start()
        if args.robot:
            from robot_io import build_robot_panel   # examples/ 동거 모듈
            bridge = build_robot_panel(model, robot, args)   # 실물 연결 GUI (기본 미연결)
        print(f"[viz] viser: http://localhost:{args.port}")
    elif args.robot:
        print("[robot] --robot 은 --viz 와 함께 써야 합니다(연결 버튼이 viser 패널에 있음)")

    # 캘리브 콘솔: Enter=yaw(자세, 즉시 캡처), 'r'+Enter=reach(위치, 8초 측정).
    def _calib_loop():
        while True:
            try:
                line = input()
            except (EOFError, KeyboardInterrupt):
                return
            if line.strip().lower() == "r":
                print("[calib] reach 측정 시작 — 팔을 최대 범위로 뻗으세요(8초)...", flush=True)
                r = model.calibrate_reach(persist=True)   # rig yaml 에 저장
                # print(f"[calib] reach 완료: input_reach={r.get("right"):.3f} m", flush=True)
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
            q = model.get_q()        # 논블로킹: 최신 입력 pull → (캐시)IK/리타게팅
            right_q = q.get(args.side) or {}
            if diag is not None:     # 단계별 계측 (원시입력 → 목표 → 클램프 → IK오차)
                raw = model._get_raw_target().get(args.side)
                q_arm_v = (np.array([right_q[n] for n in arm_names], dtype=float)
                           if all(n in right_q for n in arm_names) else None)
                diag.tick(raw, q_arm_v, now)
            if bridge is not None:   # 실물 전송 (패널의 `송신` 이 켜져 있을 때만)
                bridge.send(right_q)
            # 입력이 없는 컴포넌트는 q 에서 생략된다(0 을 채우지 않는다) → 팔 관절이
            # 아직 없으면 viz/출력을 건너뛴다(시작 직후 Quest 신호 대기 구간).
            has_arm_q = all(n in right_q for n in arm_names)
            if viz is not None and has_arm_q:
                # Model 이 채운 멤버(target)로 목표 프레임까지 그린다 — 재계산 없음.
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
