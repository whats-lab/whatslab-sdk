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

from whatslab.model import GloveModel, QuestModel
from whatslab.robot import RobotModel, load_rig


def _build_model(args, robot):
    """팔/손 소스 선택 → 프리셋 Model. (OSC 포트는 프리셋 내부 기본값 — 바꾸려면
    해당 모델 정의부에서 수정. 커스텀 조합은 TeleopModel 상속으로 직접 정의 가능.)"""
    if args.arm == "wrist":
        # Quest 핸드트래킹: 손목 pose → 팔 IK, 손가락 joints → 손 리타게팅.
        return QuestModel(robot)
    # 컨트롤러(팔) + 글러브(손) + 햅틱.
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
    args = ap.parse_args()

    rig = load_rig(args.rig)
    robot = RobotModel(rig)
    model = _build_model(args, robot)

    print(f"[setup] rig={rig.name} arm={args.arm} hand={'on' if robot.has_hand else 'off'} "
          f"reach_max={rig.solver.reach_max}")
    print(f"[setup] arm_joints={robot.arm_joint_names}")

    model.start()

    viz = None
    if args.viz:                          # 팔+손 메쉬 + 목표(/target)·EE(/ee) 프레임
        from whatslab.viz import RobotArmViz
        viz = RobotArmViz(robot, port=args.port)
        viz.start()
        print(f"[viz] viser: http://localhost:{args.port}")

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
    try:
        while True:
            now = time.monotonic()
            q = model.get_q()        # 논블로킹: 최신 입력 pull → (캐시)IK/리타게팅
            q=q.get("right")
            
            if viz is not None:
                # Model 이 채운 멤버(target)로 목표 프레임까지 그린다 — 재계산 없음.
                arm_q = [q[n] for n in arm_names]
                hand_names = [k for k in q if k not in arm_set]
                hand_q = [q[n] for n in hand_names]
                viz.update(arm_q, target_pose=model.target.get(args.side),
                           hand_q=hand_q, hand_names=hand_names, timestamp=now)
            if now - last > 0.2:
                last = now
                arm_q = [round(q[n], 3) for n in arm_names]
                tgt = "on" if model.target.get(args.side) is not None else "--"
                print(f"\r[q] arm={arm_q} hand={len(q) - len(arm_names)}j target={tgt}   ",
                      end="", flush=True)
            time.sleep(period)
    except KeyboardInterrupt:
        print("\n[stop] 종료")
    finally:
        model.stop()


if __name__ == "__main__":
    main()
