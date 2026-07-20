#!/usr/bin/env python3
"""AirGlove(AGA) → 손 리타게팅 최소 실행 예제 (HandModel).

AGA 글러브 OSC 를 받아 로봇 손 관절각을 계산해 주기적으로 출력한다(팔 없음).
whatslab.model.HandModel 이 글러브 리시버 + 리타게팅을 한 객체로 묶어 폴링식
get_q(side) 하나로 낸다 — 소비처는 GloveModel/QuestModel 과 동일하게 다룬다.

실행:
    pip install -e ~/whatslab-sdk[receiver,hand]        # python-osc, dex_retargeting, pinocchio
    python ~/whatslab-sdk/examples/airglove_hand.py --config orca_hand --side right
    # urdf-root 는 --urdf-root 또는 WHATSLAB_MODELS_ROOT 로 지정(미지정 시 패키지 내장)
"""
import argparse
import os
import time

import numpy as np

from whatslab.model import HandModel


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default="orca_hand", help="로봇 손 리타게팅 config")
    ap.add_argument("--side", default="right", choices=["left", "right"])
    ap.add_argument("--urdf-root", default=os.environ.get("WHATSLAB_MODELS_ROOT"),
                    help="models 디렉토리 (미지정 시 WHATSLAB_MODELS_ROOT / 패키지 내장)")
    ap.add_argument("--rate", type=float, default=30.0, help="처리/출력 주기 (Hz)")
    ap.add_argument("--viz", action="store_true", help="viser 사람/로봇 손 스켈레톤")
    args = ap.parse_args()

    src = args.urdf_root or "(패키지 내장 URDF)"
    print(f"[setup] config={args.config} side={args.side} models={src}")
    m = HandModel(hand_config=args.config, side=args.side, urdf_root=args.urdf_root)
    ctrl = m.retarget[args.side]        # 리타게터(viz engine 접근 + joint_names)

    viz_human = viz_robot = None
    if args.viz:
        from whatslab.viz import HandSkeletonViz, RobotHandViz
        viz_human = HandSkeletonViz()          # 청록: 사람 손 타깃
        viz_human.start()
        viz_robot = RobotHandViz(ctrl.engine)  # 주황: 로봇 손(q FK)
        print("[viz] viser: 사람 손(청록) + 로봇 손(주황) 오버레이")

    m.start()
    print(f"[run] AGA OSC 수신 대기. Ctrl-C 로 종료.")
    print(f"[run] joint_names({len(ctrl.joint_names)}): {ctrl.joint_names}")

    period, last_log = 1.0 / args.rate, 0.0
    try:
        while True:
            q = m.get_q(args.side)             # {joint: rad}
            tracked = bool(m.get_data(args.side).get("tracked"))
            now = time.monotonic()
            if viz_human is not None and tracked:  # 사람+로봇 오버레이(root 정렬)
                eng = ctrl.engine
                viz_human.update(eng.last_human_positions + eng._wrist_offset, timestamp=now)
                viz_robot.update(np.array([q[n] for n in ctrl.joint_names]), timestamp=now)
            if now - last_log > 0.2:
                last_log = now
                status = "TRACKED" if tracked else "no-signal"
                print(f"[{status}] q = {np.round([q[n] for n in ctrl.joint_names], 3).tolist()}")
            time.sleep(period)
    except KeyboardInterrupt:
        print("\n[stop] 종료")
    finally:
        m.stop()


if __name__ == "__main__":
    main()
