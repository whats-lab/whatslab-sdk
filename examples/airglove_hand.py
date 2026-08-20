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
    args = ap.parse_args()

    src = args.urdf_root or "(패키지 내장 URDF)"
    print(f"[setup] config={args.config} side={args.side} models={src}")
    m = HandModel(hand_config=args.config, side=args.side, urdf_root=args.urdf_root)
    ctrl = m.sides[args.side].retarget

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
