#!/usr/bin/env python3
import argparse
import os
import signal
import threading
import time

import numpy as np

from whatslab.hand import HandModel
from whatslab.retarget.retargeter import DEX_PREFIX


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "--config",
        default="dex:orca",
        help="손 리타게팅 — dex:<configs/hands 이름> (DexPilot) 또는"
        " geort:<configs/geort 이름> (GeoRT 소프트 프롬프트, 예 geort:orca_right)",
    )
    ap.add_argument("--side", default="right", choices=["left", "right"])
    ap.add_argument(
        "--urdf-root",
        default=os.environ.get("WHATSLAB_MODELS_ROOT"),
        help="models 디렉토리 (미지정 시 WHATSLAB_MODELS_ROOT / 패키지 내장)",
    )
    ap.add_argument("--rate", type=float, default=30.0, help="처리/출력 주기 (Hz)")
    ap.add_argument("--viz", action="store_true", help="viser 로봇 손 URDF 메쉬")
    ap.add_argument(
        "--viz-urdf",
        default=None,
        help="geort 엔진 viz 용 로봇 손 URDF (dex 는 설정의 URDF 를 쓴다)",
    )
    args = ap.parse_args()

    src = args.urdf_root or "(패키지 내장 URDF)"
    print(f"[setup] config={args.config} side={args.side} models={src}", flush=True)
    dex = args.config.startswith(DEX_PREFIX)
    m = HandModel(
        config=args.config, side=args.side, urdf_root=args.urdf_root if dex else None
    )
    ctrl = m.retarget

    viz_robot = None
    if args.viz:
        from whatslab.viz import HandViz

        urdf = ctrl.engine.urdf_path if dex else args.viz_urdf
        if urdf is None:
            raise SystemExit("geort 엔진 viz 는 --viz-urdf 가 필요하다")
        names = ctrl.engine.robot_joint_names if dex else ctrl.joint_names
        viz_robot = HandViz(urdf, names, root_path="/robot_hand")
        viz_robot.start()
        print("[viz] 로봇 손 (메쉬 %s)" % ("O" if viz_robot.mesh_mode else "X"), flush=True)

    m.start()
    print("[run] Spine 3.0.0 대기 (UDP 4040). Ctrl-C 로 종료.", flush=True)
    print(f"[run] joint_names({len(ctrl.joint_names)}): {ctrl.joint_names}", flush=True)

    period, last_log, q_prev, dq_max = 1.0 / args.rate, 0.0, None, 0.0
    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    while not stop.is_set():
        q = m.get_q()[args.side]
        tracked = all(n in q for n in ctrl.joint_names)
        now = time.monotonic()
        qv = np.array([q[n] for n in ctrl.joint_names]) if tracked else None
        if qv is not None:
            if q_prev is not None:
                dq_max = max(dq_max, float(np.abs(qv - q_prev).max()))
            q_prev = qv
        if viz_robot is not None and tracked:
            viz_robot.update(ctrl.engine.full_qpos(qv) if dex else qv)
        if now - last_log > 0.2:
            last_log = now
            if not tracked:
                print("[no-signal] 글러브 손 프레임 수신 없음", flush=True)
            elif dq_max < 1e-4:
                print(
                    "[정지] 글러브는 붙었지만 손이 안 움직인다 —"
                    f" q = {np.round(qv, 3).tolist()}",
                    flush=True,
                )
            else:
                print(
                    f"[추적] |dq|max {dq_max:.3f}  q = {np.round(qv, 3).tolist()}", flush=True
                )
            dq_max = 0.0
        time.sleep(period)
    print("\n[stop] 종료")
    m.stop()


if __name__ == "__main__":
    main()
