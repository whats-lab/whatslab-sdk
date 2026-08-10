#!/usr/bin/env python3
"""글러브 실기로 손 리타게팅을 검증한다 — 프레임별 입력·해·접촉·오차를 npz 로 기록."""
import argparse
import os
import time

import numpy as np

from whatslab.teleop import HandModel

PAIR_KEYS = ("thumb-index", "thumb-middle", "thumb-ring", "thumb-pinky")


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--config", default="orca_hand", help="로봇 손 리타게팅 config")
    ap.add_argument("--side", default="right", choices=["left", "right"])
    ap.add_argument("--backend", default="kp", choices=["dex", "kp"])
    ap.add_argument("--urdf-root", default=os.environ.get("WHATSLAB_MODELS_ROOT"))
    ap.add_argument("--rate", type=float, default=60.0)
    ap.add_argument("--dump", default=None, help="프레임 기록 npz 경로")
    ap.add_argument("--duration", type=float, default=0.0, help="초 (0 이면 Ctrl-C 까지)")
    ap.add_argument("--viz", action="store_true",
                    help="viser: 로봇 손 메쉬 + 목표점(하늘)·달성점(주황)·오차선")
    ap.add_argument("--viz-port", type=int, default=8080)
    args = ap.parse_args()

    print(f"[setup] {args.config} {args.side} backend={args.backend}")
    print(f"[setup] models={args.urdf_root or '(패키지 내장)'}")
    m = HandModel(hand_config=args.config, side=args.side, urdf_root=args.urdf_root,
                  backend=args.backend)
    ctrl = m.sides[args.side].retarget
    eng = ctrl.engine
    kp = args.backend == "kp"

    viz = None
    if args.viz:
        if not kp:
            ap.error("--viz 는 --backend kp 에서만 지원한다")
        from whatslab.viz import KPHandViz
        viz = KPHandViz(eng, port=args.viz_port)
        viz.start()
        print("[viz] 하늘색=목표 키포인트, 주황=로봇 달성, 빨간선=오차")

    m.start()
    print(f"[run] 글러브 OSC 수신 대기 (Ctrl-C 종료). 로봇 관절 {len(ctrl.joint_names)}개")
    if kp:
        print(f"[run] 사람 관절 {len(getattr(eng, 'human_joint_names', []))}개, "
              f"scale={eng.scale:.3f}")

    rec = {k: [] for k in ("t", "q", "tip_err", "dq")}
    for k in PAIR_KEYS:
        rec[f"r_{k}"] = []
        rec[f"h_{k}"] = []
    period, last_log, prev_q = 1.0 / args.rate, 0.0, None
    n_tracked, n_total = 0, 0
    t_end = time.monotonic() + args.duration if args.duration > 0 else None

    try:
        while t_end is None or time.monotonic() < t_end:
            t0 = time.monotonic()
            q = m.get_q()[args.side]
            tracked = bool(m.get_data()[args.side].get("tracked"))
            n_total += 1
            if tracked and q:
                n_tracked += 1
                qv = np.array([q[n] for n in ctrl.joint_names])
                rec["t"].append(t0)
                rec["q"].append(qv)
                rec["dq"].append(0.0 if prev_q is None
                                 else float(np.abs(qv - prev_q).max()))
                prev_q = qv
                if kp:
                    rob, hum = eng.contact_pairs(), eng.target_contact_pairs()
                    rec["tip_err"].append(eng.tip_error())
                    for pk in PAIR_KEYS:
                        rec[f"r_{pk}"].append(rob.get(pk, np.nan))
                        rec[f"h_{pk}"].append(hum.get(pk, np.nan))
                if viz is not None:
                    viz.update(timestamp=t0)

            now = time.monotonic()
            if now - last_log > 0.3:
                last_log = now
                if not tracked:
                    print("[no-signal] 글러브 데이터 없음", flush=True)
                elif kp and rec["tip_err"]:
                    print("[TRACKED] 지문오차 %5.1fmm  엄지-검지 목표 %5.1f → 로봇 %5.1fmm"
                          "  |dq| %.3f" % (
                              rec["tip_err"][-1] * 1e3, rec["h_thumb-index"][-1] * 1e3,
                              rec["r_thumb-index"][-1] * 1e3, rec["dq"][-1]), flush=True)
                else:
                    print("[TRACKED] q = %s" % np.round(
                        [q[n] for n in ctrl.joint_names], 3).tolist(), flush=True)
            time.sleep(max(0.0, period - (time.monotonic() - t0)))
    except KeyboardInterrupt:
        pass
    finally:
        m.stop()

    print("\n[stop] 프레임 %d (추적 %d, %.0f%%)" % (
        n_total, n_tracked, 100.0 * n_tracked / max(n_total, 1)))
    if kp and rec["tip_err"]:
        te = np.array(rec["tip_err"]) * 1e3
        dq = np.array(rec["dq"])
        print("  지문오차 mean %.1f / p95 %.1f mm     |dq| p95 %.3f rad" % (
            te.mean(), np.percentile(te, 95), np.percentile(dq, 95)))
        for pk in PAIR_KEYS:
            h = np.array(rec[f"h_{pk}"]) * 1e3
            r = np.array(rec[f"r_{pk}"]) * 1e3
            ok = np.isfinite(h) & np.isfinite(r)
            if not ok.any():
                continue
            near = ok & (h <= 30.0)
            tail = ("  핀치구간(목표≤30mm) %d프레임 → 로봇 %.1fmm" % (
                near.sum(), r[near].mean())) if near.any() else "  핀치 없음"
            print("  %-14s 목표 최소 %.1f → 로봇 %.1fmm%s" % (
                pk, h[ok].min(), r[ok][np.argmin(h[ok])], tail))
    if args.dump and rec["t"]:
        np.savez_compressed(args.dump, joint_names=np.array(ctrl.joint_names),
                            **{k: np.array(v) for k, v in rec.items() if v})
        print("  기록: %s (%d 프레임)" % (args.dump, len(rec["t"])))


if __name__ == "__main__":
    main()
