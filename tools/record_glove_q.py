#!/usr/bin/env python3
"""글러브가 보내는 사람 손 관절각(q)을 그대로 기록한다 — 학습용 실분포 수집."""
import argparse
import os
import time

import numpy as np
import pinocchio as pin

from whatslab.receiver.glove.human_hand import GloveHumanAnglesReceiver
from whatslab.solvers.hand.human_fk import HumanHandFK
from whatslab.solvers.hand.keyvector import HandKeyvector, human_chains

DEDUP_RAD = 1e-4


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--side", default="left", choices=["left", "right"])
    ap.add_argument("--out", required=True, help="저장할 npz 경로")
    ap.add_argument("--rate", type=float, default=60.0)
    ap.add_argument("--duration", type=float, default=0.0, help="초 (0 이면 Ctrl-C 까지)")
    ap.add_argument("--append", action="store_true", help="기존 npz 에 이어붙인다")
    ap.add_argument("--keep-still", action="store_true",
                    help="정지 프레임도 전부 남긴다 (기본은 중복 제거)")
    args = ap.parse_args()

    fk = HumanHandFK(args.side)
    kv = HandKeyvector(fk.model, fk.data, human_chains(fk),
                       "%s_sensor_dorsum" % args.side)
    names = list(fk.joint_names)
    idx = [fk._idx_q[n] for n in names]
    print("[setup] side=%s 관절 %d개  urdf=%s"
          % (args.side, len(names), os.path.basename(fk.urdf_path)))

    rows, stamps = [], []
    if args.append and os.path.exists(args.out):
        old = np.load(args.out, allow_pickle=False)
        if list(old["joint_names"]) != names:
            raise SystemExit("기존 파일의 관절 이름이 다르다 — 다른 프로파일/side 다")
        rows = list(old["q"])
        stamps = list(old["t"])
        print("[append] 기존 %d 프레임에 이어붙인다" % len(rows))

    src = GloveHumanAnglesReceiver()
    src.start()
    print("[run] /joint_angles 수신 대기. Ctrl-C 로 저장하고 종료.")

    period = 1.0 / args.rate
    t_end = time.monotonic() + args.duration if args.duration > 0 else None
    last_log, n_seen, prev, q = 0.0, 0, None, None
    try:
        while t_end is None or time.monotonic() < t_end:
            t0 = time.monotonic()
            s = src.get(args.side)
            ang = s.hand.joint_angles if (s.hand is not None and s.tracked) else None
            if ang:
                n_seen += 1
                q = fk.q_from_named(ang)
                qv = q[idx].copy()
                new = (prev is None or args.keep_still
                       or float(np.abs(qv - prev).max()) > DEDUP_RAD)
                if new:
                    rows.append(qv)
                    stamps.append(t0)
                    prev = qv
            now = time.monotonic()
            if now - last_log > 1.0:
                last_log = now
                if not ang or q is None:
                    print("[no-signal] 글러브 데이터 없음", flush=True)
                else:
                    ext = float(np.linalg.norm(kv.encode(q)[:, :3], axis=1).mean())
                    print("[rec] %5d 프레임 (수신 %d)  펼침지표 %.3f"
                          % (len(rows), n_seen, ext), flush=True)
            time.sleep(max(0.0, period - (time.monotonic() - t0)))
    except KeyboardInterrupt:
        pass
    finally:
        src.stop()

    if not rows:
        print("\n[stop] 기록된 프레임이 없다 — 저장하지 않는다")
        return 1
    q_all = np.asarray(rows, dtype=float)
    ext = np.array([float(np.linalg.norm(
        kv.encode(_full(fk, idx, v))[:, :3], axis=1).mean()) for v in q_all])
    np.savez_compressed(args.out, q=q_all, t=np.asarray(stamps, dtype=float),
                        joint_names=np.asarray(names), side=args.side,
                        urdf=fk.urdf_path)
    print("\n[stop] %d 프레임 저장 → %s" % (len(q_all), args.out))
    print("  펼침지표 범위 %.3f ~ %.3f (평균 %.3f)" % (ext.min(), ext.max(), ext.mean()))
    lo, hi = 0.3927, 0.8863
    h = np.histogram(ext, bins=10, range=(lo, hi))[0]
    print("  주먹→펼침 10등분 분포 %s" % (h / max(h.sum(), 1) * 100).round(0).astype(int).tolist())
    print("  관절별 사용범위(deg): %s" % np.round(
        np.rad2deg(q_all.max(0) - q_all.min(0)), 0).astype(int).tolist())
    return 0


def _full(fk, idx, qv):
    q = pin.neutral(fk.model)
    q[idx] = qv
    return q


if __name__ == "__main__":
    raise SystemExit(main())
