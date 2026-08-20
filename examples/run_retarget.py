#!/usr/bin/env python3
"""ONNX 리타게터를 손 하나에 물려 프레임당 지연을 잰다.

이 브랜치는 배포 경로만 남긴 것이다 — 사람 관절각이 들어가고 로봇 관절각이
나오는 그래프 하나. 학습·평가·시각화는 없다.

    python examples/run_retarget.py --hand left --robot orca_hand

`--onnx` 로 다른 모델을 지정하면 assets 기본값 대신 그걸 쓴다. 분야별로 다른
체크포인트를 굽어 놓고 갈아 끼우며 비교하는 용도다.
"""
from __future__ import annotations

import argparse
import time

import numpy as np

from whatslab.solvers import UniRetargeter


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--hand", default="left", choices=("left", "right"))
    ap.add_argument("--robot", default="orca_hand")
    ap.add_argument("--onnx", default=None, help="기본은 assets/uni_all.onnx")
    ap.add_argument("--tables", default=None)
    ap.add_argument("--threads", type=int, default=1)
    ap.add_argument("--iters", type=int, default=500)
    ap.add_argument("--warm", type=int, default=50)
    a = ap.parse_args()

    rt = UniRetargeter(a.hand, a.robot, onnx_path=a.onnx,
                       tables_path=a.tables, threads=a.threads)
    print(f"{a.robot} [{a.hand}] 로봇 관절 {len(rt.joint_names)}개, "
          f"사람 관절 {len(rt.human_joint_names)}개")

    # 관절각은 라디안. 실제 텔레옵에서는 장갑 수신기가 이 사전을 채운다.
    rng = np.random.default_rng(0)
    frames = [
        {n: float(v) for n, v in
         zip(rt.human_joint_names, rng.uniform(-0.3, 0.9,
                                               len(rt.human_joint_names)))}
        for _ in range(a.warm + a.iters)
    ]

    for f in frames[:a.warm]:
        rt.compute(f)
    t0 = time.perf_counter()
    for f in frames[a.warm:]:
        rt.compute(f)
    dt = (time.perf_counter() - t0) / a.iters * 1e3
    print(f"프레임당 {dt:.3f} ms  -> {1000.0 / dt:.0f} Hz "
          f"(스레드 {a.threads})")
    print("마지막 출력(라디안):",
          np.array2string(rt.compute(frames[-1]), precision=3))


if __name__ == "__main__":
    main()
