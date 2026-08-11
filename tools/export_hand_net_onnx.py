#!/usr/bin/env python3
"""학습된 net 리타게터를 ONNX 로 내보낸다 — 사람 q 하나 넣으면 로봇 q 가 나온다."""
import argparse
import os
import time

import numpy as np
import onnxruntime as ort
import pinocchio as pin
import torch
import torch.nn as nn

from whatslab.solvers.hand.human_fk import FINGERS
from whatslab.solvers.hand.net_losses import AffineHandNet
from whatslab.solvers.hand.net_retargeter import NetHandRetargeter
from whatslab.solvers.hand.torch_fk import TorchKeyvectorFK

OPSET = 18


class HandRetargetGraph(nn.Module):

    def __init__(self, r: NetHandRetargeter, mirror: bool = False):
        super().__init__()
        names = list(r.fk.joint_names)
        iq = [r.fk._idx_q[n] for n in names]
        self.fk = TorchKeyvectorFK(r.hkv, iq, names, dtype=torch.float32)
        self.net = r.net
        self.register_buffer("lower", torch.as_tensor(r.lower, dtype=torch.float32))
        self.register_buffer("upper", torch.as_tensor(r.upper, dtype=torch.float32))
        flip = np.array([1.0, 1.0, -1.0] * 2) if mirror else np.ones(6)
        self.register_buffer("flip", torch.as_tensor(flip, dtype=torch.float32))

    def forward(self, q_human: torch.Tensor) -> torch.Tensor:
        x = self.fk(q_human) * self.flip
        unit = self.net(x)
        return self.lower + (unit + 1.0) * 0.5 * (self.upper - self.lower)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--config", default="robotis_hx5_d20")
    ap.add_argument("--side", default="left")
    ap.add_argument("--checkpoint", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--mirror", action="store_true",
                    help="좌우 통합 모델을 반대 side 에 쓸 때 z 성분을 반전한다")
    ap.add_argument("--samples", type=int, default=256)
    args = ap.parse_args()

    sd = torch.load(args.checkpoint, map_location="cpu")
    r = NetHandRetargeter(args.side, args.config)
    net = r.net
    if bool(sd.get("affine", False)):
        net = AffineHandNet(net, len(FINGERS)).double()
        r.net = net
    net.load_state_dict(sd["net"] if "net" in sd else sd)
    r.net = net.float().eval()

    graph = HandRetargetGraph(r, mirror=args.mirror).eval()
    n_h = len(r.fk.joint_names)
    print("사람 관절 %d → 로봇 관절 %d  (affine=%s, mirror=%s)"
          % (n_h, len(r.joint_names), bool(sd.get("affine", False)), args.mirror))

    rng = np.random.default_rng(0)
    lo = r.fk.model.lowerPositionLimit[[r.fk._idx_q[n] for n in r.fk.joint_names]]
    hi = r.fk.model.upperPositionLimit[[r.fk._idx_q[n] for n in r.fk.joint_names]]
    Q = rng.uniform(lo, hi, (args.samples, n_h)).astype(np.float32)

    os.makedirs(os.path.dirname(os.path.abspath(args.out)) or ".", exist_ok=True)
    torch.onnx.export(
        graph, (torch.as_tensor(Q[:2]),), args.out, opset_version=OPSET,
        input_names=["q_human"], output_names=["q_robot"],
        dynamic_shapes={"q_human": {0: torch.export.Dim("batch")}})
    print("내보냄: %s (%.1f KB)" % (args.out, os.path.getsize(args.out) / 1024))

    sess = ort.InferenceSession(args.out, providers=["CPUExecutionProvider"])
    got = sess.run(None, {"q_human": Q})[0]

    ref = np.empty_like(got)
    for i, row in enumerate(Q):
        ang = {n: float(v) for n, v in zip(r.fk.joint_names, row)}
        ref[i] = r.compute(ang)
    err = float(np.abs(got - ref).max())
    print("pinocchio+torch 경로와 최대 오차 %.3e rad (%.4f deg)"
          % (err, np.rad2deg(err)))
    if err > 1e-4:
        raise SystemExit("오차가 크다 — 그래프가 정본 경로와 다르다")

    def bench(fn, n=200):
        fn()
        ts = []
        for _ in range(n):
            t0 = time.perf_counter()
            fn()
            ts.append(time.perf_counter() - t0)
        return float(np.median(ts)) * 1e3

    one = Q[:1]
    ang1 = {n: float(v) for n, v in zip(r.fk.joint_names, Q[0])}
    print("\n단일 프레임 지연 (중앙값)")
    print("  pinocchio+torch (현재)  %7.3f ms" % bench(lambda: r.compute(ang1)))
    print("  onnxruntime CPU         %7.3f ms"
          % bench(lambda: sess.run(None, {"q_human": one})))
    print("배치 %d" % args.samples)
    print("  onnxruntime CPU         %7.3f ms  (프레임당 %.4f ms)"
          % (bench(lambda: sess.run(None, {"q_human": Q}), 50),
             bench(lambda: sess.run(None, {"q_human": Q}), 50) / args.samples))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
