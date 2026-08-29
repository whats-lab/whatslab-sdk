#!/usr/bin/env python3
"""센서 회전 → 관절각 IK 네트워크. 클린 입력의 DLS 해를 GT 로 쓰고 입력에만 노이즈를 넣는다."""
import argparse
import os
import time

import numpy as np
import torch
import torch.nn as nn


def so3_exp(w):
    th = torch.linalg.norm(w, dim=-1, keepdim=True).clamp_min(1e-12)
    a = w / th
    K = torch.zeros(w.shape[:-1] + (3, 3), device=w.device, dtype=w.dtype)
    K[..., 0, 1] = -a[..., 2]; K[..., 0, 2] = a[..., 1]
    K[..., 1, 0] = a[..., 2]; K[..., 1, 2] = -a[..., 0]
    K[..., 2, 0] = -a[..., 1]; K[..., 2, 1] = a[..., 0]
    I = torch.eye(3, device=w.device, dtype=w.dtype).expand_as(K)
    return I + torch.sin(th)[..., None] * K \
        + (1 - torch.cos(th))[..., None] * (K @ K)


def rand_rot(shape, sigma, device):
    ax = torch.randn(shape + (3,), device=device)
    ax = ax / ax.norm(dim=-1, keepdim=True).clamp_min(1e-12)
    return so3_exp(ax * torch.randn(shape + (1,), device=device).abs() * sigma)


def corrupt(R, sig_off, sig_frame, device):
    B = R.shape[0]
    Md = rand_rot((B,), sig_off, device)[:, None]
    Mt = rand_rot((B, 5), sig_off, device)
    Rn = Md.transpose(-1, -2) @ R @ Mt
    if sig_frame > 0:
        Rn = Rn @ rand_rot((B, 5), sig_frame, device)
    return Rn


def to6d(R):
    return torch.cat([R[..., :, 0], R[..., :, 1]], dim=-1).reshape(
        R.shape[0], -1)


class Net(nn.Module):

    def __init__(self, nin, nout, hidden, layers):
        super().__init__()
        mods, d = [], nin
        for _ in range(layers):
            mods += [nn.Linear(d, hidden), nn.LayerNorm(hidden), nn.GELU()]
            d = hidden
        self.f = nn.Sequential(*mods, nn.Linear(d, nout))

    def forward(self, x):
        return self.f(x)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--train", required=True)
    ap.add_argument("--val", required=True)
    ap.add_argument("--offset-deg", type=float, default=15.0)
    ap.add_argument("--frame-deg", type=float, default=1.5)
    ap.add_argument("--epochs", type=int, default=60)
    ap.add_argument("--batch", type=int, default=8192)
    ap.add_argument("--lr", type=float, default=1e-3)
    ap.add_argument("--hidden", type=int, default=512)
    ap.add_argument("--layers", type=int, default=4)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--out", default=None)
    ap.add_argument("--dump-val", default=None)
    ap.add_argument("--label", default="dls", choices=["dls", "true"])
    a = ap.parse_args()
    torch.manual_seed(a.seed)
    dev = "cuda" if torch.cuda.is_available() else "cpu"
    ztr, zva = np.load(a.train), np.load(a.val)
    Rtr = torch.as_tensor(ztr["R"])
    key = "q_gt" if a.label == "dls" else "q_true"
    qtr = torch.as_tensor(ztr[key])
    Rva = torch.as_tensor(zva["R"]).to(dev)
    qva = torch.as_tensor(zva[key]).to(dev)
    names = [str(x) for x in ztr["names"]]
    so, sf = np.deg2rad(a.offset_deg), np.deg2rad(a.frame_deg)
    print("학습 %d / 검증 %d / dev %s / 라벨 %s / 오프셋 %.0f° 프레임 %.1f°"
          % (len(Rtr), len(Rva), dev, a.label, a.offset_deg, a.frame_deg),
          flush=True)
    print("기준선(학습 GT 평균) MAE %.2f°"
          % float(np.degrees((qva.cpu() - qtr.mean(0)).abs().mean())), flush=True)

    g = torch.Generator().manual_seed(a.seed)
    torch.manual_seed(a.seed + 1)
    Rva_n = corrupt(Rva, so, sf, dev)
    xva = to6d(Rva_n)
    net = Net(30, qtr.shape[1], a.hidden, a.layers).to(dev)
    opt = torch.optim.AdamW(net.parameters(), lr=a.lr, weight_decay=1e-4)
    sch = torch.optim.lr_scheduler.CosineAnnealingLR(opt, a.epochs)
    t0, best = time.time(), 1e9
    for ep in range(a.epochs):
        net.train()
        perm = torch.randperm(len(Rtr), generator=g)
        tot = n = 0
        for i in range(0, len(perm) - a.batch + 1, a.batch):
            sl = perm[i:i + a.batch]
            Rb = Rtr[sl].to(dev)
            x = to6d(corrupt(Rb, so, sf, dev))
            loss = (net(x) - qtr[sl].to(dev)).abs().mean()
            opt.zero_grad(); loss.backward(); opt.step()
            tot += float(loss.detach()) * len(sl); n += len(sl)
        sch.step()
        net.eval()
        with torch.no_grad():
            e = (net(xva) - qva).abs()
            mae = float(np.degrees(e.mean().cpu()))
            p95 = float(np.degrees(np.percentile(e.cpu().numpy(), 95)))
        if mae < best:
            best = mae
            if a.out:
                torch.save({"sd": net.state_dict(), "hidden": a.hidden,
                            "layers": a.layers, "nout": int(qtr.shape[1]),
                            "names": names}, a.out)
        if ep % 5 == 0 or ep == a.epochs - 1:
            print("ep %3d  train %.4f  val MAE %.2f°  p95 %.2f°  (%.0fs)"
                  % (ep, np.degrees(tot / max(n, 1)), mae, p95,
                     time.time() - t0), flush=True)
    print("최적 val MAE %.2f°" % best, flush=True)
    net.eval()
    with torch.no_grad():
        per = np.degrees((net(xva) - qva).abs().mean(0).cpu().numpy())
    print("\n%-24s %10s" % ("관절", "net MAE[°]"))
    for i in np.argsort(-per):
        print("%-24s %10.2f" % (names[i], per[i]))
    if a.dump_val:
        np.savez(a.dump_val, R_noisy=Rva_n.cpu().numpy().astype(np.float32),
                 q_gt=qva.cpu().numpy(), per_joint=per,
                 names=np.array(names))
        print("\n검증 입력 기록 %s" % a.dump_val)


if __name__ == "__main__":
    main()
