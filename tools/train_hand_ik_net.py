#!/usr/bin/env python3
"""손목·손끝 센서 회전에서 손 관절각을 추정하는 IK 네트워크를 학습한다."""
import argparse
import glob
import os
import time

import numpy as np
import torch
import torch.nn as nn

FING = ["thumb", "index", "middle", "ring", "pinky"]


def so3_exp(w):
    th = torch.linalg.norm(w, dim=-1, keepdim=True).clamp_min(1e-12)
    a = w / th
    K = torch.zeros(w.shape[:-1] + (3, 3), device=w.device, dtype=w.dtype)
    K[..., 0, 1] = -a[..., 2]; K[..., 0, 2] = a[..., 1]
    K[..., 1, 0] = a[..., 2]; K[..., 1, 2] = -a[..., 0]
    K[..., 2, 0] = -a[..., 1]; K[..., 2, 1] = a[..., 0]
    I = torch.eye(3, device=w.device, dtype=w.dtype).expand_as(K)
    s = torch.sin(th)[..., None]
    c = (1 - torch.cos(th))[..., None]
    return I + s * K + c * (K @ K)


def rand_rot(shape, sigma, device):
    ax = torch.randn(shape + (3,), device=device)
    ax = ax / ax.norm(dim=-1, keepdim=True).clamp_min(1e-12)
    mag = torch.randn(shape + (1,), device=device).abs() * sigma
    return so3_exp(ax * mag)


def to6d(R):
    return torch.cat([R[..., :, 0], R[..., :, 1]], dim=-1)


class Net(nn.Module):

    def __init__(self, nin, nout, hidden=512, layers=4):
        super().__init__()
        mods, d = [], nin
        for _ in range(layers):
            mods += [nn.Linear(d, hidden), nn.LayerNorm(hidden), nn.GELU()]
            d = hidden
        mods += [nn.Linear(d, nout)]
        self.f = nn.Sequential(*mods)

    def forward(self, x):
        return self.f(x)


def load(sensor_dir, win, stride, max_frames_per_file, dilate=1):
    files = sorted(glob.glob(os.path.join(sensor_dir, "*_s.npz")))
    by_sub = {}
    for f in files:
        sub = os.path.basename(f).split("_")[0]
        z = np.load(f)
        R, q = z["R"], z["q"]
        span = (win - 1) * dilate + 1
        if len(R) <= span:
            continue
        if max_frames_per_file and len(R) - span > max_frames_per_file:
            k = np.linspace(0, len(R) - span - 1, max_frames_per_file).astype(int)
        else:
            k = np.arange(0, len(R) - span, stride)
        if len(k) == 0:
            continue
        idx = k[:, None] + (np.arange(win) * dilate)[None, :]
        by_sub.setdefault(sub, []).append(
            (R[idx].astype(np.float32), q[idx[:, -1]].astype(np.float32)))
    out = {}
    for s, v in by_sub.items():
        out[s] = (np.concatenate([a for a, _ in v]),
                  np.concatenate([b for _, b in v]))
    return out


def make_batch(R, q, sigma_off, sigma_frame, device):
    R = torch.as_tensor(R, device=device)
    q = torch.as_tensor(q, device=device)
    B, T = R.shape[:2]
    Md = rand_rot((B,), sigma_off, device)[:, None, None]
    Mt = rand_rot((B, 5), sigma_off, device)[:, None]
    Rn = Md.transpose(-1, -2) @ R @ Mt
    if sigma_frame > 0:
        Rn = Rn @ rand_rot((B, T, 5), sigma_frame, device)
    return to6d(Rn).reshape(B, -1), q


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--sensors", required=True)
    ap.add_argument("--win", type=int, default=8)
    ap.add_argument("--stride", type=int, default=3)
    ap.add_argument("--dilate", type=int, default=1)
    ap.add_argument("--cap", type=int, default=4000)
    ap.add_argument("--val-subs", nargs="*", default=["B004", "B006", "B030"])
    ap.add_argument("--offset-deg", type=float, default=15.0)
    ap.add_argument("--frame-deg", type=float, default=1.5)
    ap.add_argument("--epochs", type=int, default=60)
    ap.add_argument("--batch", type=int, default=4096)
    ap.add_argument("--lr", type=float, default=3e-4)
    ap.add_argument("--hidden", type=int, default=512)
    ap.add_argument("--layers", type=int, default=4)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--out", default=None)
    a = ap.parse_args()

    torch.manual_seed(a.seed)
    dev = "cuda" if torch.cuda.is_available() else "cpu"
    data = load(a.sensors, a.win, a.stride, a.cap, a.dilate)
    tr = [s for s in sorted(data) if s not in a.val_subs]
    va = [s for s in sorted(data) if s in a.val_subs]
    if not va:
        raise SystemExit("검증 피험자 없음")
    Rtr = np.concatenate([data[s][0] for s in tr])
    qtr = np.concatenate([data[s][1] for s in tr])
    Rva = np.concatenate([data[s][0] for s in va])
    qva = np.concatenate([data[s][1] for s in va])
    print("학습 %s (%d win) / 검증 %s (%d win) / dev %s"
          % (tr, len(Rtr), va, len(Rva), dev), flush=True)

    base_pred = torch.as_tensor(qtr.mean(0), device=dev)
    bl = float(np.degrees(np.abs(qva - qtr.mean(0)).mean()))
    print("기준선(학습 평균 예측) MAE %.2f°  |  검증 q 표준편차 %.2f°"
          % (bl, float(np.degrees(qva.std(0).mean()))), flush=True)

    sg_o = np.deg2rad(a.offset_deg)
    sg_f = np.deg2rad(a.frame_deg)
    nin = a.win * 5 * 6
    net = Net(nin, qtr.shape[1], a.hidden, a.layers).to(dev)
    opt = torch.optim.AdamW(net.parameters(), lr=a.lr, weight_decay=1e-4)
    sch = torch.optim.lr_scheduler.CosineAnnealingLR(opt, a.epochs)
    g = torch.Generator().manual_seed(a.seed)
    t0 = time.time()
    best = 1e9
    for ep in range(a.epochs):
        net.train()
        perm = torch.randperm(len(Rtr), generator=g).numpy()
        tot = n = 0
        for i in range(0, len(perm) - a.batch + 1, a.batch):
            sl = perm[i:i + a.batch]
            x, y = make_batch(Rtr[sl], qtr[sl], sg_o, sg_f, dev)
            loss = (net(x) - y).abs().mean()
            opt.zero_grad(); loss.backward(); opt.step()
            tot += float(loss) * len(sl); n += len(sl)
        sch.step()
        net.eval()
        with torch.no_grad():
            errs = []
            for i in range(0, len(Rva), a.batch):
                x, y = make_batch(Rva[i:i + a.batch], qva[i:i + a.batch],
                                  sg_o, sg_f, dev)
                errs.append((net(x) - y).abs().cpu())
            e = torch.cat(errs)
            mae = float(np.degrees(e.mean()))
            p95 = float(np.degrees(np.percentile(e.numpy(), 95)))
        if mae < best:
            best = mae
            if a.out:
                torch.save({"sd": net.state_dict(), "win": a.win,
                            "nin": nin, "nout": qtr.shape[1],
                            "hidden": a.hidden, "layers": a.layers}, a.out)
        print("ep %3d  train %.4f  val MAE %.2f°  p95 %.2f°  (%.0fs)"
              % (ep, np.degrees(tot / max(n, 1)), mae, p95, time.time() - t0),
              flush=True)
    print("최적 검증 MAE %.2f°  (기준선 %.2f°, 개선 %.1f배)"
          % (best, bl, bl / max(best, 1e-9)), flush=True)
    net.eval()
    with torch.no_grad():
        per = []
        for i in range(0, len(Rva), a.batch):
            x, y = make_batch(Rva[i:i + a.batch], qva[i:i + a.batch],
                              sg_o, sg_f, dev)
            per.append((net(x) - y).abs().cpu().numpy())
        per = np.degrees(np.concatenate(per).mean(0))
    blp = np.degrees(np.abs(qva - qtr.mean(0)).mean(0))
    print("\n%-24s %8s %8s" % ("관절", "MAE[°]", "기준선[°]"))
    order = np.argsort(-per)
    nm = [str(x) for x in np.load(sorted(glob.glob(
        os.path.join(a.sensors, "*_s.npz")))[0])["names"]]
    for i in order:
        print("%-24s %8.2f %8.2f" % (nm[i], per[i], blp[i]))


main()
