#!/usr/bin/env python3
"""사람 keyvector → 로봇 관절각 네트워크를 정확 FK 로 학습한다 (Phase 1: GeoRT 5원칙)."""
import argparse
import json
import os
import sys

import numpy as np
import pinocchio as pin
import torch
import torch.nn.functional as F

from whatslab.solvers.hand.fk_torch import KeyvectorFK
from whatslab.solvers.hand.human_fk import FINGERS
from whatslab.solvers.hand.net_losses import (AffineHandNet, align_loss,
                                              coverage_loss, distance_loss,
                                              flatness_loss, motion_loss_global,
                                              motion_loss_local, pinch_loss)
from whatslab.solvers.hand.net_retargeter import NetHandRetargeter
from whatslab.solvers.hand.torch_fk import TorchKeyvectorFK

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import bench_hand_retarget as B  # noqa: E402

PINCH_MM = 15.0
MOTION_LO = 0.001
MOTION_HI = 0.011
FLAT_EPS = 0.002


def robot_bank(r, n, seed=0):
    rng = np.random.default_rng(seed)
    u = rng.uniform(-1.0, 1.0, (n, len(r.joint_names)))
    out = np.empty((n, len(FINGERS), 6))
    q = pin.neutral(r.model)
    for i in range(n):
        q[r._iq] = r.to_joint(u[i])
        out[i] = r.kv.encode(q)
    return out


def uniform_thetas(r, n, seed=0):
    rng = np.random.default_rng(seed)
    lo = r.fk.model.lowerPositionLimit
    hi = r.fk.model.upperPositionLimit
    idx = [r.fk._idx_q[n_] for n_ in r.fk.joint_names]
    q = pin.neutral(r.fk.model)
    out = []
    for _ in range(n):
        for i in idx:
            q[i] = rng.uniform(lo[i], hi[i])
        out.append(r.hkv.encode(q))
    return np.asarray(out)


def combo_thetas(real, blocks, n, seed=0):
    rng = np.random.default_rng(seed)
    out = np.empty((n, real.shape[1]))
    for b in blocks.values():
        pick = rng.integers(0, real.shape[0], n)
        out[:, b] = real[pick][:, b]
    return out


def synergy_thetas(real, blocks, n, seed=0, jitter=0.25, shared=True):
    rng = np.random.default_rng(seed)
    lo = real.min(axis=0)
    hi = real.max(axis=0)
    out = np.empty((n, real.shape[1]))
    g = rng.uniform(0.0, 1.0, (n, 1)) if shared else None
    for b in blocks.values():
        s = g if shared else rng.uniform(0.0, 1.0, (n, 1))
        f = np.clip(s + rng.uniform(-jitter, jitter, (n, 1)), 0.0, 1.0)
        f = np.clip(f + rng.uniform(-0.05, 0.05, (n, len(b))), 0.0, 1.0)
        out[:, b] = lo[b] + f * (hi[b] - lo[b])
    return out


def human_samples(r, real, blocks, expand, n_random, mode, seed=0):
    rows = [r.hkv.encode(r.fk.q_from_named(expand(th))) for th in real]
    n_real = len(rows)
    if n_random > 0:
        if mode == "uniform":
            rows.extend(uniform_thetas(r, n_random, seed))
        else:
            if mode == "combo":
                th_all = combo_thetas(real, blocks, n_random, seed)
            elif mode == "synergy":
                th_all = synergy_thetas(real, blocks, n_random, seed)
            else:
                third = n_random // 3
                th_all = np.concatenate([
                    synergy_thetas(real, blocks, n_random - 2 * third, seed),
                    synergy_thetas(real, blocks, third, seed + 1, shared=False),
                    combo_thetas(real, blocks, third, seed + 2)])
            for th in th_all:
                rows.append(r.hkv.encode(r.fk.q_from_named(expand(th))))
    return np.asarray(rows), n_real


def bank_q(r, n, seed=0):
    rng = np.random.default_rng(seed)
    return rng.uniform(-1.0, 1.0, (n, len(r.joint_names)))


def reach_extent(r, u):
    q = pin.neutral(r.model)
    q[r._iq] = r.to_joint(u)
    kv = r.kv.encode(q)
    return float(np.linalg.norm(kv[:, :3], axis=1).mean()), kv


def robot_anchors(r, n, seed=0):
    u = bank_q(r, n, seed)
    ext = np.array([reach_extent(r, row)[0] for row in u])
    return u[int(ext.argmax())], u[int(ext.argmin())]


def anchor_pairs(r, thetas_lo, thetas_hi, expand, u_flat, u_fist, k):
    xs, ys = [], []
    q = pin.neutral(r.model)
    for a in np.linspace(0.0, 1.0, k):
        theta = thetas_lo + (thetas_hi - thetas_lo) * a
        xs.append(r.hkv.encode(r.fk.q_from_named(expand(theta))))
        q[r._iq] = r.to_joint(u_flat + (u_fist - u_flat) * a)
        ys.append(r.kv.encode(q))
    return np.asarray(xs), np.asarray(ys)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--config", default="robotis_hx5_d20")
    ap.add_argument("--side", default="left")
    ap.add_argument("--dump", required=True)
    ap.add_argument("--profiles", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--epochs", type=int, default=200)
    ap.add_argument("--batch", type=int, default=256)
    ap.add_argument("--bank", type=int, default=20000)
    ap.add_argument("--bank-batch", type=int, default=2048)
    ap.add_argument("--random", type=int, default=6000)
    ap.add_argument("--random-mode", default="mix",
                    choices=["synergy", "combo", "uniform", "mix"],
                    help="synergy=손가락별 굽힘정도 균등 / combo=실측 블록 조합"
                         " / uniform=관절 독립균등 / mix=synergy+combo 반반")
    ap.add_argument("--real-repeat", type=int, default=1,
                    help="실측 프레임 반복 횟수 (실분포 비중 조절)")
    ap.add_argument("--lr", type=float, default=1e-4)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--phase", type=int, default=1, choices=[1, 2])
    ap.add_argument("--device", default="cpu")
    ap.add_argument("--fp64", action="store_true",
                    help="기본은 float32 — 정본 대조 검증용으로만 fp64 를 쓴다")
    ap.add_argument("--fk", default="torch", choices=["torch", "pinocchio"])
    ap.add_argument("--w-motion", type=float, default=1.0)
    ap.add_argument("--w-coverage", type=float, default=80.0)
    ap.add_argument("--w-flatness", type=float, default=0.0)
    ap.add_argument("--save-every", type=int, default=1)
    ap.add_argument("--w-pinch", type=float, default=1.0)
    ap.add_argument("--w-dist", type=float, default=1.0)
    ap.add_argument("--w-align", type=float, default=1.0)
    ap.add_argument("--anchors", type=int, default=50)
    ap.add_argument("--no-affine", action="store_true")
    ap.add_argument("--partial-chamfer", action="store_true")
    ap.add_argument("--local-motion", action="store_true")
    args = ap.parse_args()

    torch.manual_seed(args.seed)
    dt = torch.float64 if args.fp64 else torch.float32
    dev = torch.device(args.device)
    r = NetHandRetargeter(args.side, args.config)
    if args.fk == "torch":
        fk = TorchKeyvectorFK(r.kv, r._iq, r.joint_names, dtype=dt).to(dev)
    else:
        if args.device != "cpu" or not args.fp64:
            ap.error("--fk pinocchio 는 --device cpu --fp64 만 된다")
        fk = KeyvectorFK(r.kv, r._iq, r._iv, pin.neutral(r.model))
    side, trajs = B.load_poses(args.dump, args.profiles, 20,
                               ("pinch", "flex", "abd"))
    if side != args.side:
        ap.error("덤프 side=%s 와 --side %s 가 다르다" % (side, args.side))
    expand, anchor_lo, anchor_hi = B.theta_expander(args.dump, args.profiles)
    theta_names, real_th = B.real_thetas(args.dump, args.profiles, 20)
    blocks = B.theta_blocks(theta_names)
    real_th = np.repeat(real_th, args.real_repeat, axis=0)

    if args.phase == 2:
        args.partial_chamfer = True
        args.local_motion = True

    X, n_real = human_samples(r, real_th, blocks, expand, args.random,
                              args.random_mode, args.seed)
    X = torch.as_tensor(X, dtype=dt, device=dev)
    bank = torch.as_tensor(robot_bank(r, args.bank, args.seed), dtype=dt, device=dev)
    lo = torch.as_tensor(r.lower, dtype=dt, device=dev)
    hi = torch.as_tensor(r.upper, dtype=dt, device=dev)
    pinch_thr = PINCH_MM * 1e-3 / r.hkv.l_ref
    print("사람 %d (실측 %d + 합성 %d/%s)  로봇 bank %d  관절 %d  핀치임계 %.4f"
          "  FK %s/%s/%s" % (X.shape[0], n_real, args.random, args.random_mode,
                             bank.shape[0], len(r.joint_names), pinch_thr, args.fk,
                             args.device, "f64" if args.fp64 else "f32"), flush=True)

    os.makedirs(args.out, exist_ok=True)
    ckpt = os.path.join(args.out, "last.pt")
    net = r.net.to(dtype=dt, device=dev)
    if args.phase == 2 and not args.no_affine:
        net = AffineHandNet(net, len(FINGERS)).to(dtype=dt, device=dev)
    net.train()
    opt = torch.optim.AdamW(net.parameters(), lr=args.lr)

    ax = ay = None
    if args.phase == 2 and args.w_align > 0.0:
        u_flat, u_fist = robot_anchors(r, args.bank, args.seed)
        ax_np, ay_np = anchor_pairs(r, anchor_lo, anchor_hi, expand,
                                    u_flat, u_fist, args.anchors)
        ax = torch.as_tensor(ax_np, dtype=dt, device=dev)
        ay = torch.as_tensor(ay_np, dtype=dt, device=dev)
        print("앵커 %d 쌍 (flat↔fist 보간, 양쪽 다 물리 파라미터에서 생성)"
              % ax.shape[0], flush=True)
    start = 0
    if os.path.exists(ckpt):
        sd = torch.load(ckpt, map_location="cpu")
        net.load_state_dict(sd["net"])
        opt.load_state_dict(sd["opt"])
        start = int(sd["epoch"]) + 1
        print("[resume] epoch %d 부터" % start, flush=True)

    def to_kv(x):
        return fk(lo + (net(x) + 1.0) * 0.5 * (hi - lo))

    for epoch in range(start, args.epochs):
        perm = torch.randperm(X.shape[0])
        acc = torch.zeros(6, dtype=dt, device=dev)
        nb = 0
        for s in range(0, X.shape[0] - args.batch + 1, args.batch):
            x = X[perm[s:s + args.batch]]
            y = to_kv(x)

            def perturb():
                d = F.normalize(torch.randn_like(x), dim=-1)
                step = MOTION_LO + torch.rand(x.shape[0], 1, 1, dtype=x.dtype,
                                              device=x.device) * (
                    MOTION_HI - MOTION_LO)
                dx = d * step
                return dx, to_kv(x + dx) - y

            zero = torch.zeros((), dtype=x.dtype, device=x.device)
            motion = zero
            if args.w_motion > 0.0:
                dx_a, dy_a = perturb()
                if args.local_motion:
                    dx_b, dy_b = perturb()
                    motion = motion_loss_local(dx_a, dy_a, dx_b, dy_b)
                else:
                    motion = motion_loss_global(dx_a, dy_a)

            flat = zero
            if args.w_flatness > 0.0:
                d2 = F.normalize(torch.randn_like(x), dim=-1) * FLAT_EPS
                flat = flatness_loss(to_kv(x + d2), to_kv(x - d2), y)

            cover = zero
            if args.w_coverage > 0.0:
                sel = torch.randint(0, bank.shape[0],
                                    (min(args.bank_batch, bank.shape[0]),))
                cover = coverage_loss(y, bank[sel], partial=args.partial_chamfer)
            pinch = pinch_loss(x, y, pinch_thr) if args.w_pinch > 0.0 else zero

            loss = (motion * args.w_motion + cover * args.w_coverage
                    + flat * args.w_flatness + pinch * args.w_pinch)
            dist = zero
            align = zero
            if args.phase == 2:
                dist = distance_loss(x, y)
                loss = loss + dist * args.w_dist
                if ax is not None:
                    align = align_loss(to_kv(ax), ay)
                    loss = loss + align * args.w_align
            opt.zero_grad()
            loss.backward()
            opt.step()
            acc = acc + torch.stack([motion.detach(), cover.detach(),
                                     flat.detach(), pinch.detach(),
                                     dist.detach(), align.detach()])
            nb += 1

        vals = (acc / max(nb, 1)).cpu().numpy()
        print("epoch %3d  motion %+.4f  cover %.4e  flat %.4e  pinch %.4e"
              "  dist %.4e  align %.4e" % (epoch, *vals), flush=True)
        if (epoch + 1) % args.save_every == 0 or epoch + 1 == args.epochs:
            torch.save({"net": net.state_dict(), "opt": opt.state_dict(),
                        "epoch": epoch, "cfg": vars(args)}, ckpt + ".tmp")
            os.replace(ckpt + ".tmp", ckpt)
            torch.save({"net": net.state_dict(), "affine": args.phase == 2
                        and not args.no_affine},
                       os.path.join(args.out, "net.pt"))
    with open(os.path.join(args.out, "meta.json"), "w") as fh:
        json.dump(vars(args), fh, indent=2)


if __name__ == "__main__":
    main()
