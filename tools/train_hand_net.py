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
                                              bone_loss,
                                              coverage_loss, distance_loss,
                                              flatness_loss, motion_loss_global,
                                              motion_loss_local, pinch_loss)
from whatslab.solvers.hand.net_retargeter import NetHandRetargeter
from whatslab.solvers.hand.torch_fk import TorchKeyvectorFK

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import bench_hand_retarget as B  # noqa: E402

PINCH_MM = 15.0
ABD_TAG = "abd"
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


def q_axes(r):
    fk = r.fk
    names = list(fk.joint_names)
    iq = [fk._idx_q[n] for n in names]
    lo = fk.model.lowerPositionLimit[iq].copy()
    hi = fk.model.upperPositionLimit[iq].copy()
    return names, np.asarray(iq), lo, hi


def extension(r, iq, qv):
    q = pin.neutral(r.fk.model)
    q[iq] = qv
    return float(np.linalg.norm(r.hkv.encode(q)[:, :3], axis=1).mean())


def flex_mask(names):
    m = np.array([ABD_TAG not in n for n in names])
    if not m.any() or m.all():
        raise ValueError("굽힘/외전 관절을 이름으로 못 가른다 ('%s'): %s"
                         % (ABD_TAG, names))
    return m


def flat_fist(r):
    names, iq, lo, hi = q_axes(r)
    flex = flex_mask(names)
    flat = np.zeros(len(names))
    fist = flat.copy()
    for j in np.flatnonzero(flex):
        a, b = flat.copy(), flat.copy()
        a[j], b[j] = lo[j], hi[j]
        fist[j] = lo[j] if extension(r, iq, a) <= extension(r, iq, b) else hi[j]
    return flat, fist


def joint_blocks(r):
    out = {}
    for j, n in enumerate(r.fk.joint_names):
        for f in FINGERS:
            if f in n:
                out.setdefault(f, []).append(j)
                break
    return out


def q_synergy(flat, fist, blocks, lo, hi, flex, n, seed=0, shared=True,
              jitter=0.25):
    rng = np.random.default_rng(seed)
    out = np.empty((n, flat.size))
    g = rng.uniform(0.0, 1.0, (n, 1)) if shared else None
    for b in blocks.values():
        s = g if shared else rng.uniform(0.0, 1.0, (n, 1))
        f = np.clip(s + rng.uniform(-jitter, jitter, (n, 1)), 0.0, 1.0)
        f = np.clip(f + rng.uniform(-0.05, 0.05, (n, len(b))), 0.0, 1.0)
        out[:, b] = flat[b] + f * (fist[b] - flat[b])
    abd = np.flatnonzero(~flex)
    a = rng.uniform(0.0, 1.0, (n, 1))
    out[:, abd] = a * rng.uniform(lo[abd], hi[abd], (n, abd.size))
    return out


def q_combo(real_q, blocks, n, seed=0):
    rng = np.random.default_rng(seed)
    out = np.empty((n, real_q.shape[1]))
    for b in blocks.values():
        out[:, b] = real_q[rng.integers(0, real_q.shape[0], n)][:, b]
    return out


def q_uniform(lo, hi, n, seed=0):
    rng = np.random.default_rng(seed)
    return rng.uniform(lo, hi, (n, lo.size))


def human_samples(r, real_q, blocks, flat, fist, lo, hi, n_random, mode, seed=0):
    iq = np.asarray([r.fk._idx_q[n] for n in r.fk.joint_names])
    flex = flex_mask(list(r.fk.joint_names))
    rows = list(real_q)
    n_real = len(rows)
    if n_random > 0:
        if mode == "uniform":
            rows.extend(q_uniform(lo, hi, n_random, seed))
        elif mode == "combo":
            rows.extend(q_combo(real_q, blocks, n_random, seed))
        elif mode == "synergy":
            rows.extend(q_synergy(flat, fist, blocks, lo, hi, flex,
                                  n_random, seed))
        else:
            k = n_random // 3
            rows.extend(q_synergy(flat, fist, blocks, lo, hi, flex,
                                  n_random - 2 * k, seed))
            rows.extend(q_synergy(flat, fist, blocks, lo, hi, flex, k, seed + 1,
                                  shared=False))
            rows.extend(q_combo(real_q, blocks, k, seed + 2))
    q = pin.neutral(r.fk.model)
    out = np.empty((len(rows), len(FINGERS), 6))
    for i, qv in enumerate(rows):
        q[iq] = qv
        out[i] = r.hkv.encode(q)
    return out, n_real


def robot_extent(r, u):
    q = pin.neutral(r.model)
    q[r._iq] = r.to_joint(u)
    return float(np.linalg.norm(r.kv.encode(q)[:, :3], axis=1).mean())


def robot_anchors(r, n, seed=0):
    n_j = len(r.joint_names)
    rng = np.random.default_rng(seed)
    per = np.empty(n_j)
    for j in range(n_j):
        a, b = np.zeros(n_j), np.zeros(n_j)
        a[j], b[j] = -1.0, 1.0
        per[j] = -1.0 if robot_extent(r, a) >= robot_extent(r, b) else 1.0
    cand = [np.zeros(n_j), per, -per]
    cand += list(rng.uniform(-1.0, 1.0, (max(n, 1), n_j)))
    ext = np.array([robot_extent(r, u) for u in cand])
    return cand[int(ext.argmax())], cand[int(ext.argmin())]


def anchor_pairs(r, q_flat, q_fist, iq_h, u_flat, u_fist, k):
    xs, ys = [], []
    qr = pin.neutral(r.model)
    qh = pin.neutral(r.fk.model)
    for a in np.linspace(0.0, 1.0, k):
        qh[iq_h] = q_flat + (q_fist - q_flat) * a
        xs.append(r.hkv.encode(qh))
        qr[r._iq] = r.to_joint(u_flat + (u_fist - u_flat) * a)
        ys.append(r.kv.encode(qr))
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
                    help="q 공간 생성. synergy=전역 굽힘스칼라 공유(펼침·주먹 커버)"
                         " / combo=실측 q 를 손가락 블록별로 조합 / uniform=관절"
                         " 독립균등(양 끝을 못 만든다) / mix=셋 1:1:1")
    ap.add_argument("--real-npz", default=None,
                    help="tools/record_glove_q.py 로 기록한 실측 q (실분포 정본)")
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
    ap.add_argument("--w-bone", type=float, default=0.0,
                    help="손가락 내부 뼈 방향(tip-prox) 일치 — 손끝마디 굽힘 억제")
    ap.add_argument("--w-dist", type=float, default=0.0)
    ap.add_argument("--w-align", type=float, default=0.0)
    ap.add_argument("--anchors", type=int, default=50)
    ap.add_argument("--no-affine", action="store_true")
    ap.add_argument("--affine", action="store_true",
                    help="phase 1 에서도 residual affine 을 켠다")
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
    names_h, iq_h, lo_h, hi_h = q_axes(r)
    blocks = joint_blocks(r)
    q_flat, q_fist = flat_fist(r)
    if args.real_npz:
        rec = np.load(args.real_npz, allow_pickle=False)
        if list(rec["joint_names"]) != names_h:
            raise SystemExit("--real-npz 의 관절 이름이 사람 URDF 와 다르다")
        if str(rec["side"]) != args.side:
            raise SystemExit("--real-npz side=%s 와 --side %s 가 다르다"
                             % (rec["side"], args.side))
        real_q = np.asarray(rec["q"], dtype=float)
        print("실측 q %d 프레임 (%s)" % (real_q.shape[0], args.real_npz), flush=True)
    else:
        real_q = np.asarray([[r.fk.q_from_named(a)[i] for i in iq_h]
                             for traj in trajs.values() for a in traj])
    real_q = np.repeat(real_q, args.real_repeat, axis=0)
    print("q 공간: 관절 %d, 블록 %s, 펼침지표 flat %.4f / fist %.4f"
          % (len(names_h), {k: len(v) for k, v in blocks.items()},
             extension(r, iq_h, q_flat), extension(r, iq_h, q_fist)), flush=True)

    if args.phase == 2:
        args.partial_chamfer = True
        args.local_motion = True
        if args.w_dist == 0.0:
            args.w_dist = 1.0
        if args.w_align == 0.0:
            args.w_align = 1.0

    X, n_real = human_samples(r, real_q, blocks, q_flat, q_fist, lo_h, hi_h,
                              args.random, args.random_mode, args.seed)
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
    args.affine = args.affine or (args.phase == 2 and not args.no_affine)
    net = r.net.to(dtype=dt, device=dev)
    if args.affine:
        net = AffineHandNet(net, len(FINGERS)).to(dtype=dt, device=dev)
    net.train()
    opt = torch.optim.AdamW(net.parameters(), lr=args.lr)

    ax = ay = None
    if args.w_align > 0.0:
        u_flat, u_fist = robot_anchors(r, min(args.bank, 4000), args.seed)
        ax_np, ay_np = anchor_pairs(r, q_flat, q_fist, iq_h, u_flat, u_fist,
                                    args.anchors)
        ax = torch.as_tensor(ax_np, dtype=dt, device=dev)
        ay = torch.as_tensor(ay_np, dtype=dt, device=dev)
        print("앵커 %d 쌍  로봇 펼침지표 flat %.4f / fist %.4f"
              % (ax.shape[0], robot_extent(r, u_flat), robot_extent(r, u_fist)),
              flush=True)
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
            bone = bone_loss(x, y) if args.w_bone > 0.0 else zero

            loss = (motion * args.w_motion + cover * args.w_coverage
                    + flat * args.w_flatness + pinch * args.w_pinch
                    + bone * args.w_bone)
            dist = zero
            align = zero
            if args.w_dist > 0.0:
                dist = distance_loss(x, y)
                loss = loss + dist * args.w_dist
            if ax is not None:
                align = align_loss(to_kv(ax), ay)
                loss = loss + align * args.w_align
            opt.zero_grad()
            loss.backward()
            opt.step()
            acc = acc + torch.stack([motion.detach(), cover.detach(),
                                     bone.detach(), pinch.detach(),
                                     dist.detach(), align.detach()])
            nb += 1

        vals = (acc / max(nb, 1)).cpu().numpy()
        print("epoch %3d  motion %+.4f  cover %.4e  bone %.4e  pinch %.4e"
              "  dist %.4e  align %.4e" % (epoch, *vals), flush=True)
        if (epoch + 1) % args.save_every == 0 or epoch + 1 == args.epochs:
            torch.save({"net": net.state_dict(), "opt": opt.state_dict(),
                        "epoch": epoch, "cfg": vars(args)}, ckpt + ".tmp")
            os.replace(ckpt + ".tmp", ckpt)
            torch.save({"net": net.state_dict(), "affine": args.affine},
                       os.path.join(args.out, "net.pt"))
    with open(os.path.join(args.out, "meta.json"), "w") as fh:
        json.dump(vars(args), fh, indent=2)


if __name__ == "__main__":
    main()
