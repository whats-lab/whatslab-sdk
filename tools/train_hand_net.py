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
from whatslab.solvers.hand.net_losses import (AffineHandNet, coverage_loss,
                                              distance_loss, motion_loss_local,
                                              pinch_loss, position_loss,
                                              smooth_loss)
from whatslab.solvers.hand.net_retargeter import NetHandRetargeter
from whatslab.solvers.hand.torch_fk import TorchKeyvectorFK

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import bench_hand_retarget as B  # noqa: E402

PINCH_MM = 15.0
ABD_TAG = "abd"
MOTION_LO = 0.001
MOTION_HI = 0.011


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


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--config", default="robotis_hx5_d20")
    ap.add_argument("--side", default="left")
    ap.add_argument("--dump", default=None,
                    help="캘리 덤프. 실측 q 원본이 없을 때만 필요하다 — --real-npz 가"
                         " 있거나 --random-mode synergy 면 생략한다")
    ap.add_argument("--profiles", default=None)
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
    ap.add_argument("--save-every", type=int, default=1)
    ap.add_argument("--w-pinch", type=float, default=1.0)
    ap.add_argument("--w-dist", type=float, default=0.0,
                    help="손끝 쌍거리(접선). 굽힘 진폭을 깎는다 — 측정 근거는 계획서")
    ap.add_argument("--w-smooth", type=float, default=0.0,
                    help="입력 섭동당 출력 변화량(게인)을 벌금 — |dq| 떨림 억제")
    ap.add_argument("--w-pos", type=float, default=0.0,
                    help="손가락별 6D keyvector 일치. 반경·각도를 동시에 정하므로"
                         " 쌍거리(--w-dist)가 불필요해진다 — 손가락 간 결합 없음")
    ap.add_argument("--no-affine", action="store_true",
                    help="residual affine 을 끈다 — 기본은 켜져 있다. 손마다 크기가"
                         " 달라서 L_dist/L_ext 가 형태 불일치와 정면으로 부딪힌다")
    ap.add_argument("--affine", action="store_true",
                    help="기본값이라 무동작 — 명시용으로만 남긴다")
    ap.add_argument("--partial-chamfer", action="store_true",
                    help="--chamfer forward 와 같다 (구 스크립트 호환)")
    ap.add_argument("--chamfer", default=None,
                    choices=["both", "forward", "reverse"],
                    help="both=GeoRT 원본 / forward=사람->로봇(이 구조에서 무정보,"
                         " 붕괴를 보상) / reverse=로봇->사람 = coverage 본체")
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
    elif args.dump:
        if not args.profiles:
            ap.error("--dump 를 주면 --profiles 도 필요하다")
        side, trajs = B.load_poses(args.dump, args.profiles, 20,
                                   ("pinch", "flex", "abd"))
        if side != args.side:
            ap.error("덤프 side=%s 와 --side %s 가 다르다" % (side, args.side))
        real_q = np.asarray([[r.fk.q_from_named(a)[i] for i in iq_h]
                             for traj in trajs.values() for a in traj])
    else:
        if args.random_mode in ("combo", "mix"):
            ap.error("--random-mode %s 는 실측 q 를 손가락별로 조합하므로 실측이 필요하다"
                     " — --real-npz 나 --dump 를 주거나 --random-mode synergy 를 쓴다"
                     % args.random_mode)
        real_q = np.zeros((0, len(names_h)))
        print("실측 q 없음 — 순수 비지도(합성 %d, %s)"
              % (args.random, args.random_mode), flush=True)
    real_q = np.repeat(real_q, args.real_repeat, axis=0)
    print("q 공간: 관절 %d, 블록 %s, 펼침지표 flat %.4f / fist %.4f"
          % (len(names_h), {k: len(v) for k, v in blocks.items()},
             extension(r, iq_h, q_flat), extension(r, iq_h, q_fist)), flush=True)

    if args.phase == 2:
        args.partial_chamfer = True

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
    args.affine = not args.no_affine
    net = r.net.to(dtype=dt, device=dev)
    if args.affine:
        net = AffineHandNet(net, len(FINGERS)).to(dtype=dt, device=dev)
    net.train()
    opt = torch.optim.AdamW(net.parameters(), lr=args.lr)

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

            def perturb(keep=False):
                d = F.normalize(torch.randn_like(x), dim=-1)
                step = MOTION_LO + torch.rand(x.shape[0], 1, 1, dtype=x.dtype,
                                              device=x.device) * (
                    MOTION_HI - MOTION_LO)
                dx = d * step
                if keep:
                    return dx, to_kv(x + dx) - y, step
                return dx, to_kv(x + dx) - y

            zero = torch.zeros((), dtype=x.dtype, device=x.device)
            motion = zero
            if args.w_motion > 0.0:
                dx_a, dy_a = perturb()
                dx_b, dy_b = perturb()
                motion = motion_loss_local(dx_a, dy_a, dx_b, dy_b)

            cover = zero
            if args.w_coverage > 0.0:
                sel = torch.randint(0, bank.shape[0],
                                    (min(args.bank_batch, bank.shape[0]),))
                cover = coverage_loss(y, bank[sel], partial=args.partial_chamfer,
                                      mode=args.chamfer)
            pinch = pinch_loss(x, y, pinch_thr) if args.w_pinch > 0.0 else zero

            loss = (motion * args.w_motion + cover * args.w_coverage
                    + pinch * args.w_pinch)
            dist = zero
            if args.w_dist > 0.0:
                dist = distance_loss(x, y)
                loss = loss + dist * args.w_dist
            smooth = zero
            if args.w_smooth > 0.0:
                d = F.normalize(torch.randn_like(x), dim=-1)
                step = MOTION_LO + torch.rand(x.shape[0], 1, 1, dtype=x.dtype,
                                              device=x.device) * (
                    MOTION_HI - MOTION_LO)
                smooth = smooth_loss(net(x), net(x + d * step), step)
                loss = loss + smooth * args.w_smooth
            pos = zero
            if args.w_pos > 0.0:
                pos = position_loss(x, y)
                loss = loss + pos * args.w_pos
            opt.zero_grad()
            loss.backward()
            opt.step()
            acc = acc + torch.stack([motion.detach(), cover.detach(),
                                     pinch.detach(), pos.detach(),
                                     dist.detach(), smooth.detach()])
            nb += 1

        vals = (acc / max(nb, 1)).cpu().numpy()
        print("epoch %3d  motion %.4e  cover %.4e  pinch %.4e  pos %.4e"
              "  dist %.4e  smooth %.4e" % (epoch, *vals), flush=True)
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
