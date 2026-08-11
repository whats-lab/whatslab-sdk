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

from whatslab.solvers.hand.net_losses import (U_MARGIN, AffineHandNet,
                                              bone_loss, coverage_loss,
                                              motion_loss_global, pinch_loss,
                                              orientation_loss, position_loss,
                                              posture_loss, saturation_loss,
                                              unit_to_joint)
from whatslab.solvers.hand.net_retargeter import (ACTS, NORMS,
                                                  NetHandRetargeter)
from whatslab.solvers.hand.torch_fk import TorchKeyvectorFK

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import bench_hand_retarget as B  # noqa: E402

PINCH_MM = 15.0
COLLAPSE_DEG = 5.0
SAT_U = 0.99
VAL_FRAC = 0.1
WEIGHT_DECAY = 0.01
DROPOUT = 0.0
HIDDEN = 128
LAYERS = 2
ACT = "leaky"
NORM = "none"
W_MOTION = 1.0
W_COVERAGE = 5.0
W_BONE = 20.0
W_PINCH = 1.0
W_POS = 20.0
W_ORIENT = 0.0
W_SAT = 0.0
W_POSTURE = 0.0
ABD_TAG = "abd"
MOTION_LO = 0.001
MOTION_HI = 0.011


def robot_bank(r, n, seed=0):
    rng = np.random.default_rng(seed)
    u = rng.uniform(-1.0, 1.0, (n, len(r.joint_names)))
    out = np.empty((n, len(r.fingers), 6))
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
        for f in r.fingers:
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
    out = np.empty((len(rows), len(r.fingers), 6))
    rot = np.empty((len(rows), len(r.fingers), 3, 3))
    tips = [r.hkv.fids[f][-1] for f in r.fingers]
    for i, qv in enumerate(rows):
        q[iq] = qv
        out[i] = r.hkv.encode(q)
        rot[i] = np.array([r.hkv.rot.T @ r.fk.data.oMf[t].rotation for t in tips])
    return out, rot, n_real


def neutral_offset(r):
    qh = pin.neutral(r.fk.model)
    pin.forwardKinematics(r.fk.model, r.fk.data, qh)
    pin.updateFramePlacements(r.fk.model, r.fk.data)
    qr = pin.neutral(r.model)
    pin.forwardKinematics(r.model, r.data, qr)
    pin.updateFramePlacements(r.model, r.data)
    out = np.empty((len(r.fingers), 3, 3))
    for i, f in enumerate(r.fingers):
        ah = r.hkv.rot.T @ r.fk.data.oMf[r.hkv.fids[f][-1]].rotation
        ar = r.kv.rot.T @ r.data.oMf[r.kv.fids[f][-1]].rotation
        out[i] = ar @ ah.T
    return out


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
    ap.add_argument("--device", default="cpu")
    ap.add_argument("--save-every", type=int, default=1)
    ap.add_argument("--w-motion", type=float, default=W_MOTION)
    ap.add_argument("--w-coverage", type=float, default=W_COVERAGE)
    ap.add_argument("--w-bone", type=float, default=W_BONE)
    ap.add_argument("--w-pinch", type=float, default=W_PINCH)
    ap.add_argument("--w-pos", type=float, default=W_POS)
    ap.add_argument("--w-orient", type=float, default=W_ORIENT)
    ap.add_argument("--w-sat", type=float, default=W_SAT)
    ap.add_argument("--w-posture", type=float, default=W_POSTURE)
    ap.add_argument("--u-margin", type=float, default=U_MARGIN)
    ap.add_argument("--val-frac", type=float, default=VAL_FRAC)
    ap.add_argument("--weight-decay", type=float, default=WEIGHT_DECAY)
    ap.add_argument("--dropout", type=float, default=DROPOUT)
    ap.add_argument("--hidden", type=int, default=HIDDEN)
    ap.add_argument("--layers", type=int, default=LAYERS)
    ap.add_argument("--act", default=ACT, choices=sorted(ACTS))
    ap.add_argument("--norm", default=NORM, choices=list(NORMS))
    ap.add_argument("--no-affine", dest="affine", action="store_false",
                    help="ResidualAffine 을 뺀다. 표현력은 같다 — 활성화 없는"
                         " 선형층이라 바로 뒤 Linear 에 정확히 흡수된다"
                         "(측정 8.9e-16). 재파라미터화 효과만 남는다")
    args = ap.parse_args()

    torch.manual_seed(args.seed)
    dt = torch.float32
    dev = torch.device(args.device)
    r = NetHandRetargeter(args.side, args.config, dropout=args.dropout,
                          hidden=args.hidden, layers=args.layers, act=args.act,
                          norm=args.norm)
    r.u_margin = args.u_margin
    fk = TorchKeyvectorFK(r.kv, r._iq, r.joint_names, dtype=dt).to(dev)
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


    X, AH, n_real = human_samples(r, real_q, blocks, q_flat, q_fist, lo_h, hi_h,
                                  args.random, args.random_mode, args.seed)
    X = torch.as_tensor(X, dtype=dt, device=dev)
    AH = torch.as_tensor(AH, dtype=dt, device=dev)
    d_offset = torch.as_tensor(neutral_offset(r), dtype=dt, device=dev)
    n_val = int(round(X.shape[0] * args.val_frac))
    if n_val > 0:
        vperm = torch.randperm(X.shape[0], generator=torch.Generator().manual_seed(
            args.seed))
        Xval, X = X[vperm[:n_val]], X[vperm[n_val:]]
        AHval, AH = AH[vperm[:n_val]], AH[vperm[n_val:]]
    else:
        Xval, AHval = X[:0], AH[:0]
    bank = torch.as_tensor(robot_bank(r, args.bank, args.seed), dtype=dt, device=dev)
    lo = torch.as_tensor(r.lower, dtype=dt, device=dev)
    hi = torch.as_tensor(r.upper, dtype=dt, device=dev)
    pinch_thr = PINCH_MM * 1e-3 / r.hkv.l_ref
    print("사람 %d (실측 %d + 합성 %d/%s)  로봇 bank %d  관절 %d  핀치임계 %.4f  %s"
          % (X.shape[0], n_real, args.random, args.random_mode, bank.shape[0],
             len(r.joint_names), pinch_thr, args.device), flush=True)
    print("학습 %d / 검증 %d  weight_decay %.4g  dropout %.2f  u_margin %.2f"
          "  hidden %d x %d  act %s  norm %s"
          % (X.shape[0], Xval.shape[0], args.weight_decay, args.dropout,
             args.u_margin, args.hidden, args.layers, args.act, args.norm),
          flush=True)

    os.makedirs(args.out, exist_ok=True)
    ckpt = os.path.join(args.out, "last.pt")
    net = (AffineHandNet(r.net, len(r.fingers)) if args.affine
           else r.net).to(dtype=dt, device=dev)
    net.train()
    opt = torch.optim.AdamW(net.parameters(), lr=args.lr,
                            weight_decay=args.weight_decay)

    start = 0
    if os.path.exists(ckpt):
        sd = torch.load(ckpt, map_location="cpu")
        net.load_state_dict(sd["net"])
        opt.load_state_dict(sd["opt"])
        start = int(sd["epoch"]) + 1
        print("[resume] epoch %d 부터" % start, flush=True)

    def to_kv(x):
        return fk(unit_to_joint(net(x), lo, hi, args.u_margin))

    def terms(x, ah=None):
        u = net(x)
        qj = unit_to_joint(u, lo, hi, args.u_margin)
        if args.w_orient > 0.0:
            y, ar = fk(qj, with_rot=True)
        else:
            y, ar = fk(qj), None
        zero = torch.zeros((), dtype=x.dtype, device=x.device)
        motion = zero
        orient = zero
        if ar is not None and ah is not None:
            orient = orientation_loss(ar, ah, d_offset)
        if args.w_motion > 0.0:
            d = F.normalize(torch.randn_like(x), dim=-1)
            step = MOTION_LO + torch.rand(x.shape[0], 1, 1, dtype=x.dtype,
                                          device=x.device) * (
                MOTION_HI - MOTION_LO)
            dx = d * step
            motion = motion_loss_global(dx, to_kv(x + dx) - y)
        sel = torch.randint(0, bank.shape[0],
                            (min(args.bank_batch, bank.shape[0]),))
        cover = coverage_loss(y, bank[sel]) if args.w_coverage > 0.0 else zero
        pinch = pinch_loss(x, y, pinch_thr) if args.w_pinch > 0.0 else zero
        bone = bone_loss(x, y) if args.w_bone > 0.0 else zero
        pos = position_loss(x, y) if args.w_pos > 0.0 else zero
        sat = saturation_loss(u) if args.w_sat > 0.0 else zero
        post = posture_loss(u) if args.w_posture > 0.0 else zero
        loss = (motion * args.w_motion + cover * args.w_coverage
                + pinch * args.w_pinch + bone * args.w_bone
                + pos * args.w_pos + orient * args.w_orient
                + sat * args.w_sat + post * args.w_posture)
        return loss, torch.stack([motion.detach(), cover.detach(),
                                  pinch.detach(), pos.detach(), bone.detach(),
                                  orient.detach(), sat.detach(), post.detach()])

    for epoch in range(start, args.epochs):
        perm = torch.randperm(X.shape[0])
        acc = torch.zeros(8, dtype=dt, device=dev)
        nb = 0
        for s in range(0, X.shape[0] - args.batch + 1, args.batch):
            sl = perm[s:s + args.batch]
            loss, t = terms(X[sl], None if AH is None else AH[sl])
            opt.zero_grad()
            loss.backward()
            opt.step()
            acc = acc + t
            nb += 1

        vals = (acc / max(nb, 1)).cpu().numpy()
        print("epoch %3d  motion %+.4f  cover %.4e  pinch %.4e  pos %.4e"
              "  bone %.4e  orient %.4e  sat %.3e  post %.3e"
              % (epoch, *vals), flush=True)
        if (epoch + 1) % args.save_every == 0 or epoch + 1 == args.epochs:
            if Xval.shape[0] > 0:
                vacc = torch.zeros(8, dtype=dt, device=dev)
                vl, vn = 0.0, 0
                for s in range(0, Xval.shape[0], args.batch):
                    xv = Xval[s:s + args.batch]
                    if xv.shape[0] < 2:
                        continue
                    lv, tv = terms(xv, None if AHval is None else
                                   AHval[s:s + args.batch])
                    vl += float(lv.detach()) * xv.shape[0]
                    vacc = vacc + tv * xv.shape[0]
                    vn += xv.shape[0]
                vv = (vacc / max(vn, 1)).cpu().numpy()
                print("  검증 loss %.4e  motion %+.4f  cover %.4e  pinch %.4e"
                      "  pos %.4e  bone %.4e  orient %.4e  sat %.3e  post %.3e"
                      % (vl / max(vn, 1), *vv), flush=True)
            with torch.no_grad():
                u = net(X[:min(512, X.shape[0])])
                qj = unit_to_joint(u, lo, hi, args.u_margin)
                span = (qj.max(0).values - qj.min(0).values).cpu().numpy()
                sat = (u.abs() > SAT_U).double().mean(0).cpu().numpy()
            span = np.rad2deg(span)
            dead = [(r.joint_names[i], span[i], sat[i])
                    for i in np.argsort(span)[:3]]
            print("  출력범위 최대 %.1f 최소 %.1f deg%s  최소3 %s" %
                  (span.max(), span.min(),
                   "  <= 붕괴" if span.max() < COLLAPSE_DEG else "",
                   " ".join("%s %.1f/포화%.0f%%" % (n, s, 100.0 * t)
                            for n, s, t in dead)), flush=True)
            torch.save({"net": net.state_dict(), "opt": opt.state_dict(),
                        "epoch": epoch, "cfg": vars(args),
                        "u_margin": args.u_margin, "side": args.side,
                        "config": args.config, "dropout": args.dropout,
                        "act": args.act, "norm": args.norm}, ckpt + ".tmp")
            os.replace(ckpt + ".tmp", ckpt)
            snap = {"net": net.state_dict(), "affine": args.affine,
                    "u_margin": args.u_margin, "side": args.side,
                    "config": args.config, "dropout": args.dropout,
                    "act": args.act, "norm": args.norm}
            torch.save(snap, os.path.join(args.out, "net.pt"))
            torch.save(snap, os.path.join(args.out, "ep%04d.pt" % (epoch + 1)))
    with open(os.path.join(args.out, "meta.json"), "w") as fh:
        json.dump(vars(args), fh, indent=2)


if __name__ == "__main__":
    main()
