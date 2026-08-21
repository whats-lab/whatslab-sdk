#!/usr/bin/env python3
"""BHaM 키포인트에 맞춰 URDF 의 origin rpy 와 axis 를 피팅한다 (origin xyz 는 고정)."""
import argparse
import glob
import importlib.util
import os
import re
import tempfile
import time
from concurrent.futures import ProcessPoolExecutor

import numpy as np
import pinocchio as pin
from scipy.optimize import least_squares

_HERE = os.path.dirname(os.path.abspath(__file__))
_spec = importlib.util.spec_from_file_location(
    "rtd", os.path.join(_HERE, "retarget_hand_dataset.py"))
_rtd = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_rtd)

JOINT_RE = re.compile(r'<joint\s+name="([^"]+)"\s+type="([^"]+)"\s*>(.*?)</joint>',
                      re.S)
ORIGIN_RPY_RE = re.compile(r'(<origin\s+xyz="[^"]*"\s+rpy=")([^"]*)(")')
AXIS_RE = re.compile(r'(<axis\s+xyz=")([^"]*)(")')
_G = {}


def parse(text):
    out = []
    for m in JOINT_RE.finditer(text):
        name, typ, body = m.group(1), m.group(2), m.group(3)
        if typ != "revolute":
            continue
        om = ORIGIN_RPY_RE.search(body)
        am = AXIS_RE.search(body)
        if om is None or am is None:
            continue
        rpy = np.array([float(v) for v in om.group(2).split()])
        ax = np.array([float(v) for v in am.group(2).split()])
        out.append((name, m.span(), rpy, ax / np.linalg.norm(ax)))
    return out


def emit(text, joints, rpy_new, ax_new):
    parts, prev = [], 0
    for k, (_n, (s, e), _r, _a) in enumerate(joints):
        blk = text[s:e]
        blk = ORIGIN_RPY_RE.sub(
            lambda m: m.group(1) + " ".join("%.6f" % v for v in rpy_new[k])
            + m.group(3), blk, count=1)
        blk = AXIS_RE.sub(
            lambda m: m.group(1) + " ".join("%.6f" % v for v in ax_new[k])
            + m.group(3), blk, count=1)
        parts.append(text[prev:s])
        parts.append(blk)
        prev = e
    parts.append(text[prev:])
    return "".join(parts)


def unpack(x, joints):
    n = len(joints)
    d = np.asarray(x).reshape(n, 6)
    rpy = np.array([j[2] for j in joints]) + d[:, :3]
    ax = np.empty((n, 3))
    for k, j in enumerate(joints):
        a = pin.exp3(d[k, 3:]) @ j[3]
        ax[k] = a / np.linalg.norm(a)
    return rpy, ax


def _init(text, joints, side, kps, inner, tmpd, sconst, rnorm):
    _G["text"] = text
    _G["joints"] = joints
    _G["side"] = side
    _G["kps"] = kps
    _G["inner"] = inner
    _G["path"] = os.path.join(tmpd, "w%d.urdf" % os.getpid())
    _G["sconst"] = sconst
    _G["rnorm"] = rnorm


def _eval(task):
    x, Q0, S0 = task
    rpy, ax = unpack(x, _G["joints"])
    with open(_G["path"], "w") as fh:
        fh.write(emit(_G["text"], _G["joints"], rpy, ax))
    rt = _rtd.Retargeter(_G["side"], iters=_G["inner"], urdf_path=_G["path"],
                         scale_const=_G["sconst"])
    kps = _G["kps"]
    out = np.empty(len(kps))
    Q = np.empty((len(kps), rt.nq))
    for i, kp in enumerate(kps):
        q0 = None if Q0 is None else Q0[i]
        R0 = None if S0 is None else S0[i][0]
        t0 = None if S0 is None else S0[i][1]
        q, R, t, s, rms = rt.solve(kp, q0, R0, t0, None)
        out[i] = rms / _G["rnorm"]
        Q[i] = q
    return out, Q


def sample_frames(dirs, per_file, seed):
    rng = np.random.default_rng(seed)
    files = []
    for d in dirs:
        files += sorted(glob.glob(os.path.join(d, "*.npz")))
    kps = []
    for f in files:
        z = np.load(f)
        kp = z["keypoints"]
        if len(kp) < per_file:
            continue
        idx = rng.choice(len(kp), per_file, replace=False)
        kps.append(kp[np.sort(idx)])
    return np.concatenate(kps) if kps else np.zeros((0, 21, 3))


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--urdf", required=True)
    ap.add_argument("--dirs", nargs="+", required=True)
    ap.add_argument("--side", default="left")
    ap.add_argument("--per-file", type=int, default=2)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--iters", type=int, default=30)
    ap.add_argument("--inner", type=int, default=12)
    ap.add_argument("--workers", type=int, default=24)
    ap.add_argument("--rpy-max", type=float, default=0.6)
    ap.add_argument("--axis-max", type=float, default=0.6)
    ap.add_argument("--reg", type=float, default=0.02)
    ap.add_argument("--step", type=float, default=0.02)
    ap.add_argument("--freeze", nargs="*", default=[])
    ap.add_argument("--out", default=None)
    a = ap.parse_args()

    text = open(a.urdf).read()
    joints = parse(text)
    n = len(joints)
    kps = sample_frames(a.dirs, a.per_file, a.seed)
    print("관절 %d개 / 샘플 프레임 %d / worker %d" % (n, len(kps), a.workers),
          flush=True)
    if len(kps) == 0:
        raise SystemExit("샘플 없음")

    free = np.ones(n * 6, dtype=bool)
    for k, j in enumerate(joints):
        if any(f in j[0] for f in a.freeze):
            free[k * 6:k * 6 + 6] = False
    fi = np.where(free)[0]
    print("자유 파라미터 %d / 고정 관절 %s" % (
        len(fi), [joints[k][0] for k in range(n)
                  if not free[k * 6]]), flush=True)

    probe = _rtd.Retargeter(a.side, iters=1, urdf_path=a.urdf)
    rnorm = probe.ref_len
    chain = np.array([sum(np.linalg.norm(k[b] - k[a]) for a, b in probe.ref_seg)
                      for k in kps])
    norm_c = float(chain.max())
    sconst = rnorm / norm_c
    print("정규화 상수: BHaM 손목→중지 사슬 최대 %.1fmm / 우리 %.1fmm → scale %.4f"
          % (1000 * norm_c, 1000 * rnorm, sconst), flush=True)

    tmpd = tempfile.mkdtemp(prefix="fiturdf_")
    ex = ProcessPoolExecutor(max_workers=a.workers, initializer=_init,
                             initargs=(text, joints, a.side, kps, a.inner, tmpd,
                                       sconst, rnorm))
    st = {"Q": None, "n": 0, "best": None, "t0": time.time()}

    def resid(x):
        r, Q = list(ex.map(_eval, [(x, st["Q"], None)]))[0]
        st["n"] += 1
        v = np.concatenate([r, a.reg * x])
        c = float(v @ v)
        print("    f#%d 잔차 %.3f%% cost %.5f (%.0fs)" % (
            st["n"], 100 * r.mean(), c, time.time() - st["t0"]), flush=True)
        if st["best"] is None or c < st["best"] - 1e-12:
            st["best"] = c
            st["Q"] = Q
            print("  it %3d  잔차 %.3f%%  cost %.5f  (%.0fs)" % (
                st["n"], 100 * r.mean(), c, time.time() - st["t0"]), flush=True)
        return v

    def jac(x):
        tj = time.time()
        f0 = resid(x)
        tasks = []
        for i in fi:
            xp = x.copy()
            xp[i] += a.step
            tasks.append((xp, st["Q"], None))
        outs = list(ex.map(_eval, tasks, chunksize=1))
        print("    jac %.0fs" % (time.time() - tj), flush=True)
        J = np.zeros((f0.size, x.size))
        for c, i in enumerate(fi):
            fp = np.concatenate([outs[c][0], a.reg * tasks[c][0]])
            J[:, i] = (fp - f0) / a.step
        return J

    x0 = np.zeros(n * 6)
    lo = np.tile([-a.rpy_max] * 3 + [-a.axis_max] * 3, n)
    hi = np.tile([a.rpy_max] * 3 + [a.axis_max] * 3, n)
    lo[~free] = -1e-9
    hi[~free] = 1e-9

    r0 = resid(x0)
    base = 100 * r0[:len(kps)].mean()
    print("초기 잔차 %.3f%%" % base, flush=True)
    res = least_squares(resid, x0, jac=jac, bounds=(lo, hi), method="trf",
                        x_scale=0.05, max_nfev=a.iters, verbose=0)
    rf = res.fun[:len(kps)]
    print("최종 잔차 %.3f%%  (초기 %.3f%%, %.0fs)" % (
        100 * rf.mean(), base, time.time() - st["t0"]), flush=True)

    rpy, ax = unpack(res.x, joints)
    print("\n%-26s %11s %11s" % ("관절", "rpy변화[°]", "axis변화[°]"))
    for k, j in enumerate(joints):
        print("%-26s %11.2f %11.2f" % (
            j[0], np.degrees(np.linalg.norm(rpy[k] - j[2])),
            np.degrees(np.arccos(np.clip(ax[k] @ j[3], -1, 1)))))
    if a.out:
        with open(a.out, "w") as fh:
            fh.write(emit(text, joints, rpy, ax))
        np.savez(os.path.splitext(a.out)[0] + "_fit.npz",
                 names=np.array([j[0] for j in joints]), rpy=rpy, axis=ax,
                 rpy0=np.array([j[2] for j in joints]),
                 axis0=np.array([j[3] for j in joints]),
                 resid0=base / 100.0, resid=float(rf.mean()))
        print("\n기록 %s" % a.out, flush=True)
    ex.shutdown()


main()
