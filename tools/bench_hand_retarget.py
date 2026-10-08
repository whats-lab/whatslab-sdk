#!/usr/bin/env python3
"""OpenXR(글러브) 손 리타게팅을 목표 구성과 무관한 물리 지표로 잰다.

입력은 OpenXR 26관절 시퀀스다 — npz(xr: (N, 26, 8) [qx qy qz qw px py pz r] 손목 공간, side)
또는 Quest WebXR 녹화 json(frames[{t, left: 25x7, right: 25x7}], [px py pz qx qy qz qw]).
지표(전부 사람·로봇 각자의 해부학 프레임에서 잰다 — 리타게터의 목표를 정답으로 쓰지 않는다):
  open   펴진 손 프레임에서 |로봇 엄지-손가락 끝 거리 − s·사람 거리| 평균 (mm)
  pinch  사람 엄지-손가락 끝 거리 < --pinch-mm(기본 20) 인 프레임의 로봇 끝 거리 평균 (mm)
         (OpenXR 끝 관절은 손끝 표면이라 사람 패드 접촉도 10~15mm 로 남는다)
  bone   손바닥 프레임 기준 뼈 방향각 오차 평균 (°) — 엄지 MCP→IP→끝, 손가락 MCP→PIP→끝
  |dq|   연속 프레임 관절 변화 최댓값의 p95 (rad)
  limit  관절범위 양 끝 2% 안에 있는 (프레임, 관절) 비율
  ms     호출 1회 비용, 2손 = 같은 틱의 왼손+오른손 합 (같은 녹화의 양손 — Quest json,
         또는 이름이 *_left.npz / *_right.npz 로 짝인 npz)
s = 로봇 손 길이(q=0 손목→중지 끝) / 사람 손 길이(시퀀스의 손목→중지 끝 p95).
"""
import argparse
import glob
import json
import os
import time

import numpy as np
import pinocchio as pin
from scipy.spatial.transform import Rotation

from whatslab.solvers.hand import XRHandRetargeter, mirror_xr_joints
from whatslab.solvers.hand.xr_input import palm_frame, xr_wrist_positions

FINGERS = ("thumb", "index", "middle", "ring", "pinky")
XR_TIP = (5, 10, 15, 20, 25)
XR_BONES = ((3, 4, 5), (7, 8, 10), (12, 13, 15), (17, 18, 20), (22, 23, 25))
XR_BASE = (3, 7, 12, 17, 22)
XR_PALM = (1, 7, 12, 22)
OPEN_FRAC = 0.9
LIMIT_FRAC = 0.02


def quest_to_xr(a25):
    a = np.asarray(a25, dtype=float).reshape(25, 7)
    Rw = Rotation.from_quat(a[0, 3:]).as_matrix()
    out = np.zeros((26, 8))
    out[1:, :4] = Rotation.from_matrix(Rw.T @ Rotation.from_quat(a[:, 3:]).as_matrix()).as_quat()
    out[1:, 4:7] = (a[:, :3] - a[0, :3]) @ Rw
    out[0, :4] = out[11, :4]
    out[0, 4:7] = 0.5 * (out[11, 4:7] + out[12, 4:7])
    return out


def load_sequences(paths):
    seqs = []
    for p in paths:
        tag = os.path.splitext(os.path.basename(p))[0]
        if p.endswith(".npz"):
            z = np.load(p, allow_pickle=False)
            side = str(z["side"])
            sq = {"tag": tag, "side": side, "xr": np.asarray(z["xr"], float)}
            if tag.endswith("_" + side):
                sq["pair"] = tag[:-len(side) - 1]
            seqs.append(sq)
            continue
        with open(p) as f:
            d = json.load(f)
        for side in ("left", "right"):
            xr = [quest_to_xr(fr[side]) for fr in d["frames"] if fr.get(side)]
            if len(xr) > 1:
                seqs.append({"tag": f"{tag}:{side}", "side": side, "xr": np.array(xr),
                             "pair": tag})
    return seqs


class RobotProbe:

    def __init__(self, eng: XRHandRetargeter):
        self.eng = eng
        self.model = pin.buildModelFromUrdf(eng.urdf_path)
        self.data = self.model.createData()
        names = eng.robot_joint_names
        self._qidx = np.array([self.model.idx_qs[self.model.getJointId(n)] for n in names])
        cfg = eng.config.retargeting
        self.tips = [self.model.getFrameId(n) for n in cfg["finger_tip_link_names"]]
        self.wrist = self.model.getFrameId(cfg["wrist_link_name"])
        self.chains = [self._chain(t, k == 0) for k, t in enumerate(self.tips)]
        self._fk(eng.full_qpos(np.clip(np.zeros(len(eng.joint_names)),
                                       eng.joint_limits[:, 0], eng.joint_limits[:, 1])))
        palm = [self.model.getFrameId(n) for n in eng.config.palm_links]
        self.R_palm = palm_frame(self._pos(self.wrist), *(self._pos(f) for f in palm),
                                 side=eng.config.side)
        self.length = float(np.linalg.norm(self._pos(self.tips[2]) - self._pos(self.wrist)))

    def _pos(self, fid):
        return self.data.oMf[fid].translation.copy()

    def _chain(self, fid, thumb):
        target = set(self.eng.joint_names)
        j, joints = self.model.frames[fid].parent, []
        while j > 0:
            if self.model.names[j] in target:
                joints.append(j)
            j = self.model.parents[j]
        joints = joints[::-1]
        return joints[-2:] if thumb else joints[1:3]

    def _fk(self, q_full):
        q = pin.neutral(self.model)
        q[self._qidx] = q_full
        pin.framesForwardKinematics(self.model, self.data, q)

    def points(self, q):
        self._fk(self.eng.full_qpos(q))
        tips = np.array([self.data.oMf[f].translation for f in self.tips])
        bones = [np.array([self.data.oMi[j].translation for j in ch] + [tips[k]])
                 for k, ch in enumerate(self.chains)]
        return tips, bones


def bone_angles(h_pts, r_bones, R_h, R_r):
    out = []
    for k, idx in enumerate(XR_BONES):
        hb = h_pts[list(idx)] @ R_h
        rb = r_bones[k] @ R_r
        for a in range(2):
            u, v = hb[a + 1] - hb[a], rb[a + 1] - rb[a]
            nu, nv = np.linalg.norm(u), np.linalg.norm(v)
            if nu > 1e-6 and nv > 1e-6:
                out.append(np.degrees(np.arccos(np.clip(u @ v / (nu * nv), -1.0, 1.0))))
    return out


def run_config(name, seqs, mirror, overrides, pinch_mm):
    acc = {k: [] for k in ("open", "pinch", "pinch_err", "bone", "dq", "limit", "ms")}
    n_frames = 0
    eng = XRHandRetargeter(name, **overrides)
    probe = RobotProbe(eng)
    lo, hi = eng.joint_limits[:, 0], eng.joint_limits[:, 1]
    band = LIMIT_FRAC * (hi - lo)
    side = eng.config.side
    for sq in seqs:
        if sq["side"] == side:
            xr = sq["xr"]
        elif mirror:
            xr = np.array([mirror_xr_joints(x) for x in sq["xr"]])
        else:
            continue
        eng.reset()
        H = np.array([xr_wrist_positions(x) for x in xr])
        s = probe.length / np.percentile(np.linalg.norm(H[:, 15] - H[:, 1], axis=1), 95)
        ext = np.linalg.norm(H[:, list(XR_TIP)] - H[:, list(XR_BASE)], axis=2)
        ext_ref = np.percentile(ext, 95, axis=0)
        prev = None
        for i, x in enumerate(xr):
            t0 = time.perf_counter()
            q = eng.compute(x)
            acc["ms"].append((time.perf_counter() - t0) * 1e3)
            n_frames += 1
            if prev is not None:
                acc["dq"].append(float(np.abs(q - prev).max()))
            prev = q
            acc["limit"].append(float(np.mean((q < lo + band) | (q > hi - band))))
            tips, bones = probe.points(q)
            h = H[i]
            R_h = palm_frame(*(h[j] for j in XR_PALM), side=side)
            acc["bone"].extend(bone_angles(h, bones, R_h, probe.R_palm))
            hd = np.linalg.norm(h[list(XR_TIP[1:])] - h[XR_TIP[0]], axis=1) * 1e3
            rd = np.linalg.norm(tips[1:] - tips[0], axis=1) * 1e3
            if np.all(ext[i] >= OPEN_FRAC * ext_ref):
                acc["open"].append(float(np.mean(np.abs(rd - s * hd))))
            for f in np.flatnonzero(hd < pinch_mm):
                acc["pinch"].append(float(rd[f]))
                acc["pinch_err"].append(float(abs(rd[f] - s * hd[f])))
    return eng, acc, n_frames


def two_hand_latency(name_l, name_r, seqs, overrides):
    eng = {"left": XRHandRetargeter(name_l, **overrides),
           "right": XRHandRetargeter(name_r, **overrides)}
    pairs = {}
    for sq in seqs:
        if "pair" in sq:
            pairs.setdefault(sq["pair"], {})[sq["side"]] = sq["xr"]
    ms = []
    for p in pairs.values():
        if len(p) != 2:
            continue
        for e in eng.values():
            e.reset()
        for xl, xrr in zip(p["left"], p["right"]):
            t0 = time.perf_counter()
            eng["left"].compute(xl)
            eng["right"].compute(xrr)
            ms.append((time.perf_counter() - t0) * 1e3)
    return np.array(ms)


def _pct(v, q):
    return float(np.percentile(v, q)) if len(v) else float("nan")


def _mean(v):
    return float(np.mean(v)) if len(v) else float("nan")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("data", nargs="+", help="npz/json 파일 또는 디렉터리")
    ap.add_argument("--configs", nargs="+",
                    default=["orca_right", "orca_left", "dg5f_right", "dg5f_left"])
    ap.add_argument("--mirror", action="store_true",
                    help="반대손 시퀀스를 거울상으로 바꿔 같이 쓴다")
    ap.add_argument("--set", nargs="*", default=[], metavar="KEY=VAL",
                    help="dex RetargetingConfig 덮어쓰기 (예: low_pass_alpha=0.3)")
    ap.add_argument("--pinch-mm", type=float, default=20.0,
                    help="핀치 프레임 판정: 사람 엄지-손가락 끝 거리 상한 (mm)")
    ap.add_argument("--max-evals", type=int, help="SLSQP 평가 상한 덮어쓰기 (0 = 무제한)")
    ap.add_argument("--json", help="결과를 json 으로 저장")
    args = ap.parse_args()

    paths = []
    for p in args.data:
        paths += sorted(glob.glob(os.path.join(p, "*.npz")) + glob.glob(os.path.join(p, "*.json"))
                        ) if os.path.isdir(p) else [p]
    seqs = load_sequences(paths)
    overrides = {}
    for kv in args.set:
        k, v = kv.split("=", 1)
        overrides[k] = float(v)
    if args.max_evals is not None:
        overrides["max_evals"] = args.max_evals
    print(f"시퀀스 {len(seqs)}개, 프레임 {sum(len(s['xr']) for s in seqs)}")
    hdr = "%-14s %7s %6s %9s %8s %8s %7s %7s %7s %7s %7s" % (
        "config", "frames", "open", "pinch(n)", "pinchErr", "bone°", "|dq|95", "limit",
        "ms50", "ms99", "2손99")
    print(hdr)
    out = {}
    for name in args.configs:
        eng, acc, n = run_config(name, seqs, args.mirror, overrides, args.pinch_mm)
        row = {
            "frames": n, "open_mm": _mean(acc["open"]), "open_n": len(acc["open"]),
            "pinch_mm": _mean(acc["pinch"]), "pinch_p90_mm": _pct(acc["pinch"], 90),
            "pinch_n": len(acc["pinch"]), "pinch_err_mm": _mean(acc["pinch_err"]),
            "bone_deg": _mean(acc["bone"]), "dq_p95": _pct(acc["dq"], 95),
            "limit_ratio": _mean(acc["limit"]), "ms_p50": _pct(acc["ms"], 50),
            "ms_p99": _pct(acc["ms"], 99),
        }
        other = name.replace("_right", "_left") if name.endswith("_right") else \
            name.replace("_left", "_right")
        two = two_hand_latency(other if name.endswith("_right") else name,
                               name if name.endswith("_right") else other, seqs, overrides)
        row["two_hand_ms_p50"] = _pct(two, 50)
        row["two_hand_ms_p99"] = _pct(two, 99)
        out[name] = row
        print("%-14s %7d %6.1f %5.1f(%3d) %8.1f %8.1f %7.3f %7.3f %7.2f %7.2f %7.2f" % (
            name, n, row["open_mm"], row["pinch_mm"], row["pinch_n"], row["pinch_err_mm"],
            row["bone_deg"], row["dq_p95"], row["limit_ratio"], row["ms_p50"], row["ms_p99"],
            row["two_hand_ms_p99"]))
    if args.json:
        with open(args.json, "w") as f:
            json.dump(out, f, indent=2)


if __name__ == "__main__":
    main()
