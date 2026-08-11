#!/usr/bin/env python3
import argparse
import threading
import time

import numpy as np

from whatslab.solvers.hand import HandRetargetController
from whatslab.teleop import GloveModel, QuestModel


def _override_hand_backend(args, model):
    if args.backend is None:
        return
    if args.backend == "net" and not args.net_checkpoint:
        raise SystemExit("--hand-backend net 은 --net-checkpoint 가 필요합니다")
    kw = {"checkpoint": args.net_checkpoint} if args.backend == "net" else {}
    for side, m in model.sides.items():
        if m.retarget is None:
            continue
        m.retarget = HandRetargetController(side, args.hand_config,
                                            backend=args.backend, **kw)
    print("[hand] backend=%s%s" % (args.backend,
                                   "" if not args.net_checkpoint
                                   else " ckpt=%s" % args.net_checkpoint))


def _build_model(args, robot):
    if args.sides == "both":
        arg = robot
    else:
        arg = [robot if s == args.sides else None for s in ("left", "right")]
    if args.arm == "wrist":
        return QuestModel(arg)
    return GloveModel(arg)


class _Recorder:

    def __init__(self, robot, model, side):
        self.robot, self.model, self.side = robot, model, side
        self.rows = []

    def tick(self, now, q_map, arm_names):
        m = self.model.sides[self.side]
        raw, T = m.raw_target, m.target
        if raw is None or T is None:
            return
        have = all(n in q_map for n in arm_names)
        q = np.array([q_map[n] for n in arm_names], dtype=float) if have else None
        T_b = self.robot.clamp_reach(self.robot.to_base(T))
        pe = oe = np.nan
        ee = np.full((4, 4), np.nan)
        if q is not None:
            pe, oe = self.robot.solver.pose_error(q, T_b)
            ee = self.robot.solver.fk(q)
        c = self.model.sides[self.side].calib
        self.rows.append(dict(
            t=now,
            raw_pos=np.asarray(raw.pos, dtype=float),
            raw_quat=np.asarray(raw.quat, dtype=float),
            target=np.asarray(T, dtype=float),
            target_base=np.asarray(T_b, dtype=float),
            q=q if q is not None else np.full(len(arm_names), np.nan),
            ee_base=ee, pe=pe, oe=oe,
            W=(c._W if c is not None and c._W is not None else np.full((3, 3), np.nan)),
            enabled=float(getattr(c, "enabled", np.nan)) if c is not None else np.nan,
        ))

    def save(self, path):
        if not self.rows:
            print(f"[dump] 기록된 프레임 없음 — {path} 미생성")
            return
        out = {k: np.array([r[k] for r in self.rows]) for k in self.rows[0]}
        out["arm_joint_names"] = np.array(self.robot.arm_joint_names)
        rm = self.robot.rig.solver.reach_max
        out["reach_max"] = np.array([rm if rm else np.nan])
        out["input_reach"] = np.array([self.robot.rig.calibration.input_reach
                                       or np.nan])
        np.savez_compressed(path, **out)
        pe = out["pe"][np.isfinite(out["pe"])] * 1e3
        extra = (f"   pos 오차 mean {pe.mean():.1f} p95 {np.percentile(pe, 95):.1f} "
                 f"max {pe.max():.1f} mm" if pe.size else "")
        print(f"[dump] {len(self.rows)} 프레임 → {path}{extra}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--rig", default="rigs/nero_orca_right.yaml", help="rig config 경로")
    ap.add_argument("--side", default="right", choices=["left", "right"],
                    help="계측/전송 대상 side")
    ap.add_argument("--sides", default=None, choices=["left", "right", "both"],
                    help="IK 를 돌릴 side (기본: --side 하나만). 한 팔 rig 에 "
                         "반대쪽 컨트롤러를 물리면 도달 불가 목표에 전역탐색을 "
                         "태우고 프레임 예산을 넘긴다")
    ap.add_argument("--arm", default="controller", choices=["controller", "wrist"],
                    help="팔 소스: controller=Quest 컨트롤러(+글러브 손), wrist=Quest 핸드트래킹")
    ap.add_argument("--hand-config", default="orca_hand", help="손 리타게팅 config (hand 포함 rig)")
    ap.add_argument("--backend", default=None, choices=["kp", "dex", "net"],
                    help="rig 의 hand_solver.backend 를 덮어쓴다")
    ap.add_argument("--net-checkpoint", default=None,
                    help="--hand-backend net 의 학습 체크포인트")
    ap.add_argument("--rate", type=float, default=60.0, help="폴링/출력 주기 (Hz)")
    ap.add_argument("--viz", action="store_true", help="viser: 팔+손 메쉬 + 목표(/target)·EE(/ee) 프레임")
    ap.add_argument("--port", type=int, default=8080, help="viser 포트")
    ap.add_argument("--no-safety", action="store_true",
                    help="관절 속도 rate-limit(SafetyFilter) 미적용 — 진단용")
    ap.add_argument("--diag", action="store_true",
                    help="단계별 계측 출력 — 입력/캘리브/클램프/IK오차/불연속을 한 줄로")
    ap.add_argument("--robot", action="store_true",
                    help="실물 전송 패널을 viser 에 띄운다 (--viz 필요, 기본 미연결)")
    ap.add_argument("--can", default="can0", help="nero CAN 채널")
    ap.add_argument("--speed", type=int, default=20, help="nero 속도 퍼센트 (텔레옵은 낮게)")
    ap.add_argument("--dump-targets", metavar="PATH",
                    help="프레임별 원시입력/목표/해/오차를 npz 로 기록 — 오프라인 진단용")
    args = ap.parse_args()

    args.sides = args.sides or args.side
    if args.sides not in ("both", args.side):
        ap.error(f"--side {args.side} 가 --sides {args.sides} 에 없습니다")

    model = _build_model(args, args.rig)
    _override_hand_backend(args, model)
    robot = model.sides[args.side].robot

    print(f"[setup] sides={sorted(s for s, v in model.sides.items() if v.ik)} arm_joints={robot.arm_joint_names}")

    if not args.no_safety:
        from robot_io import attach_safety
        sf = attach_safety(model, robot, args.rate)

    model.start()

    viz = None
    bridge = None
    if args.viz:
        from whatslab.viz import RobotArmViz
        viz = RobotArmViz(robot, port=args.port)
        viz.start()
        if args.robot:
            from robot_io import build_robot_panel
            bridge = build_robot_panel(model, robot, args)
        print(f"[viz] viser: http://localhost:{args.port}")
    elif args.robot:
        print("[robot] --robot 은 --viz 와 함께 써야 합니다(연결 버튼이 viser 패널에 있음)")

    def _calib_loop():
        while True:
            try:
                line = input()
            except (EOFError, KeyboardInterrupt):
                return
            if line.strip().lower() == "r":
                print("[calib] reach 측정 시작 — 팔을 최대 범위로 뻗으세요(8초)...", flush=True)
                r = model.calibrate_reach(persist=True)
            else:
                ok = model.calibrate_yaw()
                print("[calib] yaw " + ("완료(머리연동)" if ok else "실패 — HMD/자세 신호 확인"),
                      flush=True)
    threading.Thread(target=_calib_loop, daemon=True, name="calib").start()
    print("[calib] 기준 자세로 Enter → yaw 캘리브 | 'r'+Enter → reach 캘리브. Ctrl-C 종료.")

    period, last = 1.0 / args.rate, 0.0
    next_t = time.monotonic()
    lag = 0
    arm_names = list(robot.arm_joint_names)
    arm_set = set(arm_names)
    diag = None
    if args.diag:
        from robot_io import Diag
        diag = Diag(robot, model, args.side)
    rec = _Recorder(robot, model, args.side) if args.dump_targets else None
    try:
        while True:
            now = time.monotonic()
            q = model.get_q()
            right_q = q.get(args.side) or {}
            if diag is not None:
                raw = model.sides[args.side].raw_target
                q_arm_v = (np.array([right_q[n] for n in arm_names], dtype=float)
                           if all(n in right_q for n in arm_names) else None)
                diag.tick(raw, q_arm_v, now)
            if rec is not None:
                rec.tick(now, right_q, arm_names)
            if bridge is not None:
                bridge.send(right_q)
            has_arm_q = all(n in right_q for n in arm_names)
            if viz is not None and has_arm_q:
                arm_q = [right_q[n] for n in arm_names]
                hand_names = [k for k in right_q if k not in arm_set]
                hand_q = [right_q[n] for n in hand_names]
                viz.update(arm_q, target_pose=model.sides[args.side].target,
                           hand_q=hand_q, hand_names=hand_names, timestamp=now)
            if now - last > 0.2:
                last = now
                arm_q = [round(right_q[n], 3) for n in arm_names] if has_arm_q else "--"
                tgt = "on" if model.sides[args.side].target is not None else "--"
                print(f"\r[q] arm={arm_q} hand={len(right_q) - len(arm_names)}j target={tgt}   ",
                      end="", flush=True)
            next_t += period
            rest = next_t - time.monotonic()
            if rest > 0:
                time.sleep(rest)
            else:
                lag += 1
                next_t = time.monotonic()
    except KeyboardInterrupt:
        print("\n[stop] 종료")
        if lag:
            print(f"[rate] {args.rate:g}Hz 를 못 맞춘 프레임 {lag}개 — 작업이 "
                  f"{period*1e3:.1f}ms 를 넘었다")
    finally:
        model.stop()
        if rec is not None:
            rec.save(args.dump_targets)


if __name__ == "__main__":
    main()
