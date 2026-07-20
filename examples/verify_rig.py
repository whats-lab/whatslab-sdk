#!/usr/bin/env python3
"""rig config 검증 + reach_max 실측 — 정렬(align_frames.py)을 숫자로 검증한다.

    python ~/whatslab-sdk/examples/verify_rig.py --rig rigs/nero_orca_right.yaml
    python ~/whatslab-sdk/examples/verify_rig.py --rig rigs/nero_orca_right.yaml --write   # reach_max 기록
    python ~/whatslab-sdk/examples/verify_rig.py --target 0.4 0.0 0.3   # 정준좌표 목표 추가

검증/측정 항목:
  0) reach_max 측정 : 관절 샘플 FK 로 target_ee 최대 반경 → reach_max 권장(--write)
  1) 왕복 일관성    : random q → ee_pose(정준) → solve → 오차 (config 내부 정합)
  2) --target       : 사용자 지정 정준좌표 목표
  · IK 대상 조인트(지지 체인)를 헤더에 출력. reach/워크스페이스는 reach_max 로 일원화.
"""
import argparse

import numpy as np

from whatslab.robot import RobotModel, load_rig, save_reach_max

PASS_POS_MM = 5.0        # 위치 오차 합격선 [mm]
SETTLE_TICKS = 150       # diff 백엔드 정착 틱 수


def _solve_settled(model, T_c):
    """고정 목표를 정착까지 반복 solve → (q, pos_err[m], ori_err[rad])."""
    for _ in range(SETTLE_TICKS):
        q = model.solve(T_c)
    T = model.ee_pose(q)
    pos_err = float(np.linalg.norm(T[:3, 3] - T_c[:3, 3]))
    R = T[:3, :3].T @ T_c[:3, :3]
    ori_err = float(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1)))
    return q, pos_err, ori_err


def _row(name, pos_err, ori_err, note=""):
    ok = "OK  " if pos_err * 1e3 < PASS_POS_MM else "FAIL"
    print(f"  [{ok}] {name:28s} pos={pos_err*1e3:7.2f}mm "
          f"ori={np.degrees(ori_err):6.2f}°  {note}")
    return pos_err * 1e3 < PASS_POS_MM


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--rig", default="rigs/nero_orca_right.yaml")
    ap.add_argument("--target", nargs=3, type=float, action="append", default=[],
                    metavar=("X", "Y", "Z"), help="정준좌표 목표 (반복 지정 가능)")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--samples", type=int, default=200000,
                    help="도달영역 측정용 관절 샘플 수")
    ap.add_argument("--reach-factor", type=float, default=1.01,
                    help="reach_max = 실측 최대반경 × factor")
    ap.add_argument("--write", action="store_true",
                    help="측정한 reach_max 를 rig(solver)에 기록")
    args = ap.parse_args()

    rig = load_rig(args.rig)
    model = RobotModel(rig)
    if not model.has_arm:
        raise SystemExit("[verify] arm 없는 rig — IK 검증 대상 아님")
    print(f"[verify] rig={rig.name}  backend={rig.solver.backend}  "
          f"target_ee={rig.resolve_target_ee()}")
    print(f"[verify] IK 대상 조인트 ({len(model.arm_joint_names)}): "
          f"{model.arm_joint_names}")
    results = []

    lo = np.where(np.isfinite(model.solver.model.lowerPositionLimit),
                  model.solver.model.lowerPositionLimit, -np.pi)
    hi = np.where(np.isfinite(model.solver.model.upperPositionLimit),
                  model.solver.model.upperPositionLimit, np.pi)

    # verify 는 사전 정의된 reach_max 를 따르지 않고 실측한다.
    # (IK 검증도 reach 클램프 없이 순수 정합을 본다)
    rig.solver.reach_max = None

    # ── 0) reach_max 측정 (target_ee 최대 반경, 베이스 기준) ──
    # 관절공간을 무작위 샘플해 target_ee 위치의 최대 반경을 잰다. FK 만 사용.
    # 도달영역은 박스가 아니라 껍질이라 바운딩박스는 무의미 → 반경만 측정한다.
    print("\n0) reach_max 측정 (target_ee 최대 반경)")
    rng0 = np.random.default_rng(args.seed)
    Q = lo + (hi - lo) * rng0.random((args.samples, model.solver.nq))
    r_max = max(float(np.linalg.norm(model.solver.fk(q)[:3, 3])) for q in Q)
    reach_reco = round(r_max * args.reach_factor, 4)
    print(f"  샘플 {args.samples}개  최대 반경 {r_max:.4f}m")
    print(f"  → 측정 reach_max = {reach_reco} (={args.reach_factor:.2f}×)")
    if args.write:
        save_reach_max(rig, reach_reco)
        print(f"  [기록] rig.solver.reach_max = {reach_reco} → {rig.path}")
    else:
        print("  (--write 로 rig(solver)에 기록)")

    # ── 1) 왕복 일관성 (calibration 끄고 순수 정합. reach 는 위에서 이미 해제) ──
    print("\n1) 왕복 일관성 (random q → ee_pose → solve)")
    cal_enabled = rig.calibration.enabled
    rig.calibration.enabled = False
    rng = np.random.default_rng(args.seed)
    for i in range(5):
        q_true = lo + (hi - lo) * (0.2 + 0.6 * rng.random(model.solver.nq))
        T_c = model.ee_pose(q_true)
        model.sync_state(q_true + 0.2 * rng.standard_normal(model.solver.nq))
        _, pe, oe = _solve_settled(model, T_c)
        results.append(_row(f"roundtrip #{i}", pe, oe))

    rig.calibration.enabled = cal_enabled

    # ── 2) 사용자 지정 목표 (정준좌표) ──
    if args.target:
        print("\n2) 사용자 지정 목표 (정준좌표)")
        for t in args.target:
            T_c = np.eye(4)
            T_c[:3, 3] = np.asarray(t, dtype=float)
            model.sync_state(np.zeros(model.solver.nq))
            _, pe, oe = _solve_settled(model, T_c)
            results.append(_row(f"target {t}", pe, oe))

    n_ok = sum(results)
    print(f"\n[verify] {n_ok}/{len(results)} 통과 "
          f"(합격선 pos < {PASS_POS_MM}mm)")
    if n_ok < len(results):
        print("[verify] FAIL 점검: ① 목표가 reach 안인지 "
              "② axis_align/attach 정렬(align_frames.py)")
        raise SystemExit(1)


if __name__ == "__main__":
    main()
