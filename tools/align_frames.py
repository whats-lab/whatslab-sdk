#!/usr/bin/env python3
"""정준좌표 정렬 튜너 (viser) — 2단계 워크플로우로 config 를 정립한다.

    pip install -e ~/whatslab-sdk[viz]          # 튜너 의존 (viser, trimesh)

1단계 `robot` — 로봇(arm 이든 hand 든) 하나를 정준 축에 정렬 → axis_align 확정
    python ~/whatslab-sdk/tools/align_frames.py robot --robot robots/nero.yaml
    python ~/whatslab-sdk/tools/align_frames.py robot --robot robots/orca_right.yaml
    # (config 없이) python ... robot --urdf orca_hand/urdf/right.urdf
    # --robot 이면 URDF 와 현재 axis_align 을 config 에서 읽어 슬라이더 초기값으로.
    # 슬라이더: axis_align xyz+rpy. 방향(x=앞, z=위)과 위치(장착 기준점=원점)를
    # 모두 원점에 정합 → 2단계 attach 가 자연히 거의 항등이 된다.
    # [print yaml] → robot yaml 의 axis_align 에 붙여넣기.

2단계 `attach` — 팔 TCP 에 손을 거치해 보정 → ee.origin + attach 확정
    python ~/whatslab-sdk/tools/align_frames.py attach --rig rigs/nero_orca_right.yaml
    # (rig 없이) python ... attach --arm-robot robots/nero.yaml --hand-robot robots/orca_right.yaml
    # 팔·손의 axis_align(1단계 확정, xyz+rpy 전체)이 적용된 채 고정. 슬라이더: ee.origin, attach.
    # 손이 플랜지에 올바로 앉으면 [print yaml] → arm yaml(ee) + rig yaml(attach).

3단계 `ee` — target_ee(IK 제어) 프레임을 정준 tool 규약에 정렬 → ee_align 확정
    python ~/whatslab-sdk/tools/align_frames.py ee --rig rigs/nero_orca_right.yaml
    # 팔+손 메쉬는 확정 config 로 고정. 슬라이더(rpy)로 target_ee 프레임 축만 회전.
    # 손끝→x(빨강,앞), 손바닥→-z 가 되게 맞추고 [print yaml] → hand yaml(ee_align).
    # 메쉬는 안 움직인다 — IK 가 제어하는 프레임 축의 방향만 바꾼다(글러브 remap 과 분리).

실행 후 http://localhost:8080 을 브라우저로 연다.
"""
import argparse
import os
import time

import numpy as np
import pinocchio as pin
import viser

from whatslab.paths import models_root
from whatslab.robot import load_rig, load_robot
from whatslab.robot.config import origin_to_T
from whatslab.viz.scene import URDFScene       # 메쉬/스켈레톤 렌더 (중복 제거 — 단일 출처)


def _wxyz_from_mat(R: np.ndarray):
    q = pin.Quaternion(np.asarray(R, dtype=float))     # SVD 없는 변환
    return (float(q.w), float(q.x), float(q.y), float(q.z))


DEG = 180.0 / np.pi


def _fmt(v):
    return "[" + ", ".join(f"{x:.4f}" for x in v) + "]"


def _v(sls, scale=1.0):
    out = []
    for sl in sls:
        x = sl.value
        out.append(0.0 if (x is None or not np.isfinite(x)) else float(x))
    return np.array(out) * scale


def _setup_server(port: int) -> viser.ViserServer:
    server = viser.ViserServer(port=port)
    server.scene.add_frame("/canonical", axes_length=0.3, axes_radius=0.008)
    server.scene.add_label("/canonical/x_label", "x=앞", position=(0.35, 0, 0))
    server.scene.add_label("/canonical/z_label", "z=위", position=(0, 0, 0.35))
    print(f"[align] 브라우저: http://localhost:{port}")
    return server


def _resolve(path: str) -> str:
    return path if os.path.isabs(path) else os.path.join(models_root(), path)


def _sl_xyz(gui, v, r=0.3, step=0.005):
    return [gui.add_slider(f"{a} [m]", -r, r, step, float(x))
            for a, x in zip("xyz", v)]


def _sl_rpy(gui, v):
    return [gui.add_slider(f"{a} [°]", -180, 180, 5,
                           float(np.clip(x * DEG, -180, 180)))
            for a, x in zip(("roll", "pitch", "yaw"), v)]


# ─────────────────────────── 1단계: robot 정준 정렬 ───────────────────────────
def mode_robot(args):
    # --robot(config) 이면 URDF·현재 axis_align 을 config 에서 가져와 초기값으로.
    init_xyz, init_rpy = np.zeros(3), np.zeros(3)
    if args.robot:
        spec = load_robot(args.robot)
        urdf = spec.urdf_abspath()
        init_xyz = np.array(spec.axis_align.xyz, dtype=float)
        init_rpy = np.array(spec.axis_align.rpy, dtype=float)
        print(f"[robot] config={args.robot} (name={spec.name}, kind={spec.kind})")
    elif args.urdf:
        urdf = _resolve(args.urdf)
    else:
        raise SystemExit("[robot] --robot(config) 또는 --urdf 필요")
    mesh_dir = args.mesh_dir or models_root()
    server = _setup_server(args.port)
    scene = URDFScene(server, urdf, mesh_dir, "/robot")
    print(f"[robot] {urdf}  렌더={'메쉬' if scene.mesh_mode else '스켈레톤'}")
    print("[robot] 목표: x=앞(빨강)을 보고 z=위(파랑)로 바로 서게 → print yaml")

    with server.gui.add_folder("axis_align (robot yaml)"):
        s_ax = _sl_xyz(server.gui, init_xyz, r=0.5)
        s_ar = _sl_rpy(server.gui, init_rpy)
    btn = server.gui.add_button("print yaml")

    def _snippet():
        return (f"\n# ── robot yaml ──\n"
                f"axis_align: {{xyz: {_fmt(_v(s_ax))}, "
                f"rpy: {_fmt(_v(s_ar, 1 / DEG))}}}\n")

    @btn.on_click
    def _(_evt):
        print(_snippet())

    try:
        while True:
            scene.set_root(origin_to_T(_v(s_ax), _v(s_ar, 1 / DEG)))
            scene.fk(np.zeros(scene.model.nq))
            time.sleep(0.05)
    except KeyboardInterrupt:
        print(_snippet())


# ─────────────────────────── 2단계: attach 보정 ──────────────────────────────
def mode_attach(args):
    init_eeo_xyz = np.zeros(3)
    init_eeo_rpy = np.zeros(3)
    init_at_xyz = np.zeros(3)
    init_at_rpy = np.zeros(3)
    T_arm_align = np.eye(4)          # 1단계 확정 axis_align (xyz+rpy 전체)
    T_hand_align = np.eye(4)
    ee_parent = args.ee_parent
    if args.rig:
        rig = load_rig(args.rig)
        if rig.arm is None or rig.hand is None:
            raise SystemExit("[attach] rig 에 arm+hand 둘 다 필요")
        arm_urdf = rig.arm.urdf_abspath()
        hand_urdf = rig.hand.urdf_abspath()
        ee_parent = rig.arm.ee_parent
        T_arm_align = rig.arm.axis_align.T           # xyz+rpy 전체 반영
        T_hand_align = rig.hand.axis_align.T
        init_eeo_xyz = np.array(rig.arm.ee_origin.xyz, dtype=float)
        init_eeo_rpy = np.array(rig.arm.ee_origin.rpy, dtype=float)
        init_at_xyz = np.array(rig.attach.xyz, dtype=float)
        init_at_rpy = np.array(rig.attach.rpy, dtype=float)
    else:
        # rig 없이 개별 robot config(권장) 또는 raw URDF 도 허용
        if args.arm_robot and args.hand_robot:
            arm_spec, hand_spec = load_robot(args.arm_robot), load_robot(args.hand_robot)
            arm_urdf, hand_urdf = arm_spec.urdf_abspath(), hand_spec.urdf_abspath()
            ee_parent = arm_spec.ee_parent
            T_arm_align, T_hand_align = arm_spec.axis_align.T, hand_spec.axis_align.T
            init_eeo_xyz = np.array(arm_spec.ee_origin.xyz, dtype=float)
            init_eeo_rpy = np.array(arm_spec.ee_origin.rpy, dtype=float)
        elif args.urdf and args.hand_urdf:
            arm_urdf = _resolve(args.urdf)
            hand_urdf = _resolve(args.hand_urdf)
        else:
            raise SystemExit("[attach] --rig / --arm-robot+--hand-robot / "
                             "--urdf+--hand-urdf 중 하나 필요")

    mesh_dir = args.mesh_dir or models_root()
    server = _setup_server(args.port)
    arm = URDFScene(server, arm_urdf, mesh_dir, "/robot")
    hand = URDFScene(server, hand_urdf, mesh_dir, "/robot/hand")
    print(f"[attach] arm={arm_urdf}")
    print(f"[attach] hand={hand_urdf}  (@ {ee_parent})")
    print("[attach] 팔의 axis_align 은 1단계 확정값으로 고정 — ee.origin/attach 만 조정")

    with server.gui.add_folder("ee.origin (arm yaml)"):
        s_eo_x = _sl_xyz(server.gui, init_eeo_xyz)
        s_eo_r = _sl_rpy(server.gui, init_eeo_rpy)
    with server.gui.add_folder("attach (rig yaml)"):
        s_at_x = _sl_xyz(server.gui, init_at_xyz)
        s_at_r = _sl_rpy(server.gui, init_at_rpy)
    btn = server.gui.add_button("print yaml")

    def _snippet():
        return ("\n# ── arm yaml ──\n"
                f"ee:\n  parent: {ee_parent}\n"
                f"  origin: {{xyz: {_fmt(_v(s_eo_x))}, rpy: {_fmt(_v(s_eo_r, 1/DEG))}}}\n"
                "# ── rig yaml ──\n"
                f"attach: {{xyz: {_fmt(_v(s_at_x))}, rpy: {_fmt(_v(s_at_r, 1/DEG))}}}\n")

    @btn.on_click
    def _(_evt):
        print(_snippet())

    try:
        while True:
            arm.set_root(T_arm_align)                # 1단계 확정 axis_align (고정)
            arm.fk(np.zeros(arm.model.nq))
            # 손 루트(팔베이스 기준) = FK(ee.parent) ∘ ee.origin ∘ attach ∘ 손 axis_align
            T_h = (arm.frame_pose(ee_parent)
                   @ origin_to_T(_v(s_eo_x), _v(s_eo_r, 1 / DEG))
                   @ origin_to_T(_v(s_at_x), _v(s_at_r, 1 / DEG))
                   @ T_hand_align)
            hand.set_root(T_h)                        # /robot 자식 → 팔 정렬 상속
            hand.fk(np.zeros(hand.model.nq))
            time.sleep(0.05)
    except KeyboardInterrupt:
        print(_snippet())


# ─────────────────── 3단계: target_ee 프레임 정렬 (ee_align) ───────────────────
def mode_ee(args):
    """팔+손을 확정 config 로 고정 렌더하고, target_ee(IK 제어) 프레임 축만
    rpy 슬라이더로 회전 → hand yaml ee_align 확정. 메쉬는 움직이지 않는다."""
    rig = load_rig(args.rig)
    if rig.arm is None or rig.hand is None:
        raise SystemExit("[ee] rig 에 arm+hand 둘 다 필요")
    arm_urdf = rig.arm.urdf_abspath()
    hand_urdf = rig.hand.urdf_abspath()
    ee_parent = rig.arm.ee_parent
    target_ee = rig.resolve_target_ee()
    T_arm_align = rig.arm.axis_align.T
    T_hand_align = rig.hand.axis_align.T
    T_eeo = rig.arm.ee_origin.T
    T_at = rig.attach.T
    init_rpy = np.array(rig.hand.ee_align.rpy, dtype=float)

    mesh_dir = args.mesh_dir or models_root()
    server = _setup_server(args.port)
    arm = URDFScene(server, arm_urdf, mesh_dir, "/robot")
    hand = URDFScene(server, hand_urdf, mesh_dir, "/robot/hand")
    # target_ee(IK 제어) 프레임 축 — 굵게. 정준축(원점)과 방향 비교.
    ee_frame = server.scene.add_frame("/target_ee", show_axes=True,
                                      axes_length=0.15, axes_radius=0.008)
    print(f"[ee] arm={arm_urdf}")
    print(f"[ee] hand={hand_urdf}  target_ee={target_ee}")
    print("[ee] 목표: target_ee 축을 정준 규약에 — 손끝→x(빨강,앞), 손바닥→-z(파랑 반대)")
    print("[ee] 메쉬 고정, 프레임 축만 회전. print yaml → hand yaml 에 붙여넣기.")

    with server.gui.add_folder("ee_align (hand yaml, rpy)"):
        s_rpy = _sl_rpy(server.gui, init_rpy)
    btn = server.gui.add_button("print yaml")

    def _snippet():
        return ("\n# ── hand yaml ──\n"
                f"ee_align: {{rpy: {_fmt(_v(s_rpy, 1 / DEG))}}}\n")

    @btn.on_click
    def _(_evt):
        print(_snippet())

    try:
        while True:
            arm.set_root(T_arm_align)
            arm.fk(np.zeros(arm.model.nq))
            # 손 루트(팔베이스 기준) = FK(ee.parent) ∘ ee.origin ∘ attach ∘ 손 axis_align
            T_h = arm.frame_pose(ee_parent) @ T_eeo @ T_at @ T_hand_align
            hand.set_root(T_h)
            hand.fk(np.zeros(hand.model.nq))
            # target_ee(제어 프레임) 정준 pose = 팔정렬 ∘ 손루트 ∘ FK(target_ee) ∘ ee_align
            T_ee = (T_arm_align @ T_h @ hand.frame_pose(target_ee)
                    @ origin_to_T(np.zeros(3), _v(s_rpy, 1 / DEG)))
            ee_frame.position = tuple(T_ee[:3, 3])
            ee_frame.wxyz = _wxyz_from_mat(T_ee[:3, :3])
            time.sleep(0.05)
    except KeyboardInterrupt:
        print(_snippet())


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="mode", required=True)

    p1 = sub.add_parser("robot", help="1단계: 로봇 하나를 정준 축에 정렬 (axis_align)")
    p1.add_argument("--robot", default=None,
                    help="robot config (configs/robots 상대) — URDF·현재 axis_align 로드")
    p1.add_argument("--urdf", default=None, help="raw URDF (config 없이)")
    p1.add_argument("--mesh-dir", default=None)
    p1.add_argument("--port", type=int, default=8080)

    p2 = sub.add_parser("attach", help="2단계: 팔 TCP 에 손 거치 보정 (ee.origin/attach)")
    p2.add_argument("--rig", default=None, help="rig 에서 URDF/현재값 로드 (권장)")
    p2.add_argument("--arm-robot", default=None, help="팔 robot config (rig 미지정 시)")
    p2.add_argument("--hand-robot", default=None, help="손 robot config (rig 미지정 시)")
    p2.add_argument("--urdf", default=None, help="팔 raw URDF (config 없이)")
    p2.add_argument("--hand-urdf", default=None, help="손 raw URDF (config 없이)")
    p2.add_argument("--ee-parent", default="joint7")
    p2.add_argument("--mesh-dir", default=None)
    p2.add_argument("--port", type=int, default=8080)

    p3 = sub.add_parser("ee", help="3단계: target_ee(IK 제어) 프레임 정렬 (ee_align)")
    p3.add_argument("--rig", required=True, help="rig (arm+hand) — URDF/확정값 로드")
    p3.add_argument("--mesh-dir", default=None)
    p3.add_argument("--port", type=int, default=8080)

    args = ap.parse_args()
    {"robot": mode_robot, "attach": mode_attach, "ee": mode_ee}[args.mode](args)


if __name__ == "__main__":
    main()
