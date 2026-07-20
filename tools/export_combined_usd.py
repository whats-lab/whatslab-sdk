#!/usr/bin/env python3
"""결합 USD export — rig(arm+hand) → 결합 URDF → USD (한 커맨드).

whatslab 이 config 하나로 텔레옵 기구학과 **sim 에셋(USD)** 을 동일 출처에서 낸다
(손으로 만든 USD 는 부착/손목이 rig 와 어긋나기 쉬움 — 이 도구로 정합 보장).

USD 변환은 Isaac Sim 앱이 필요하므로 **dex_vla(Isaac) 환경에서 실행**한다:

    conda activate dex_vla        # isaaclab + whatslab 이 있는 환경
    python ~/whatslab-sdk/tools/export_combined_usd.py \
        --rig rigs/nero_orca_right.yaml \
        --out ~/Desktop/dex_vla/assets/manipulator/OrcaNero.usd \
        --merge-joints                 # fixed joint 통합 (권장)
        # 베이스는 기본 고정(mount manipulator 표준). 떠있는 로봇이면 --free-base

동작: (1) export_combined_urdf 로 결합 URDF 생성(package:// → 절대경로),
(2) IsaacLab UrdfConverter 로 USD 변환. 결과 USD 를 dex_vla ArticulationCfg
(orca_nero.py 의 usd_path)가 가리키면 된다.
"""
import argparse
import os
import tempfile

# Isaac Sim 앱 런치 (다른 isaac 모듈 import 보다 먼저). isaac 환경에서만 가능.
try:
    from isaaclab.app import AppLauncher
except ImportError:
    raise SystemExit(
        "[export-usd] isaaclab 를 찾을 수 없습니다 — 이 스크립트는 Isaac Sim 환경\n"
        "  (예: conda activate dex_vla) 에서 실행해야 합니다. URDF 만 필요하면\n"
        "  tools/export_combined_urdf.py 를 쓰세요 (isaac 불필요).")

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--rig", default="rigs/nero_orca_right.yaml", help="rig config")
parser.add_argument("--out", required=True, help="출력 USD 경로 (.usd)")
parser.add_argument("--merge-joints", action="store_true",
                    help="fixed joint 로 연결된 링크 통합 (권장)")
parser.add_argument("--free-base", action="store_true",
                    help="베이스 고정 해제 (기본: 고정). 고정 mount manipulator 가 "
                         "일반적이라 기본 고정 — 안 하면 USD 가 free base 라 "
                         "articulation 이 월드에 앵커되지 않아 날아다닌다.")
parser.add_argument("--rename-map", default=None,
                    help="joint 이름 rename JSON {old: new} (sim 규약 정합용)")
parser.add_argument("--no-collision", action="store_true",
                    help="visual 메쉬로부터 콜라이더 생성 끄기 (기본 ON). 끄면 로봇에 "
                         "콜라이더가 없어 물체를 뚫는다 — 그랩엔 반드시 ON.")
parser.add_argument("--collider-type", default="convex_decomposition",
                    choices=["convex_hull", "convex_decomposition", "sdf"],
                    help="콜라이더 정밀도. convex_decomposition(기본)=볼록 조각 근사"
                         "(그랩에 충분·확실히 동작). convex_hull=최경량. "
                         "sdf=메쉬 SDF(가장 정밀)이나 UrdfConverter 가 메쉬를 "
                         "instanceable 로 만들어 후처리 편집이 막힘 → 현재 실험적"
                         "(de-instance 필요, 미완).")
parser.add_argument("--sdf-resolution", type=int, default=256,
                    help="SDF 격자 해상도 (sdf 사용 시, 높을수록 정밀·비쌈).")
parser.add_argument("--joint-stiffness", type=float, default=100.0)
parser.add_argument("--joint-damping", type=float, default=10.0)
parser.add_argument("--joint-target-type", default="position",
                    choices=["position", "velocity", "none"])
parser.add_argument("--no-sanitize-inertia", action="store_true",
                    help="관성 타당성 보정 끄기. 기본 ON — SolidWorks URDF 는 "
                         "손 링크 관성을 ~1e4 배 부풀려 내보내는 경우가 많고("
                         "예: 46g 손끝이 I=0.24 kg·m²) 그러면 어떤 액추에이터 "
                         "게인으로도 손가락이 무거운 플라이휠처럼 굴어 추적 불가.")
parser.add_argument("--inertia-max-r", type=float, default=0.2,
                    help="관성 타당성 임계 반경 [m]: I > mass*max_r^2 인 바디는 "
                         "물리적으로 불가능하다고 보고 교정 (기본 0.2).")
parser.add_argument("--inertia-fix-r", type=float, default=0.02,
                    help="교정 시 등방 관성 반경 [m]: I := mass*fix_r^2 (기본 0.02).")
parser.add_argument("--cap-link-mass", type=float, default=2.0,
                    help="이 [kg] 초과 링크 질량을 --link-mass-cap 으로 클램프 "
                         "(SolidWorks 단위 오류 대비, 기본 2.0). 0 이면 끔.")
parser.add_argument("--link-mass-cap", type=float, default=0.6,
                    help="질량 캡 대상 링크의 목표 질량 [kg] (기본 0.6).")
parser.add_argument("--grip-friction", type=float, nargs=2,
                    metavar=("STATIC", "DYNAMIC"), default=(2.0, 1.5),
                    help="손 링크 그립 재질 마찰 (static dynamic, 기본 2.0 1.5). "
                         "0 0 이면 끔.")
parser.add_argument("--hand-link-keys", nargs="*",
                    default=["index", "middle", "ring", "pinky", "thumb", "_AP",
                             "_PP", "_DP", "_TP", "FingerTip", "Carpals",
                             "TopTower", "ForeArm"],
                    help="그립 재질을 바인딩할 손 링크 이름 키워드.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app


def sanitize_inertia(usd_path: str, max_r: float, fix_r: float):
    """물리적으로 불가능한 링크 관성(부풀린 SolidWorks 값)을 등방으로 클램프.

    바디별 대각 관성의 최대 성분이 mass*max_r^2 를 넘으면(그만한 관성이 나오려면
    링크 반경이 max_r 보다 커야 함 → 손 링크엔 불가능) mass*fix_r^2 로 리셋한다.
    이름을 하드코딩하지 않아 어떤 핸드에도 적용되고, 정상인 팔 링크는 건드리지
    않는다(관성이 임계 이하라 통과). 값은 top layer override 로 저장."""
    from pxr import Usd, UsdPhysics, Gf
    stage = Usd.Stage.Open(usd_path)
    fixed = []
    for prim in stage.Traverse():
        if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
            continue
        mapi = UsdPhysics.MassAPI.Apply(prim)
        m = mapi.GetMassAttr().Get()
        diag = mapi.GetDiagonalInertiaAttr().Get()
        if not m or diag is None:
            continue
        if max(diag[0], diag[1], diag[2]) > float(m) * max_r ** 2:
            Iv = float(m) * fix_r ** 2
            mapi.CreateDiagonalInertiaAttr().Set(Gf.Vec3f(Iv, Iv, Iv))
            fixed.append((prim.GetName(), tuple(round(x, 4) for x in diag), Iv))
    stage.GetRootLayer().Save()
    print(f"[export-usd] 관성 타당성 보정: {len(fixed)}개 링크 "
          f"(I > m*{max_r}^2 → m*{fix_r}^2)")
    for nm, old, new in fixed:
        print(f"           · {nm}: I={old} → {new:.2e}")


def cap_link_mass(usd_path: str, mass_max: float, mass_cap: float):
    """비현실적으로 무거운 링크 질량(SolidWorks 단위 오류)을 mass_cap 으로 클램프.
    mass > mass_max 인 바디만 대상(정상 링크는 통과). 관성도 질량비로 함께 축소."""
    from pxr import Usd, UsdPhysics, Gf
    stage = Usd.Stage.Open(usd_path)
    capped = []
    for prim in stage.Traverse():
        if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
            continue
        mapi = UsdPhysics.MassAPI.Apply(prim)
        m = mapi.GetMassAttr().Get()
        if m and m > mass_max:
            diag = mapi.GetDiagonalInertiaAttr().Get()
            s = mass_cap / m
            mapi.GetMassAttr().Set(float(mass_cap))
            if diag is not None:
                mapi.CreateDiagonalInertiaAttr().Set(
                    Gf.Vec3f(diag[0]*s, diag[1]*s, diag[2]*s))
            capped.append((prim.GetName(), round(m, 3)))
    stage.GetRootLayer().Save()
    print(f"[export-usd] 질량 캡: {len(capped)}개 링크 (>{mass_max}kg → {mass_cap}kg) "
          f"{capped}")


def apply_sdf_colliders(usd_path: str, resolution: int):
    """Switch every mesh collider to SDF approximation — near-exact mesh
    collision that PhysX supports on DYNAMIC bodies (raw triangle mesh is
    static-only). Needed for precise finger↔object contact. Cost scales with
    sdf-resolution; drop to convex_decomposition if it stutters."""
    from pxr import Usd, UsdGeom, UsdPhysics
    try:
        from pxr import PhysxSchema
    except ImportError:
        PhysxSchema = None
    stage = Usd.Stage.Open(usd_path)
    n = 0
    for prim in stage.Traverse():
        if not prim.HasAPI(UsdPhysics.CollisionAPI):
            continue
        if not prim.IsA(UsdGeom.Mesh):
            continue
        UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr().Set("sdf")
        if PhysxSchema:
            sdf = PhysxSchema.PhysxSDFMeshCollisionAPI.Apply(prim)
            sdf.CreateSdfResolutionAttr().Set(int(resolution))
        n += 1
    stage.GetRootLayer().Save()
    print(f"[export-usd] SDF 콜라이더: {n}개 메쉬 (resolution={resolution})")


def bind_grip_material(usd_path: str, hand_link_keys, static: float, dynamic: float):
    """손 링크 콜라이더에 고마찰 그립 재질 바인딩(combine=max) — 물체 미끄러짐 방지.
    flat USD 는 /Hand 그룹이 없어 hand_link_keys 로 손 링크를 골라 개별 바인딩."""
    from pxr import Usd, UsdPhysics, UsdShade
    try:
        from pxr import PhysxSchema
    except ImportError:
        PhysxSchema = None
    stage = Usd.Stage.Open(usd_path)
    root = stage.GetDefaultPrim().GetPath().pathString
    grip = UsdShade.Material.Define(stage, root + "/Looks/GripMaterial")
    pm = UsdPhysics.MaterialAPI.Apply(grip.GetPrim())
    pm.CreateStaticFrictionAttr().Set(static)
    pm.CreateDynamicFrictionAttr().Set(dynamic)
    pm.CreateRestitutionAttr().Set(0.0)
    if PhysxSchema:
        PhysxSchema.PhysxMaterialAPI.Apply(grip.GetPrim()) \
            .CreateFrictionCombineModeAttr().Set("max")
    n = 0
    for prim in stage.Traverse():
        if prim.HasAPI(UsdPhysics.RigidBodyAPI) and \
                any(k in prim.GetName() for k in hand_link_keys):
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(
                grip, bindingStrength=UsdShade.Tokens.strongerThanDescendants,
                materialPurpose="physics")
            n += 1
    stage.GetRootLayer().Save()
    print(f"[export-usd] 그립 재질 바인딩: {n}개 손 링크 "
          f"(static={static} dynamic={dynamic} combine=max)")


def main():
    from isaaclab.sim.converters import UrdfConverter, UrdfConverterCfg

    # export_combined_urdf.py 는 같은 examples/ 디렉토리 (스크립트 실행 시 sys.path[0])
    from export_combined_urdf import export_combined_urdf

    out = os.path.abspath(os.path.expanduser(args_cli.out))
    os.makedirs(os.path.dirname(out), exist_ok=True)

    # 1) 결합 URDF (절대경로/USD-safe 메쉬, joint rename) → 임시 파일
    rename = None
    if args_cli.rename_map:
        import json
        with open(args_cli.rename_map) as f:
            rename = json.load(f)
    tmp_urdf = os.path.join(tempfile.gettempdir(), "whatslab_combined.urdf")
    export_combined_urdf(args_cli.rig, tmp_urdf, abs_meshes=True, rename=rename)

    # 2) URDF → USD (IsaacLab UrdfConverter)
    cfg = UrdfConverterCfg(
        asset_path=tmp_urdf,
        usd_dir=os.path.dirname(out),
        usd_file_name=os.path.basename(out),
        fix_base=not args_cli.free_base,
        merge_fixed_joints=args_cli.merge_joints,
        # Collision from the VISUAL meshes: the OrcaHand collision <mesh> tags
        # reference .dae files not bundled here, which the URDF mesh-fixer
        # drops -> the robot ended up with ZERO colliders and fingers tunnelled
        # through objects. Generating colliders from the (present) visual meshes
        # gives every link a real mesh-based collider. convex_decomposition =
        # close fit (needed so fingers actually grip, not a loose hull).
        collision_from_visuals=not args_cli.no_collision,
        # UrdfConverter only makes convex colliders; for --collider-type sdf we
        # create convex_decomposition prims here then switch them to SDF below.
        collider_type=("convex_decomposition"
                       if args_cli.collider_type == "sdf"
                       else args_cli.collider_type),
        joint_drive=UrdfConverterCfg.JointDriveCfg(
            gains=UrdfConverterCfg.JointDriveCfg.PDGainsCfg(
                stiffness=args_cli.joint_stiffness,
                damping=args_cli.joint_damping,
            ),
            target_type=args_cli.joint_target_type,
        ),
    )
    conv = UrdfConverter(cfg)
    print(f"[export-usd] USD 생성 완료 → {conv.usd_path}")

    # 3) 물리 보정: SDF 콜라이더 → 관성 클램프 → 질량 캡 → 그립 재질
    if args_cli.collider_type == "sdf" and not args_cli.no_collision:
        apply_sdf_colliders(conv.usd_path, args_cli.sdf_resolution)
    if not args_cli.no_sanitize_inertia:
        sanitize_inertia(conv.usd_path, args_cli.inertia_max_r,
                         args_cli.inertia_fix_r)
    if args_cli.cap_link_mass > 0:
        cap_link_mass(conv.usd_path, args_cli.cap_link_mass,
                      args_cli.link_mass_cap)
    gs, gd = args_cli.grip_friction
    if gs > 0 or gd > 0:
        bind_grip_material(conv.usd_path, args_cli.hand_link_keys, gs, gd)

    # 생성된 USD 의 prim 트리(링크/바디·조인트) 출력 — 카메라/센서 prim_path
    # 정합용(다운스트림이 /Robot/<link> 평면 구조를 알아야 함).
    try:
        from pxr import Usd, UsdPhysics
        stage = Usd.Stage.Open(conv.usd_path)
        print("[export-usd] ── USD prim 트리 (rigid body / joint) ──")
        for prim in stage.Traverse():
            t = prim.GetTypeName()
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                print(f"  [body]  {prim.GetPath()}")
            elif "Joint" in str(t):
                print(f"  [joint] {prim.GetPath()}  ({t})")
    except Exception as e:
        print(f"[export-usd] prim 트리 출력 실패 ({type(e).__name__}: {e})")
    print("[export-usd] dex_vla orca_nero.py usd_path + 카메라/센서 prim_path 를 "
          "위 경로에 맞추세요.")


if __name__ == "__main__":
    main()
    simulation_app.close()
