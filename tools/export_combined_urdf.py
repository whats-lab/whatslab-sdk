#!/usr/bin/env python3
"""결합 URDF export — rig(arm+hand)을 단일 URDF 로 병합 (IsaacLab USD 변환용).

whatslab 은 팔·손 URDF 를 런타임에 결합(arm_ik.from_appended)하지만, IsaacLab 등
sim 은 **단일 로봇 에셋**(URDF→USD)이 필요하다. 이 도구는 rig 의 부착 관계
(ee.origin ∘ attach ∘ hand.axis_align)를 fixed joint 로 구워 하나의 URDF 로 낸다.

    python ~/whatslab-sdk/tools/export_combined_urdf.py --rig rigs/nero_orca_right.yaml \
        --out /tmp/nero_orca_right.urdf

주의:
- axis_align/mount(정준 정렬)·reach 스케일은 whatslab 텔레옵 전용이라 **굽지 않는다**
  (로봇 베이스는 팔 URDF 원 베이스 그대로 — sim 이 씬에 배치).
- 메쉬 경로는 원본 그대로 유지 → IsaacLab URDF importer 에서 해석 가능해야 한다
  (package:// 또는 절대/상대경로). 필요 시 import 전에 경로 보정.
"""
import argparse
import os
import re
import shutil
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation

from whatslab.paths import models_root
from whatslab.robot import load_rig


def _existing_mesh(abspath: str):
    """실제 존재하는 메쉬 경로 반환. 없으면 같은 stem 의 형제(.stl/.obj) 나
    /dae/ 상위 디렉토리의 .stl 을 탐색(visual .dae 미번들 → collision .stl 대체)."""
    if os.path.exists(abspath):
        return abspath
    stem = os.path.splitext(abspath)[0]
    base = os.path.basename(stem)
    dirs = [os.path.dirname(abspath)]
    if os.path.basename(os.path.dirname(abspath)) == "dae":
        dirs.append(os.path.dirname(os.path.dirname(abspath)))   # /dae 제거
    for d in dirs:
        for e in (".stl", ".obj", ".STL", ".dae", ".DAE"):
            cand = os.path.join(d, base + e)
            if os.path.exists(cand):
                return cand
    return None


def _usd_safe(name: str) -> str:
    """USD prim 이름 규칙(영숫자·_·. 만) 위반 문자를 _ 로. Isaac 은 링크/조인트
    이름은 자동 정리하지만 **메쉬 파일 basename** 으로 만든 지오메트리 prim 은
    안 고쳐서(하이픈 등 → ill-formed SdfPath → null prim) 파일명을 미리 정리한다."""
    return re.sub(r"[^0-9A-Za-z._]", "_", name)


def _fix_mesh_paths(root: ET.Element, mesh_dir: str):
    """<mesh> 를 USD 변환 안전하게 정리: package:// → 절대, 없는 파일은 형제
    (.dae→.stl)로 스왑, 그래도 없으면 지오메트리 제거, 그리고 **USD-safe 이름으로
    mesh_dir 에 복사**(하이픈 등 제거)하고 URDF 가 사본을 가리키게 한다.

    반환: (복사수, 제거수)."""
    root_dir = models_root()
    parent = {c: p for p in root.iter() for c in p}
    os.makedirs(mesh_dir, exist_ok=True)
    dst_of: dict[str, str] = {}       # src(abs) → dst(복사본)
    used: dict[str, str] = {}         # dst basename → src (충돌 방지)
    copied = dropped = 0
    for mesh in list(root.iter("mesh")):
        fn = mesh.get("filename", "")
        if fn.startswith("package://"):
            # models_root() 는 (단일소스 리팩터 후) dexhand_description **패키지
            # 디렉토리 자체**를 가리키므로 "package://dexhand_description/" 를 통째로
            # 떼야 한다. 구 의미(패키지 부모)도 지원하도록 두 후보를 시도.
            rest = fn[len("package://"):]              # "dexhand_description/orca_hand/..."
            cand_parent = os.path.join(root_dir, rest)
            stripped = rest.split("/", 1)[1] if "/" in rest else rest
            cand_pkgdir = os.path.join(root_dir, stripped)
            abs_fn = cand_parent if os.path.exists(cand_parent) else cand_pkgdir
        else:
            abs_fn = fn
        resolved = _existing_mesh(abs_fn)
        if resolved is None:                       # 대체 불가 → 지오메트리 제거
            geom = parent.get(mesh)
            gp = parent.get(geom)                  # <visual>/<collision>
            gpp = parent.get(gp)                   # <link>
            if gpp is not None and gp is not None:
                gpp.remove(gp)
                dropped += 1
                print(f"[export] WARN: 메쉬 없음 → 지오메트리 제거: {abs_fn}")
            continue
        dst = dst_of.get(resolved)
        if dst is None:
            safe = _usd_safe(os.path.basename(resolved))
            stem, ext = os.path.splitext(safe)
            i = 1
            while safe in used and used[safe] != resolved:   # basename 충돌
                safe = f"{stem}_{i}{ext}"
                i += 1
            used[safe] = resolved
            dst = os.path.join(mesh_dir, safe)
            shutil.copyfile(resolved, dst)
            dst_of[resolved] = dst
            copied += 1
        mesh.set("filename", dst)
    return copied, dropped


def _resolve_parent_link(arm_root: ET.Element, ee_parent: str) -> str:
    """ee_parent(joint 또는 link 이름) → 부착 대상 링크. joint 면 그 child 링크."""
    for j in arm_root.findall("joint"):
        if j.get("name") == ee_parent:
            child = j.find("child")
            if child is None:
                raise ValueError(f"joint {ee_parent} 에 child 없음")
            return child.get("link")
    for lnk in arm_root.findall("link"):
        if lnk.get("name") == ee_parent:
            return ee_parent
    raise ValueError(f"ee_parent '{ee_parent}' 를 arm URDF 에서 못 찾음")


def _find_root_link(root: ET.Element) -> str:
    """어떤 joint 의 child 도 아닌 링크 = 루트 링크."""
    links = [lnk.get("name") for lnk in root.findall("link")]
    children = {j.find("child").get("link") for j in root.findall("joint")
                if j.find("child") is not None}
    roots = [n for n in links if n not in children]
    if len(roots) != 1:
        raise ValueError(f"루트 링크가 1개가 아님: {roots}")
    return roots[0]


def _rename_joints(root: ET.Element, rename: dict) -> int:
    """<joint name=...> 를 rename 맵(old→new)으로 치환. 링크/부모참조는 불변
    (액추에이터/action 은 joint 이름 기준). sim 규약 이름 정합용. 치환수 반환."""
    n = 0
    for j in root.iter("joint"):
        nm = j.get("name")
        if nm in rename:
            j.set("name", rename[nm])
            n += 1
    return n


def export_combined_urdf(rig_path: str, out_path: str,
                         abs_meshes: bool = True, rename: dict = None) -> str:
    rig = load_rig(rig_path)
    if rig.arm is None or rig.hand is None:
        raise SystemExit("[export] rig 에 arm+hand 둘 다 필요")

    arm_tree = ET.parse(rig.arm.urdf_abspath())
    arm_root = arm_tree.getroot()
    hand_root = ET.parse(rig.hand.urdf_abspath()).getroot()

    parent_link = _resolve_parent_link(arm_root, rig.arm.ee_parent)
    hand_root_link = _find_root_link(hand_root)

    # 부착 변환: arm ee_parent 프레임 → hand 루트 (model.py 결합 체인과 동일)
    T = rig.arm.ee_origin.T @ rig.attach.T @ rig.hand.axis_align.T
    xyz = T[:3, 3]
    rpy = Rotation.from_matrix(T[:3, :3]).as_euler("xyz")

    # 이름 충돌 검사 후 hand 의 link/joint/material 병합 (orca GUID 라 충돌 없음)
    arm_names = {e.get("name") for e in arm_root if e.tag in ("link", "joint")}
    merged = 0
    for el in list(hand_root):
        if el.tag in ("link", "joint"):
            if el.get("name") in arm_names:
                raise ValueError(f"이름 충돌: {el.get('name')} (arm/hand 중복)")
            arm_root.append(el)
            merged += 1
        elif el.tag == "material":                       # 링크가 참조하는 재질
            if el.get("name") not in {m.get("name")
                                      for m in arm_root.findall("material")}:
                arm_root.append(el)

    # 연결 fixed joint (parent=arm flange link, child=hand 루트 link)
    j = ET.SubElement(arm_root, "joint",
                      {"name": f"{parent_link}__to__{hand_root_link}",
                       "type": "fixed"})
    ET.SubElement(j, "parent", {"link": parent_link})
    ET.SubElement(j, "child", {"link": hand_root_link})
    ET.SubElement(j, "origin",
                  {"xyz": " ".join(f"{v:.6f}" for v in xyz),
                   "rpy": " ".join(f"{v:.6f}" for v in rpy)})

    n_ren = _rename_joints(arm_root, rename) if rename else 0

    copied = dropped = 0
    if abs_meshes:
        mesh_dir = os.path.join(os.path.dirname(os.path.abspath(out_path)), "meshes")
        copied, dropped = _fix_mesh_paths(arm_root, mesh_dir)

    ET.indent(arm_tree, space="  ")
    arm_tree.write(out_path, encoding="utf-8", xml_declaration=True)
    print(f"[export] {rig.arm.name}+{rig.hand.name} → {out_path}")
    if rename:
        print(f"[export] joint rename {n_ren}개 (sim 규약 이름 정합)")
    if abs_meshes:
        print(f"[export] 메쉬 {copied}개 USD-safe 이름으로 복사 → {mesh_dir}"
              + (f", 제거 {dropped}개" if dropped else ""))
    print(f"[export] 연결: {parent_link} → {hand_root_link}  "
          f"xyz={np.round(xyz,4).tolist()} rpy={np.round(rpy,4).tolist()}")
    print(f"[export] 병합 link/joint {merged}개")
    return out_path


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--rig", default="rigs/nero_orca_right.yaml")
    ap.add_argument("--out", required=True, help="출력 URDF 경로")
    ap.add_argument("--rename-map", default=None,
                    help="joint 이름 rename JSON {old: new} (sim 규약 정합용)")
    args = ap.parse_args()
    rename = None
    if args.rename_map:
        import json
        with open(args.rename_map) as f:
            rename = json.load(f)
    out = export_combined_urdf(args.rig, args.out, rename=rename)

    # 검증: pinocchio 로 로드해 팔+손 관절이 모두 있는지
    try:
        import pinocchio as pin
        m = pin.buildModelFromUrdf(out)
        print(f"[export] 검증 OK — pinocchio njoints={m.njoints} nq={m.nq}")
    except Exception as e:
        print(f"[export] 검증 경고: pinocchio 로드 실패 ({type(e).__name__}: {e})")


if __name__ == "__main__":
    main()
