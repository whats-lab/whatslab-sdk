import argparse
import json
import os
import re
import shutil
import xml.etree.ElementTree as ET
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np
import yaml

HERE = Path(__file__).resolve().parent
FPS = 30
Q_SCALE = 10000
P_SCALE = 10000
MOTIONS = (
    ("fist", "Flat to fist"),
    ("pinch_index", "Pinch thumb–index"),
    ("pinch_middle", "Pinch thumb–middle"),
    ("pinch_ring", "Pinch thumb–ring"),
    ("pinch_little", "Pinch thumb–little"),
    ("v_pose", "V pose"),
)
SIDES = ("right", "left")
GROUPS = ("Korean hands", "Korean humanoid hands", "Foreign hands", "Foreign humanoid hands")
KOREAN_MAKERS = {"Tesollo", "Wonik Robotics", "ROBOTIS"}
KOREAN_HUMANOID_MAKERS = {"ROBROS", "Rainbow Robotics"}
HUMANOID_MAKERS = {"Unitree", "Fourier", "RobotEra", "AgiBot"}
LINK = re.compile(r"\[([^\]]+)\]\(([^)]+)\)")
PACKAGE = "package://dexhand_description/"
NOTE_FILES = re.compile(r"^(LICENSE.*|PATCHES\.txt)$")


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--repo", type=Path, required=True)
    p.add_argument("--motions", type=Path, default=HERE / "motions")
    p.add_argument("--out", type=Path, default=HERE.parent / "_site")
    p.add_argument("--wheel", type=Path)
    p.add_argument("--repo-url", default="https://github.com/whats-lab/dexterous-hand-urdf")
    p.add_argument("--ref", default="master")
    p.add_argument("--jobs", type=int, default=os.cpu_count())
    p.add_argument("--check-only", action="store_true")
    return p.parse_args()


def cell_link(cell):
    m = LINK.search(cell)
    return (m.group(1), m.group(2)) if m else (cell.strip(), None)


def plain(cell):
    return LINK.sub(r"\1", cell).replace("`", "").strip()


def readme_rows(repo):
    rows = {}
    columns = None
    for line in (repo / "README.md").read_text().splitlines():
        cells = [c.strip() for c in line.strip().strip("|").split("|")]
        if cells and cells[0] == "Robot Model":
            columns = {c.split(" (")[0]: i for i, c in enumerate(cells)}
            continue
        if columns is None or "](robots/hands/" not in cells[0]:
            continue
        name, path = cell_link(cells[0])
        maker, website = cell_link(cells[columns["Official Website"]])
        source_cell = cells[columns["URDF Source"]]
        source, source_url = cell_link(source_cell)
        lic = plain(cells[columns["License"]])
        rows[path.rstrip("/").split("/")[-1]] = {
            "name": name,
            "maker": maker,
            "website": website,
            "source": source if source_url else plain(source_cell),
            "sourceUrl": source_url,
            "license": lic.replace("†", "").strip(),
            "licenseUnclear": "†" in lic,
        }
    return rows


def hand_folders(hands_root):
    return sorted(d.name for d in hands_root.iterdir() if d.is_dir() and any(d.rglob("*.urdf")))


def check_readme(rows, folders):
    missing_folder = sorted(set(rows) - set(folders))
    missing_row = sorted(set(folders) - set(rows))
    if missing_folder or missing_row:
        raise SystemExit(
            f"README table and robots/hands disagree: rows without a folder {missing_folder}, "
            f"folders without a row {missing_row}"
        )


def group_of(row):
    if row["maker"] in KOREAN_MAKERS:
        return GROUPS[0]
    if row["maker"] in KOREAN_HUMANOID_MAKERS:
        return GROUPS[1]
    if row["maker"] in HUMANOID_MAKERS or "Unitree" in row["name"]:
        return GROUPS[3]
    return GROUPS[2]


def side_of(name):
    tokens = set(re.split(r"[^a-z]+", name.lower()))
    return "right" if "right" in tokens else "left" if "left" in tokens else None


def folder_urdfs(hands_root, hid):
    by_side = {}
    for p in sorted((hands_root / hid).rglob("*.urdf"), key=lambda p: (len(p.name), p.name)):
        by_side.setdefault(side_of(p.name), p.relative_to(hands_root).as_posix())
    fallback = by_side.get(None) or next(iter(by_side.values()))
    return {s: by_side.get(s) or fallback for s in SIDES}


def urdf_structure(path):
    root = ET.parse(path).getroot()
    joints = []
    for j in root.findall("joint"):
        joints.append(
            {
                "name": j.get("name"),
                "type": j.get("type"),
                "parent": j.find("parent").get("link"),
                "child": j.find("child").get("link"),
                "mimic": j.find("mimic") is not None,
            }
        )
    movable = [j for j in joints if j["type"] != "fixed"]
    children = {}
    for j in joints:
        children.setdefault(j["parent"], []).append(j)

    def has_movable(link):
        return any(j["type"] != "fixed" or has_movable(j["child"]) for j in children.get(link, []))

    fingers = sum(1 for j in movable if not has_movable(j["child"]))
    names = [j["name"] for j in movable if not j["mimic"]]
    return {"dof": len(names), "fingers": fingers, "joints": names}


def hand_configs(configs_dir, folders):
    chosen = {}
    for f in sorted(configs_dir.glob("*.yaml")):
        cfg = yaml.safe_load(f.read_text())
        hid = cfg["urdf"].split("/")[0]
        if hid not in folders:
            continue
        key = (hid, cfg["side"])
        if key not in chosen or len(f.stem) < len(chosen[key]):
            chosen[key] = f.stem
    return chosen


def load_clip(path):
    d = np.load(path)
    t = d["t"]
    idx = np.searchsorted(t, np.arange(0.0, t[-1] + 1e-9, 1.0 / FPS))
    return d["xr_joints"][np.minimum(idx, len(t) - 1)].astype(float)


def canonical_skeleton(xr_seq, mirror, as_side):
    from whatslab.retarget.dex.preprocess import mano_palm_frame, mirror_xr_joints, xr_to_mano

    out = []
    for xr in xr_seq:
        mano = xr_to_mano(mirror_xr_joints(xr) if mirror else xr)
        out.append(mano @ mano_palm_frame(mano, as_side))
    return np.array(out)


def quantize(a, scale):
    return np.rint(np.asarray(a) * scale).astype(int).tolist()


def mesh_files(urdf_path, hands_root):
    files = {urdf_path.resolve()}
    for visual in ET.parse(urdf_path).getroot().iter("visual"):
        for mesh in visual.iter("mesh"):
            name = mesh.get("filename")
            if name.startswith(PACKAGE):
                path = hands_root / name[len(PACKAGE) :]
            else:
                path = urdf_path.parent / name
            files.add(path.resolve())
            if path.suffix.lower() == ".obj":
                for line in path.read_text(errors="ignore").splitlines():
                    if line.startswith("mtllib "):
                        files.add((path.parent / line.split(None, 1)[1].strip()).resolve())
    return files


def solve_hand(job):
    from whatslab.retarget.dex import DexEngine

    hid, side, config, clips_path = job
    clips = dict(np.load(clips_path, allow_pickle=True).item())
    engine = DexEngine(config, side=side)
    kin = engine.kinematics
    names = list(kin.actuated_joint_names)
    idx = [kin.joint_names.index(n) for n in names]
    lim = engine.joint_limits
    rest = np.clip(np.zeros(len(lim)), lim[:, 0], lim[:, 1])
    tf = kin.link_placements(engine.full_qpos(rest))
    wrist = tf[kin.frame_ids([engine.wrist_frame])[0], :3, 3]
    out = {"clips": {}, "failures": 0}
    fit_c, fit_k = [], []
    for motion, _ in MOTIONS:
        xr_seq = clips[f"{motion}_{side}"]
        engine.reset()
        last = rest
        frames = []
        for xr in xr_seq:
            q = engine.solve(xr)
            if q is None:
                out["failures"] += 1
            else:
                last = q
            frames.append(engine.full_qpos(last)[idx])
        out["clips"][motion] = quantize(frames, Q_SCALE)
        sample = xr_seq[:: max(1, len(xr_seq) // 10)]
        fit_c.append(canonical_skeleton(sample, engine.mirror, engine.config.side))
        fit_k.append(np.array([engine.human_keypoints(xr) for xr in sample]))
    c = np.concatenate(fit_c).reshape(-1, 3)
    k = np.concatenate(fit_k).reshape(-1, 3)
    rot_t, *_ = np.linalg.lstsq(c, k, rcond=None)
    out.update(
        {
            "hand": hid,
            "side": side,
            "config": config,
            "urdf": engine.config.urdf,
            "mirrored": bool(engine.mirror),
            "modelSide": engine.config.side,
            "joints": names,
            "palm": np.round(rot_t, 6).tolist(),
            "fitError": float(np.abs(c @ rot_t - k).max()),
            "wrist": np.round(wrist, 6).tolist(),
            "scale": round(float(engine.scaling_factor), 6),
        }
    )
    return out


def main():
    args = parse_args()
    hands_root = (args.repo / "robots" / "hands").resolve()
    rows = readme_rows(args.repo)
    folders = hand_folders(hands_root)
    check_readme(rows, folders)
    print(f"README table matches robots/hands: {len(folders)} hands")
    if args.check_only:
        return

    os.environ["WHATSLAB_MODELS_ROOT"] = str(hands_root)
    from whatslab.paths import configs_root

    out = args.out
    if out.exists():
        shutil.rmtree(out)
    shutil.copytree(HERE / "web", out)
    (out / "data" / "hands").mkdir(parents=True)

    configs = hand_configs(Path(configs_root()) / "hands", set(folders))
    clips = {
        f"{m}_{s}": load_clip(args.motions / f"{m}_{s}.npz") for m, _ in MOTIONS for s in SIDES
    }
    clips_path = out / "clips.npy"
    np.save(clips_path, np.array(clips, dtype=object), allow_pickle=True)
    jobs = []
    for hid in folders:
        for side in SIDES:
            config = configs.get((hid, side)) or configs.get((hid, SIDES[1 - SIDES.index(side)]))
            if config:
                jobs.append((hid, side, config, str(clips_path)))
    with ProcessPoolExecutor(args.jobs) as pool:
        solved = list(pool.map(solve_hand, jobs))
    clips_path.unlink()

    retargeted = {}
    skeleton = {}
    for res in solved:
        retargeted.setdefault(res["hand"], {})[res["side"]] = res
        key = f"{res['side']}_as_{res['modelSide']}"
        if key not in skeleton:
            skeleton[key] = {
                m: quantize(
                    canonical_skeleton(
                        clips[f"{m}_{res['side']}"], res["mirrored"], res["modelSide"]
                    ).reshape(-1, 63),
                    P_SCALE,
                )
                for m, _ in MOTIONS
            }

    model_files = set()
    hands = []
    for hid in folders:
        row = rows[hid]
        solved_sides = retargeted.get(hid, {})
        urdfs = {s: solved_sides[s]["urdf"] for s in solved_sides} or folder_urdfs(hands_root, hid)
        data = {"id": hid, "sides": {}}
        for side, rel in urdfs.items():
            model_files |= mesh_files(hands_root / rel, hands_root)
            entry = {"urdf": f"robots/{rel}"}
            res = solved_sides.get(side)
            if res:
                entry.update(
                    {
                        "config": res["config"],
                        "mirrored": res["mirrored"],
                        "joints": res["joints"],
                        "palm": res["palm"],
                        "wrist": res["wrist"],
                        "scale": res["scale"],
                        "skeleton": f"{side}_as_{res['modelSide']}",
                        "clips": res["clips"],
                    }
                )
            else:
                entry["mirrored"] = side_of(rel) not in (side, None)
            data["sides"][side] = entry
        (out / "data" / "hands" / f"{hid}.json").write_text(
            json.dumps(data, separators=(",", ":"))
        )
        primary = urdfs.get("right") or urdfs["left"]
        structure = urdf_structure(hands_root / primary)
        notes = sorted(p.name for p in (hands_root / hid).iterdir() if NOTE_FILES.match(p.name))
        base = f"{args.repo_url}/tree/{args.ref}/robots/hands/{hid}"
        hands.append(
            {
                "id": hid,
                **row,
                "group": group_of(row),
                "dof": structure["dof"],
                "fingers": structure["fingers"],
                "retargeted": bool(solved_sides),
                "sides": [s for s in SIDES if s in urdfs and not data["sides"][s]["mirrored"]],
                "folder": base,
                "notes": [
                    {"name": n, "url": base.replace("/tree/", "/blob/", 1) + "/" + n}
                    for n in notes
                ],
            }
        )
        fails = sum(s["failures"] for s in solved_sides.values())
        fit = max((s["fitError"] for s in solved_sides.values()), default=0.0)
        print(
            f"{hid:28s} dof={structure['dof']:2d} fingers={structure['fingers']} "
            f"sides={','.join(urdfs)} retarget={'yes' if solved_sides else 'no '} "
            f"fail={fails} fit={fit:.1e}"
        )

    hands.sort(key=lambda h: GROUPS.index(h["group"]))
    (out / "data" / "hands.json").write_text(
        json.dumps(
            {"groups": GROUPS, "hands": hands, "default": "tesollo_dg5f_s"},
            separators=(",", ":"),
            ensure_ascii=False,
        )
    )
    (out / "data" / "motions.json").write_text(
        json.dumps(
            {
                "fps": FPS,
                "qScale": Q_SCALE,
                "pScale": P_SCALE,
                "motions": [{"id": m, "label": label} for m, label in MOTIONS],
                "skeleton": skeleton,
            },
            separators=(",", ":"),
            ensure_ascii=False,
        )
    )

    for src in sorted(model_files):
        dst = out / "robots" / src.relative_to(hands_root)
        dst.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(src, dst)
    if args.wheel:
        (out / "py").mkdir()
        shutil.copy2(args.wheel, out / "py" / args.wheel.name)
        (out / "py" / "wheel.json").write_text(json.dumps({"wheel": args.wheel.name}))
    total = sum(f.stat().st_size for f in out.rglob("*") if f.is_file())
    print(
        f"{len(hands)} hands ({len(retargeted)} retargeted), {len(model_files)} model files, "
        f"{total / 1e6:.1f} MB -> {out}"
    )


if __name__ == "__main__":
    main()
