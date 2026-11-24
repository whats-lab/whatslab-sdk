from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from ..preprocess import to_geort
from ..prompt import load_prompt_model
from .hand import HandKinematics, read_hand, urdf_path, urdf_tips
from .human import FINGERS, TIPS, load_human, sample_frames

THUMB = "thumb"
MIN_REACH_M = 0.05
MIMIC_TOL_M = 1e-9
MIMIC_SAMPLES = 20
EVAL_QUEST_SHARE = 0.35


def check_hand(hand: str) -> list[str]:
    cfg = read_hand(hand)
    kin = HandKinematics(cfg, urdf_path(cfg))
    names = [f.name for f in kin.fingers]
    q0 = kin.zero()
    tips = kin.tips(q0)
    problems = []
    print(f"== {cfg['name']}: {len(kin.joint_order)} joints, fingers {names}")
    for name, tip in zip(names, tips, strict=True):
        print(f"  q0 tip {name:7s} {np.round(tip * 1000, 1)} mm")
    for i, joint in enumerate(kin.joint_order):
        finger = next(k for k, f in enumerate(kin.fingers) if any(s.index == i for s in f.steps))
        if not (cfg.get("posture") or {}).get(joint) or names[finger] == THUMB:
            continue
        q = q0.copy()
        q[i] = 0.5 * (kin.lower[i] + kin.upper[i])
        moved = (kin.tips(q)[finger] - tips[finger])[0]
        print(f"  flex {joint:>28s} -> {names[finger]:7s} toward palm {moved * 1000:+6.1f} mm")
        if moved <= 0.0:
            problems.append(f"{joint} bends {names[finger]} away from the palm")
    if tips[names.index(THUMB)][1] <= 0.0:
        problems.append("thumb is not on +Y at q=0")
    short = [n for n, t in zip(names, tips, strict=True) if n != THUMB and t[2] <= MIN_REACH_M]
    if short:
        problems.append(f"fingers not along +Z at q=0: {short}")
    rng = np.random.default_rng(0)
    worst = max(
        float(np.abs(urdf_tips(kin, q) - kin.tips(q)).max())
        for q in rng.uniform(kin.lower, kin.upper, size=(MIMIC_SAMPLES, len(kin.lower)))
    )
    print(f"  chain FK vs URDF FK (with mimic) max {worst * 1000:.1e} mm")
    if worst > MIMIC_TOL_M:
        problems.append(f"chain FK differs from URDF FK by {worst:.2e} m")
    for p in problems:
        print(f"  PROBLEM: {p}")
    return problems


def tip_errors(
    model: Path, hands: list[str], human: Path, frames: int, seed: int
) -> dict[str, dict[str, object]]:
    data = load_human(human)
    pick = sample_frames(
        data.points, data.source, frames, EVAL_QUEST_SHARE, np.random.default_rng(seed)
    )
    canonical = data.points[pick]
    inputs = to_geort(canonical[np.all(np.isfinite(canonical), axis=(1, 2))])
    prompt = load_prompt_model(model)
    with np.load(model, allow_pickle=False) as d:
        scales = json.loads(str(d["meta_json"]))["scales"]
    out = {}
    for hand in hands:
        cfg = read_hand(hand)
        name = cfg["name"]
        kin = HandKinematics(cfg, urdf_path(cfg))
        emb = prompt.embodiments[name]
        q = emb.forward(inputs)
        names = [f.name for f in kin.fingers]
        target = inputs[:, [TIPS[FINGERS.index(n)] for n in names]] * scales[name]
        err = np.linalg.norm(kin.tips(q) - target, axis=-1) * 1000.0
        span = emb.upper - emb.lower
        saturated = np.mean((q - emb.lower < 0.01 * span) | (emb.upper - q < 0.01 * span), axis=0)
        out[name] = {
            "mean_mm": round(float(err.mean()), 2),
            "p95_mm": round(float(np.percentile(err, 95)), 2),
            "fingers_mm": {n: round(float(e), 1) for n, e in zip(names, err.mean(0), strict=True)},
            "max_saturation": round(float(saturated.max()), 3),
            "scale": scales[name],
        }
    return out
