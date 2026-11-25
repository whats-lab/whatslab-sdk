#!/usr/bin/env python3
import argparse
import dataclasses
import sys

import numpy as np
import yaml

from whatslab.retarget.dex import DexEngine, hand_config_names, resolve_hand_config
from whatslab.retarget.dex.posture import POSTURE_KEY, AbductionSpec, spread_joints

FINGERS = ("thumb", "index", "middle", "ring", "little")
FLEX_SLOTS = {1: [{"mcp": 1.0}], 2: [{"mcp": 1.0}, {"pip": 1.0, "dip": 0.5}],
              3: [{"mcp": 1.0}, {"pip": 1.0}, {"dip": 1.0}]}
THUMB_SLOTS = {1: [{"mcp": 1.0, "ip": 1.0}], 2: [{"mcp": 1.0}, {"ip": 1.0}]}
DEFAULT_WEIGHT = 0.01
PALM_DEPTH = 0.03
TWIST_RATIO = 0.2
FLEX_PROBE = 0.5
MIN_BONE = 0.005
ONE_SIDED = np.radians(5.0)
CUP_GAIN = 0.25
AXIS_COS = np.cos(np.radians(45.0))


class Hand:
    def __init__(self, eng: DexEngine):
        self.eng = eng
        self.kin = eng.kinematics
        kin = self.kin
        self.open = np.clip(np.zeros(len(eng.joint_names)), *eng.joint_limits.T)
        self.pose(self.open)
        self.palm = eng.robot_palm
        side = eng.config.side
        self.volar = self.palm[:, 0]
        self.ulnar = self.palm[:, 1] * (-1.0 if side == "right" else 1.0)
        self.source = {n: (n, 1.0) for n in eng.joint_names}
        for m, s, k in zip(kin.mimic_names, kin.mimic_source_names, kin.mimic_multipliers):
            if s in self.source:
                self.source[m] = (s, float(k))
        self.child = {}
        for i, j in enumerate(kin.link_joints):
            if j >= 0 and kin.joint_names[j] not in self.child:
                self.child[kin.joint_names[j]] = kin.link_names[i]
        wrist = self.link_pos(eng.wrist_frame)
        mcps = [np.mean([self.link_pos(n) for n in g], axis=0) for g in eng.config.palm_links]
        self.palm_point = np.mean([wrist, *mcps], axis=0) + PALM_DEPTH * self.volar

    def pose(self, x):
        qpos = self.eng.full_qpos(x)
        self.world = self.kin.joint_frames(qpos)
        self.links = self.kin.link_placements(qpos)

    def link_pos(self, name):
        return self.links[self.kin.frame_ids([name])[0], :3, 3]

    def chain(self, tip):
        kin = self.kin
        j = int(kin.link_joints[kin.frame_ids([tip])[0]])
        out = []
        while j >= 0:
            name = kin.joint_names[j]
            if name in self.source and not kin.joint_prismatic[j]:
                out.append(name)
            j = kin.joint_parents[j]
        return out[::-1]

    def axis(self, joint):
        j = self.kin.joint_names.index(joint)
        return self.world[j, :3, :3] @ self.kin.joint_axes[j], self.world[j, :3, 3]

    def tip_velocity(self, joint, tip):
        axis, origin = self.axis(joint)
        return np.cross(axis, self.link_pos(tip) - origin)

    def kind(self, joint, tip):
        v = self.tip_velocity(joint, tip)
        reach = np.linalg.norm(self.link_pos(tip) - self.axis(joint)[1])
        if not np.linalg.norm(v) > TWIST_RATIO * reach:
            return "twist"
        return "flexion" if abs(v @ self.volar) >= abs(v @ self.ulnar) else "abduction"

    def flexed(self, joints, tip):
        x = self.open.copy()
        lim = self.eng.joint_limits
        for j in joints:
            source, _ = self.source[j]
            i = self.eng.joint_names.index(source)
            up = 1.0 if self.tip_velocity(j, tip) @ self.volar > 0 else 0.0
            x[i] = lim[i, 0] + (FLEX_PROBE if up else 1.0 - FLEX_PROBE) * (lim[i, 1] - lim[i, 0])
        return x


def combine(entries):
    out = {}
    for source, terms in entries:
        out.setdefault(source, []).append(terms)
    gains = {}
    for source, rows in out.items():
        norm = sum(k * k for k, _ in rows)
        merged = {}
        for k, terms in rows:
            for name, g in terms.items():
                merged[name] = merged.get(name, 0.0) + k * g / norm
        gains[source] = {n: round(v, 3) for n, v in merged.items() if abs(v) > 1e-9}
    return gains


def finger_entries(hand, finger, joints, tip):
    joints = [j for j in joints if j not in hand.shared]
    kinds = {j: hand.kind(j, tip) for j in joints}
    twist = [j for j in joints if kinds[j] == "twist"]
    if twist:
        hand.pose(hand.flexed([j for j in joints if kinds[j] == "flexion"], tip))
        kinds.update({j: hand.kind(j, tip) for j in twist})
    lateral = [j for j in joints if kinds[j] == "abduction"]
    first_flex = next((i for i, j in enumerate(joints) if kinds[j] == "flexion"), len(joints))
    before = [j for j in lateral if joints.index(j) < first_flex]
    abduction = before[-1:] or lateral[:1]
    cup = [j for j in joints if kinds[j] == "flexion" and abduction
           and joints.index(j) < joints.index(abduction[0])
           and np.linalg.norm(hand.axis(j)[1] - hand.axis(abduction[0])[1])
           > np.linalg.norm(hand.axis(j)[1] - hand.link_pos(hand.eng.wrist_frame))]
    flexion = [j for j in joints if kinds[j] == "flexion" and j not in cup][-3:]
    entries = []
    for j in cup:
        source, k = hand.source[j]
        sign = 1.0 if hand.tip_velocity(j, tip) @ hand.volar > 0 else -1.0
        entries.append((source, (k, {f"{finger}.{n}+": sign * CUP_GAIN
                                     for n in ("mcp", "pip", "dip")})))
    for j in abduction:
        source, k = hand.source[j]
        sign = 1.0 if hand.tip_velocity(j, tip) @ hand.ulnar > 0 else -1.0
        entries.append((source, (k, {f"{finger}.spread": sign})))
    hand.pose(hand.open)
    slots = FLEX_SLOTS.get(len(flexion), [])
    for j, slot in zip(flexion, slots):
        source, k = hand.source[j]
        sign = 1.0 if hand.tip_velocity(j, tip) @ hand.volar > 0 else -1.0
        entries.append((source, (k, {f"{finger}.{n}": sign * g for n, g in slot.items()})))
    return entries


def one_sided(hand, joint):
    source, k = hand.source[joint]
    lo, hi = hand.eng.joint_limits[hand.eng.joint_names.index(source)]
    if lo >= -ONE_SIDED or hi <= ONE_SIDED:
        return (1.0 if hi > -lo else -1.0) * (1.0 if k > 0 else -1.0)
    return None


def thumb_sign(hand, joint, tip):
    fixed = one_sided(hand, joint)
    if fixed is not None:
        return fixed
    toward = hand.palm_point - hand.link_pos(tip)
    return 1.0 if hand.tip_velocity(joint, tip) @ toward > 0 else -1.0


def thumb_block(hand, joints, tip):
    driven = [j for j in joints if hand.source[j][0] == j]
    if len(driven) < 2:
        return None, [(hand.source[j][0], (hand.source[j][1], {"thumb.mcp": thumb_sign(
            hand, j, tip)})) for j in joints[-1:]]
    base = driven[:2]
    after = joints[joints.index(base[1]) + 1:]
    start = hand.link_pos(hand.child[base[1]])
    ends = [hand.child[j] for j in after] + [tip]
    bone_end = next(n for n in ends if np.linalg.norm(hand.link_pos(n) - start) > MIN_BONE)
    flex_joint = after[0] if after else base[1]
    spec = {"joints": base, "bone": [hand.child[base[1]], bone_end], "flex_joint": flex_joint,
            "step_deg": 2.0, "axis_weight": 0.5}
    distal = hand.axis(joints[-1])[0]
    flexion = [j for j in after
               if hand.source[j][0] not in base and hand.kind(j, tip) != "twist"
               and abs(hand.axis(j)[0] @ distal) > AXIS_COS]
    slots = THUMB_SLOTS.get(len(flexion)) or (
        [{"mcp": 1.0}] + [{"ip": round(1.0 / (len(flexion) - 1), 3)}] * (len(flexion) - 1))
    entries = []
    for j, slot in zip(flexion, slots):
        source, k = hand.source[j]
        sign = thumb_sign(hand, j, tip)
        entries.append((source, (k, {f"thumb.{n}": sign * g for n, g in slot.items()})))
    return spec, entries


def posture_block(config, urdf_root=None) -> dict:
    eng = DexEngine(config, urdf_root=urdf_root, posture_prior=False)
    name = eng.config.name
    hand = Hand(eng)
    tips = eng.tip_frames
    if len(tips) not in (4, 5):
        raise ValueError(f"{name}: 손가락 {len(tips)}개 — 4·5손가락 손만 만든다")
    chains = {f: hand.chain(t) for f, t in zip(FINGERS, tips)}
    seen = {}
    for joints in chains.values():
        for j in joints:
            seen[j] = seen.get(j, 0) + 1
    hand.shared = {j for j, c in seen.items() if c > 1}
    thumb = [j for j in chains["thumb"] if j not in hand.shared]
    spec, entries = thumb_block(hand, thumb, tips[0])
    for finger, tip in zip(FINGERS[1:], tips[1:]):
        entries += finger_entries(hand, finger, chains[finger], tip)
    block = {"weight": DEFAULT_WEIGHT, "joints": combine(entries)}
    if spec is not None:
        block["thumb_base"] = spec
    if spread_joints(block["joints"]):
        block["abduction"] = dataclasses.asdict(AbductionSpec())
    return block


def dump(block: dict) -> str:
    lines = [f"{POSTURE_KEY}:", f"  weight: {block['weight']}", "  joints:"]
    for joint, terms in block["joints"].items():
        body = ", ".join(f"{n}: {g}" for n, g in terms.items())
        lines.append(f"    {joint}: {{{body}}}")
    if "thumb_base" in block:
        t = block["thumb_base"]
        lines += ["  thumb_base:", f"    joints: [{', '.join(t['joints'])}]",
                  f"    bone: [{', '.join(t['bone'])}]", f"    flex_joint: {t['flex_joint']}",
                  f"    step_deg: {t['step_deg']}", f"    axis_weight: {t['axis_weight']}"]
    if "abduction" in block:
        body = ", ".join(f"{k}: {v}" for k, v in block["abduction"].items())
        lines.append(f"  abduction: {{{body}}}")
    return "\n".join(lines) + "\n"


def write(name: str, text: str) -> None:
    path = resolve_hand_config(name)
    with open(path) as f:
        current = f.read()
    if POSTURE_KEY in (yaml.safe_load(current) or {}):
        raise ValueError(f"{name}: 이미 {POSTURE_KEY} 가 있다 — 지우고 다시 만든다")
    with open(path, "w") as f:
        f.write(current.rstrip("\n") + "\n" + text)


def main() -> int:
    ap = argparse.ArgumentParser(description="URDF·DexPilot config 로 posture_prior 블록을 만든다")
    ap.add_argument("names", nargs="*", help="hand config 이름 (없으면 --all)")
    ap.add_argument("--all", action="store_true", help="posture_prior 가 없는 동봉 config 전부")
    ap.add_argument("--write", action="store_true", help="config 파일 끝에 블록을 붙인다")
    args = ap.parse_args()
    names = args.names or (hand_config_names() if args.all else [])
    if not names:
        ap.error("config 이름이나 --all 을 준다")
    for name in names:
        with open(resolve_hand_config(name)) as f:
            if args.all and POSTURE_KEY in (yaml.safe_load(f) or {}):
                continue
        text = dump(posture_block(name))
        if args.write:
            write(name, text)
            print(f"{name}: 썼다")
        else:
            print(f"# {name}\n{text}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
