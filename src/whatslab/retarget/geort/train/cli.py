from __future__ import annotations

import argparse
import dataclasses
import json
from pathlib import Path

from .data import build_data
from .evaluate import check_hand, tip_errors
from .hand import config_names
from .human import human_path
from .trainer import FREEZE, Hyper, run

CHECK_TOL = 1e-5


def hand_list(text: str) -> list[str]:
    return config_names() if text == "all" else [h for h in text.split(",") if h]


def add_hyper(parser: argparse.ArgumentParser) -> None:
    for field in dataclasses.fields(Hyper):
        flag = "--" + field.name.replace("_", "-")
        if field.name == "freeze":
            parser.add_argument(flag, choices=FREEZE, default=field.default)
        else:
            parser.add_argument(flag, type=type(field.default), default=field.default)


def parser() -> argparse.ArgumentParser:
    root = argparse.ArgumentParser(
        prog="whatslab-geort-train",
        description="GeoRT soft-prompt training (facebookresearch/GeoRT, CC BY-NC 4.0)",
    )
    sub = root.add_subparsers(dest="command", required=True)
    check = sub.add_parser("check", help="check hand training configs (frames, bend signs, FK)")
    check.add_argument("hands", help="comma-separated config names or yaml paths, or 'all'")
    data = sub.add_parser("data", help="build the training data file")
    data.add_argument("--hands", default="", help="comma-separated, 'all', or empty")
    data.add_argument("--human", type=Path, default=None)
    data.add_argument("--out", type=Path, required=True)
    data.add_argument("--frames", type=int, default=400_000)
    data.add_argument("--quest-share", type=float, default=0.35)
    data.add_argument("--seed", type=int, default=0)
    train = sub.add_parser("train", help="train and export a geort-prompt-v1 npz")
    train.add_argument("--data", type=Path, required=True)
    train.add_argument("--out", type=Path, required=True)
    train.add_argument("--device", default="cuda")
    train.add_argument("--init", type=Path, default=None)
    train.add_argument("--human-only", action="store_true")
    add_hyper(train)
    evaluate = sub.add_parser("eval", help="fingertip error of a model on human data")
    evaluate.add_argument("--model", type=Path, required=True)
    evaluate.add_argument("--hands", required=True)
    evaluate.add_argument("--human", type=Path, default=None)
    evaluate.add_argument("--frames", type=int, default=20_000)
    evaluate.add_argument("--seed", type=int, default=123)
    return root


def main(argv: list[str] | None = None) -> int:
    args = parser().parse_args(argv)
    if args.command == "check":
        bad = [h for h in hand_list(args.hands) if check_hand(h)]
        print(f"failed: {bad}" if bad else "all hands passed")
        return 1 if bad else 0
    if args.command == "data":
        hands = hand_list(args.hands)
        human = human_path(args.human)
        spec = build_data(hands, human, args.out, args.frames, args.quest_share, args.seed)
        print(f"saved {args.out}: {spec['human']['frames']} frames, hands {hands}")
        return 0
    if args.command == "eval":
        errors = tip_errors(
            args.model, hand_list(args.hands), human_path(args.human), args.frames, args.seed
        )
        print(json.dumps(errors, indent=1))
        return 0
    names = {f.name for f in dataclasses.fields(Hyper)}
    hp = Hyper(**{k: v for k, v in vars(args).items() if k in names})
    meta = run(args.data, args.out, hp, args.device, args.init, args.human_only)
    error = meta["numpy_vs_torch_max"]
    print(f"saved {args.out} in {meta['train_seconds']} s, numpy vs torch max diff {error:.2e}")
    return 0 if error <= CHECK_TOL else 3
