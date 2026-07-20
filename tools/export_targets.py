#!/usr/bin/env python3
"""v2.1 LeRobot 데이터셋 → 소비자 포맷 2종 파생 도구.

whatslab 의 미니 라이터(`whatslab.data.LeRobotRecorder`)는 LeRobot v2.1 만 낸다. VLA
학습 타깃은 둘로 갈린다:

  1. **GR00T** (NVIDIA Isaac-GR00T) — v2.x 그대로 두고 `meta/modality.json`
     만 추가하면 됨. concatenated `observation.state`/`action` 배열을
     이름 붙인 하위 필드(start/end 인덱스 범위)로 매핑.
     스키마: https://github.com/NVIDIA/Isaac-GR00T/blob/main/getting_started/data_preparation.md
  2. **v3.0** (pi0.5 / 최신 lerobot) — lerobot 공식
     `convert_dataset_v21_to_v30` 를 감싸 사본을 만든다 (원본 v2.1 은 불변).

    python ~/whatslab-sdk/tools/export_targets.py --input <v2.1_ds> --groot
    python ~/whatslab-sdk/tools/export_targets.py --input <v2.1_ds> --v30
    python ~/whatslab-sdk/tools/export_targets.py --input <v2.1_ds> --groot --v30 \
        --modality-config my_layout.json --out-v30 /tmp/ds_v30

--v30 은 lerobot(0.4.4)이 설치된 인터프리터로 실행해야 한다
(예: ~/micromamba/envs/dex_mj/bin/python) — --groot 만 쓸 경우 lerobot 불필요.
"""
from __future__ import annotations

import argparse
import json
import shutil
from pathlib import Path

# GR00T annotation 기본 키 — "사람이 준 task 지시문" 주석 슬롯 (source.type 규약).
DEFAULT_ANNOTATION_KEY = "human.task_description"


def _default_video_map(features: dict) -> dict:
    """observation.images.<cam> -> {"<cam>": {"original_key": "observation.images.<cam>"}}."""
    video = {}
    for key, spec in features.items():
        if spec.get("dtype") != "video":
            continue
        prefix = "observation.images."
        new_key = key[len(prefix):] if key.startswith(prefix) else key
        video[new_key] = {"original_key": key}
    return video


def build_modality(
    features: dict,
    state_layout: dict | None = None,
    action_layout: dict | None = None,
    video_map: dict | None = None,
    annotation: dict | None = None,
) -> dict:
    """GR00T `meta/modality.json` 구조를 만든다.

    `features` 는 v2.1 `meta/info.json["features"]` 딕셔너리(dtype/shape 포함).
    기본값은 `observation.state`/`action` 전체를 각각 단일 필드로 매핑한다
    (`{"start": 0, "end": J}`) — 팔/손 분리 등 커스텀 분할은 `state_layout`/
    `action_layout` 로 오버라이드한다 (호출자가 임의 필드 레이아웃을 넘길 수 있게).
    """
    if "observation.state" not in features:
        raise KeyError("features에 'observation.state'가 없음 — v2.1 info.json을 확인하라")
    if "action" not in features:
        raise KeyError("features에 'action'이 없음 — v2.1 info.json을 확인하라")

    state_dim = features["observation.state"]["shape"][0]
    action_dim = features["action"]["shape"][0]

    if state_layout is None:
        state_layout = {"state": {"start": 0, "end": state_dim}}
    if action_layout is None:
        action_layout = {"action": {"start": 0, "end": action_dim}}
    if video_map is None:
        video_map = _default_video_map(features)
    if annotation is None:
        annotation = {DEFAULT_ANNOTATION_KEY: {}}

    return {
        "state": state_layout,
        "action": action_layout,
        "video": video_map,
        "annotation": annotation,
    }


def export_groot(input_dir, modality_config: str | Path | dict | None = None) -> Path:
    """v2.1 데이터셋 옆에 `meta/modality.json` 을 쓴다. v2.1 파일은 건드리지 않는다."""
    input_dir = Path(input_dir)
    info_path = input_dir / "meta" / "info.json"
    info = json.loads(info_path.read_text())
    features = info["features"]

    overrides: dict = {}
    if modality_config is not None:
        overrides = (
            modality_config
            if isinstance(modality_config, dict)
            else json.loads(Path(modality_config).read_text())
        )

    modality = build_modality(
        features,
        state_layout=overrides.get("state"),
        action_layout=overrides.get("action"),
        video_map=overrides.get("video"),
        annotation=overrides.get("annotation"),
    )

    out_path = input_dir / "meta" / "modality.json"
    out_path.write_text(json.dumps(modality, indent=4))
    return out_path


def export_v30(input_dir, out_dir=None) -> Path:
    """v2.1 데이터셋을 복사한 뒤 lerobot의 `convert_dataset_v21_to_v30` 로 그 사본을
    v3.0 으로 변환한다 (원본 `input_dir` 는 절대 수정하지 않음).

    lerobot 은 in-place 변환만 지원(`root/repo_id` 규약, task-1b-report.md 확인)
    하므로, 여기서는 먼저 사본을 만들고 그 사본 경로에 대해 in-place 변환을 돌린다.
    """
    from lerobot.datasets.v30.convert_dataset_v21_to_v30 import convert_dataset

    input_dir = Path(input_dir).resolve()
    if out_dir is None:
        out_dir = input_dir.parent / f"{input_dir.name}_v30"
    out_dir = Path(out_dir).resolve()

    if out_dir.exists():
        shutil.rmtree(out_dir)
    shutil.copytree(input_dir, out_dir)

    convert_dataset(repo_id=out_dir.name, root=str(out_dir.parent), push_to_hub=False)
    return out_dir


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--input", required=True, help="v2.1 LeRobot 데이터셋 디렉토리")
    parser.add_argument("--groot", action="store_true", help="GR00T용 meta/modality.json 생성")
    parser.add_argument("--modality-config", default=None, help="state/action/video/annotation 레이아웃 오버라이드 JSON 경로")
    parser.add_argument("--v30", action="store_true", help="lerobot v3.0 사본 생성")
    parser.add_argument("--out-v30", default=None, help="v3.0 사본 출력 디렉토리 (기본: <input>_v30)")
    args = parser.parse_args(argv)

    if not args.groot and not args.v30:
        parser.error("--groot 와 --v30 중 최소 하나는 지정해야 함")

    if args.groot:
        path = export_groot(args.input, args.modality_config)
        print(f"[groot] wrote {path}")

    if args.v30:
        out = export_v30(args.input, args.out_v30)
        print(f"[v30] wrote {out}")


if __name__ == "__main__":
    main()
