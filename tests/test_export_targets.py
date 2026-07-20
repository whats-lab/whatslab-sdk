import json
import sys
from pathlib import Path

import numpy as np
import pytest

# tools/ 는 패키지가 아니라 독립 스크립트 디렉토리 (align_frames.py 등과 동일 관례) —
# 테스트에서 직접 import 하기 위해 sys.path 에 추가한다.
_TOOLS_DIR = Path(__file__).resolve().parents[1] / "tools"
if str(_TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(_TOOLS_DIR))

from whatslab.data.lerobot_recorder import LeRobotRecorder  # noqa: E402

import export_targets  # noqa: E402

FEATURES = {
    "observation.state": {"dtype": "float32", "shape": (7,), "names": None},
    "action": {"dtype": "float32", "shape": (7,), "names": None},
    "observation.images.cam0": {
        "dtype": "video",
        "shape": (64, 64, 3),
        "names": ["height", "width", "channels"],
    },
    "replay.obj0": {"dtype": "float32", "shape": (7,), "names": None},
}


def _make_small_dataset(tmp_path, name="ds"):
    rec = LeRobotRecorder(str(tmp_path / name), FEATURES, fps=30)
    rng = np.random.default_rng(0)
    for _ep in range(2):
        for f in range(3):
            rec.add_frame(
                np.full(7, f, np.float32),
                np.full(7, f, np.float32),
                {"cam0": rng.integers(0, 255, (64, 64, 3)).astype(np.uint8)},
                {"obj0": np.arange(7, dtype=np.float32)},
                task="pick the red block",
            )
        rec.save_episode()
    rec.finalize()
    return tmp_path / name


def test_build_modality_default_layout():
    modality = export_targets.build_modality(FEATURES)
    assert set(modality) == {"state", "action", "video", "annotation"}
    assert modality["state"] == {"state": {"start": 0, "end": 7}}
    assert modality["action"] == {"action": {"start": 0, "end": 7}}
    assert modality["video"] == {"cam0": {"original_key": "observation.images.cam0"}}
    assert modality["annotation"] == {"human.task_description": {}}


def test_build_modality_custom_layout():
    modality = export_targets.build_modality(
        FEATURES,
        state_layout={"arm": {"start": 0, "end": 4}, "hand": {"start": 4, "end": 7}},
        action_layout={"arm": {"start": 0, "end": 4}, "hand": {"start": 4, "end": 7}},
    )
    assert modality["state"] == {"arm": {"start": 0, "end": 4}, "hand": {"start": 4, "end": 7}}
    assert modality["action"] == {"arm": {"start": 0, "end": 4}, "hand": {"start": 4, "end": 7}}
    # video/annotation은 기본값 유지
    assert "cam0" in modality["video"]


def test_export_groot_writes_modality_json(tmp_path):
    root = _make_small_dataset(tmp_path)

    out_path = export_targets.export_groot(root)

    assert out_path == root / "meta" / "modality.json"
    assert out_path.exists()

    modality = json.loads(out_path.read_text())
    assert set(modality) == {"state", "action", "video", "annotation"}

    # video 키의 original_key 가 실제 observation.images.* feature 와 일치
    video_entries = list(modality["video"].values())
    assert len(video_entries) == 1
    original_key = video_entries[0]["original_key"]
    assert original_key.startswith("observation.images.")

    info = json.loads((root / "meta" / "info.json").read_text())
    assert original_key in info["features"]

    # v2.1 파일은 그대로 (unchanged) — info.json codebase_version 안 바뀜
    assert info["codebase_version"] == "v2.1"


def test_export_groot_with_modality_config_override(tmp_path):
    root = _make_small_dataset(tmp_path)
    config = {
        "state": {"arm": {"start": 0, "end": 4}, "hand": {"start": 4, "end": 7}},
        "action": {"arm": {"start": 0, "end": 4}, "hand": {"start": 4, "end": 7}},
    }
    config_path = tmp_path / "modality_config.json"
    config_path.write_text(json.dumps(config))

    out_path = export_targets.export_groot(root, config_path)
    modality = json.loads(out_path.read_text())
    assert modality["state"] == config["state"]
    assert modality["action"] == config["action"]


def test_export_v30_produces_loadable_v3_dataset(tmp_path):
    pytest.importorskip("lerobot")

    root = _make_small_dataset(tmp_path, name="whatslab_ref")

    out_dir = export_targets.export_v30(root)

    assert out_dir == root.parent / "whatslab_ref_v30"
    assert out_dir.exists()

    # 원본 v2.1 은 불변
    orig_info = json.loads((root / "meta" / "info.json").read_text())
    assert orig_info["codebase_version"] == "v2.1"

    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    ds = LeRobotDataset(out_dir.name, root=str(out_dir))
    assert ds.num_frames > 0
    assert ds.num_episodes == 2

    converted_info = json.loads((out_dir / "meta" / "info.json").read_text())
    assert converted_info["codebase_version"] == "v3.0"


def test_export_v30_custom_out_dir(tmp_path):
    pytest.importorskip("lerobot")

    root = _make_small_dataset(tmp_path, name="whatslab_ref2")
    custom_out = tmp_path / "custom_v30_dir"

    out_dir = export_targets.export_v30(root, out_dir=custom_out)

    assert out_dir == custom_out.resolve()

    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    ds = LeRobotDataset(out_dir.name, root=str(out_dir))
    assert ds.num_frames > 0
