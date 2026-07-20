import json

import numpy as np
import pytest

from whatslab.data.lerobot_recorder import LeRobotRecorder

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


def test_layout_matches_reference(tmp_path):
    root = _make_small_dataset(tmp_path)
    # 필수 파일 존재
    assert (root / "meta/info.json").exists()
    assert (root / "meta/episodes.jsonl").exists()
    assert (root / "meta/tasks.jsonl").exists()
    assert (root / "meta/episodes_stats.jsonl").exists()
    assert (root / "data/chunk-000/episode_000000.parquet").exists()
    assert (root / "data/chunk-000/episode_000001.parquet").exists()
    assert (root / "videos/chunk-000/observation.images.cam0/episode_000000.mp4").exists()
    assert (root / "videos/chunk-000/observation.images.cam0/episode_000001.mp4").exists()

    info = json.loads((root / "meta/info.json").read_text())
    assert info["codebase_version"] == "v2.1"
    assert info["total_episodes"] == 2 and info["total_frames"] == 6 and info["fps"] == 30
    assert set(FEATURES).issubset(info["features"].keys())

    episodes = [json.loads(line) for line in (root / "meta/episodes.jsonl").read_text().splitlines()]
    assert len(episodes) == 2
    assert episodes[0] == {"episode_index": 0, "tasks": ["pick the red block"], "length": 3}

    tasks = [json.loads(line) for line in (root / "meta/tasks.jsonl").read_text().splitlines()]
    assert tasks == [{"task_index": 0, "task": "pick the red block"}]

    ep_stats = [
        json.loads(line) for line in (root / "meta/episodes_stats.jsonl").read_text().splitlines()
    ]
    assert len(ep_stats) == 2
    for line in ep_stats:
        for feat in ("observation.state", "action", "replay.obj0", "observation.images.cam0",
                     "timestamp", "frame_index", "episode_index", "index", "task_index"):
            assert feat in line["stats"]
            for k in ("min", "max", "mean", "std", "count"):
                assert k in line["stats"][feat]


def test_loads_with_real_lerobot(tmp_path):
    pytest.importorskip("lerobot")
    from lerobot.datasets.v30.convert_dataset_v21_to_v30 import convert_dataset

    ds_root = _make_small_dataset(tmp_path, name="whatslab_ref")
    parent = ds_root.parent

    convert_dataset(repo_id="whatslab_ref", root=str(parent), push_to_hub=False)

    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    ds = LeRobotDataset("whatslab_ref", root=str(ds_root))
    assert ds.num_episodes == 2
    assert ds.num_frames == 6
    item = ds[0]
    assert item["observation.state"].shape[-1] == 7
    assert "observation.images.cam0" in item


def test_save_episode_rejects_second_chunk(tmp_path):
    """MINOR-5: the mini-writer only ever writes under `chunk-000`, so it
    must raise a clear error rather than silently mis-writing once
    `episode_index` would cross into a second chunk (`chunks_size`=1000)."""
    from whatslab.data import lerobot_schema as S

    rec = LeRobotRecorder(str(tmp_path / "ds"), FEATURES, fps=30)
    rec._ep = S.CHUNKS_SIZE  # simulate having already written a full chunk
    rec.add_frame(
        np.zeros(7, np.float32), np.zeros(7, np.float32),
        {"cam0": np.zeros((64, 64, 3), np.uint8)},
        {"obj0": np.zeros(7, np.float32)}, task="t",
    )
    with pytest.raises(NotImplementedError):
        rec.save_episode()
