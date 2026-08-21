from __future__ import annotations

import json
from pathlib import Path

import imageio.v2 as imageio
import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq

from . import lerobot_schema as S


class LeRobotRecorder:
    def __init__(self, out_dir, features: dict, fps: int, robot_type: str = "orca_nero"):
        self.root = Path(out_dir)
        self.features = features
        self.fps = fps
        self.robot_type = robot_type

        (self.root / "meta").mkdir(parents=True, exist_ok=True)
        (self.root / "data/chunk-000").mkdir(parents=True, exist_ok=True)

        self._img_keys = [k for k, v in features.items() if v["dtype"] == "video"]
        self._vec_keys = [k for k in features if k not in self._img_keys]
        for k in self._img_keys:
            (self.root / f"videos/chunk-000/{k}").mkdir(parents=True, exist_ok=True)

        self._ep = 0
        self._global = 0
        self._tasks: dict[str, int] = {}
        self._ep_lines: list[dict] = []
        self._ep_stats_lines: list[dict] = []
        self._buf: list[dict] = []
        self._writers: dict[str, object] = {}
        self._img_stats: dict[str, S.ImageStats] = {}

    def _video_path(self, key: str, episode: int) -> Path:
        return self.root / f"videos/chunk-000/{key}/episode_{episode:06d}.mp4"

    def _data_path(self, episode: int) -> Path:
        return self.root / f"data/chunk-000/episode_{episode:06d}.parquet"

    def _open_writers(self) -> None:
        for k in self._img_keys:
            self._writers[k] = imageio.get_writer(
                self._video_path(k, self._ep),
                fps=self.fps,
                codec="libx264",
                pixelformat="yuv420p",
                macro_block_size=1,
            )
            self._img_stats[k] = S.ImageStats()

    def _close_writers(self) -> None:
        for w in self._writers.values():
            w.close()
        self._writers = {}

    def add_frame(self, state, action, images: dict, replay: dict, task: str) -> None:
        if self._ep >= S.CHUNKS_SIZE:
            raise NotImplementedError(
                f"mini-writer supports a single chunk (<{S.CHUNKS_SIZE} episodes); "
                f"episode {self._ep} exceeds chunks_size"
            )
        if self._img_keys and not self._writers:
            self._open_writers()
        for k in self._img_keys:
            frame = np.asarray(images[k.split(".")[-1]], dtype=np.uint8)
            self._writers[k].append_data(frame)
            self._img_stats[k].update(frame)
        self._buf.append(
            {
                "observation.state": np.asarray(state, dtype=np.float32),
                "action": np.asarray(action, dtype=np.float32),
                "replay": {k: np.asarray(v, dtype=np.float32) for k, v in replay.items()},
                "task": task,
            }
        )

    def _task_index(self, task: str) -> int:
        if task not in self._tasks:
            self._tasks[task] = len(self._tasks)
        return self._tasks[task]

    def _vec_value(self, frame: dict, feat: str) -> np.ndarray:
        if feat.startswith("replay."):
            return frame["replay"][feat.split(".", 1)[1]]
        return frame[feat]

    def discard_episode(self) -> None:
        self._close_writers()
        for k in self._img_keys:
            self._video_path(k, self._ep).unlink(missing_ok=True)
        self._data_path(self._ep).unlink(missing_ok=True)
        self._img_stats = {}
        self._buf = []

    def save_episode(self) -> None:
        if not self._buf:
            self.discard_episode()
            return
        self._close_writers()
        n = len(self._buf)

        stacks: dict[str, np.ndarray] = {}
        for feat in self._vec_keys:
            stacks[feat] = np.stack([self._vec_value(f, feat) for f in self._buf]).astype(np.float32)

        timestamp = np.array([i / self.fps for i in range(n)], dtype=np.float32)
        frame_index = np.arange(n, dtype=np.int64)
        episode_index = np.full(n, self._ep, dtype=np.int64)
        index = np.arange(self._global, self._global + n, dtype=np.int64)
        task_index = np.array([self._task_index(f["task"]) for f in self._buf], dtype=np.int64)

        arrays = {}
        for feat in self._vec_keys:
            width = stacks[feat].shape[1]
            arrays[feat] = pa.array(stacks[feat].tolist(), type=pa.list_(pa.float32(), width))
        arrays["timestamp"] = pa.array(timestamp.tolist(), type=pa.float32())
        arrays["frame_index"] = pa.array(frame_index.tolist(), type=pa.int64())
        arrays["episode_index"] = pa.array(episode_index.tolist(), type=pa.int64())
        arrays["index"] = pa.array(index.tolist(), type=pa.int64())
        arrays["task_index"] = pa.array(task_index.tolist(), type=pa.int64())
        pq.write_table(pa.table(arrays), self._data_path(self._ep))

        tasks_in_ep = sorted({f["task"] for f in self._buf})
        self._ep_lines.append({"episode_index": self._ep, "tasks": tasks_in_ep, "length": n})

        stats = {}
        for feat in self._vec_keys:
            stats[feat] = S._reduce_stats(stacks[feat].astype(np.float64))
        for k in self._img_keys:
            stats[k] = self._img_stats[k].result()
        stats["timestamp"] = S._reduce_stats(timestamp.astype(np.float64))
        stats["frame_index"] = S._reduce_stats(frame_index.astype(np.float64))
        stats["episode_index"] = S._reduce_stats(episode_index.astype(np.float64))
        stats["index"] = S._reduce_stats(index.astype(np.float64))
        stats["task_index"] = S._reduce_stats(task_index.astype(np.float64))
        self._ep_stats_lines.append({"episode_index": self._ep, "stats": stats})

        self._global += n
        self._ep += 1
        self._img_stats = {}
        self._buf = []

    def finalize(self) -> None:
        self._close_writers()
        info = S.build_info(
            self.features, self.fps, self.robot_type, self._ep, self._global, len(self._tasks)
        )
        (self.root / "meta/info.json").write_text(json.dumps(info, indent=4))

        with open(self.root / "meta/episodes.jsonl", "w") as fp:
            for line in self._ep_lines:
                fp.write(json.dumps(line) + "\n")

        with open(self.root / "meta/tasks.jsonl", "w") as fp:
            for t, i in sorted(self._tasks.items(), key=lambda x: x[1]):
                fp.write(json.dumps({"task_index": i, "task": t}) + "\n")

        with open(self.root / "meta/episodes_stats.jsonl", "w") as fp:
            for line in self._ep_stats_lines:
                fp.write(json.dumps(line) + "\n")
