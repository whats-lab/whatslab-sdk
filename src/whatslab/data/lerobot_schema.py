"""LeRobot v2.1 메타 빌더 — Task 1b 레퍼런스(`tests/fixtures/lerobot_ref_v21/meta/info.json`)의
실제 on-disk 구조를 미러한다. 필드명/값은 그 레퍼런스에서 그대로 확정했다."""
from __future__ import annotations

import numpy as np

STANDARD_COLUMNS = ["timestamp", "frame_index", "episode_index", "index", "task_index"]
CODEBASE_VERSION = "v2.1"
CHUNKS_SIZE = 1000

DATA_PATH_TEMPLATE = "data/chunk-{episode_chunk:03d}/episode_{episode_index:06d}.parquet"
VIDEO_PATH_TEMPLATE = "videos/chunk-{episode_chunk:03d}/{video_key}/episode_{episode_index:06d}.mp4"


def build_info(
    features: dict,
    fps: int,
    robot_type: str,
    total_episodes: int,
    total_frames: int,
    total_tasks: int,
) -> dict:
    """레퍼런스 info.json 구조를 그대로 재현한다 (키 이름/중첩 구조 동일)."""
    feat: dict = {}
    for k, v in features.items():
        entry = {"dtype": v["dtype"], "shape": list(v["shape"]), "names": v.get("names")}
        if v["dtype"] == "video":
            h, w, c = v["shape"]
            entry["info"] = {
                "video.height": h,
                "video.width": w,
                "video.codec": "libx264",
                "video.pix_fmt": "yuv420p",
                "video.is_depth_map": False,
                "video.fps": fps,
                "video.channels": c,
                "has_audio": False,
            }
        feat[k] = entry
    for c in STANDARD_COLUMNS:
        feat[c] = {
            "dtype": "float32" if c == "timestamp" else "int64",
            "shape": [1],
            "names": None,
        }
    total_videos = total_episodes * sum(1 for v in features.values() if v["dtype"] == "video")
    return {
        "codebase_version": CODEBASE_VERSION,
        "robot_type": robot_type,
        "total_episodes": total_episodes,
        "total_frames": total_frames,
        "total_tasks": total_tasks,
        "total_videos": total_videos,
        "total_chunks": 1,
        "chunks_size": CHUNKS_SIZE,
        "fps": fps,
        "splits": {"train": f"0:{total_episodes}"},
        "data_path": DATA_PATH_TEMPLATE,
        "video_path": VIDEO_PATH_TEMPLATE,
        "features": feat,
    }


def _reduce_stats(arr: np.ndarray) -> dict:
    """arr: (n, *dims) float64 배열 -> {min,max,mean,std,count} (dims 별 리스트, count=[n])."""
    n = arr.shape[0]
    mn = arr.min(axis=0)
    mx = arr.max(axis=0)
    mean = arr.mean(axis=0)
    std = arr.std(axis=0)
    return {
        "min": np.atleast_1d(mn).tolist(),
        "max": np.atleast_1d(mx).tolist(),
        "mean": np.atleast_1d(mean).tolist(),
        "std": np.atleast_1d(std).tolist(),
        "count": [n],
    }


def _reduce_image_stats(frames: np.ndarray) -> dict:
    """frames: (n, H, W, C) uint8 -> per-channel stats, normalized to [0,1],
    nested as [[[v]], [[v]], [[v]]] (채널별로 depth-3 중첩) — 레퍼런스와 동일 구조."""
    x = frames.astype(np.float64) / 255.0
    c = x.shape[-1]
    flat = x.reshape(-1, c)
    mn = flat.min(axis=0)
    mx = flat.max(axis=0)
    mean = flat.mean(axis=0)
    std = flat.std(axis=0)
    nest = lambda v: [[[float(x)]] for x in v]
    n = frames.shape[0]
    return {
        "min": nest(mn),
        "max": nest(mx),
        "mean": nest(mean),
        "std": nest(std),
        "count": [n],
    }
