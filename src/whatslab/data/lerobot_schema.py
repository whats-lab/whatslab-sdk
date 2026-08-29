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


class ImageStats:
    # uint8 프레임의 가능한 값은 256개뿐이므로, 픽셀을 전부 훑는 대신 채널별
    # 256-bin 히스토그램에서 sum/sumsq/min/max 를 정확히 유도할 수 있다.
    _LEVELS = np.arange(256, dtype=np.float64) / 255.0
    _LEVELS_SQ = _LEVELS * _LEVELS

    def __init__(self) -> None:
        self.frames = 0
        self.pixels = 0
        self._sum: np.ndarray | None = None
        self._sumsq: np.ndarray | None = None
        self._min: np.ndarray | None = None
        self._max: np.ndarray | None = None

    @classmethod
    def _moments_u8(cls, flat: np.ndarray):
        """uint8 전용 경로 — float64 사본(프레임당 수 MB)을 만들지 않는다.

        640x480x3 3대 기준 45.6ms -> 2.4ms (상대오차 1e-13, 256항만 더하므로
        전 픽셀 누산보다 오히려 반올림 오차가 작다).
        """
        c = flat.shape[1]
        s = np.empty(c, dtype=np.float64)
        q = np.empty(c, dtype=np.float64)
        mn = np.empty(c, dtype=np.float64)
        mx = np.empty(c, dtype=np.float64)
        for j in range(c):
            h = np.bincount(flat[:, j], minlength=256).astype(np.float64)
            s[j] = h @ cls._LEVELS
            q[j] = h @ cls._LEVELS_SQ
            nz = np.nonzero(h)[0]
            mn[j] = cls._LEVELS[nz[0]]
            mx[j] = cls._LEVELS[nz[-1]]
        return s, q, mn, mx

    def update(self, frame: np.ndarray) -> None:
        raw = np.asarray(frame)
        if raw.dtype == np.uint8:
            flat = raw.reshape(-1, raw.shape[-1])
            s, q, mn, mx = self._moments_u8(flat)
        else:
            x = np.asarray(raw, dtype=np.float64) / 255.0
            flat = x.reshape(-1, x.shape[-1])
            s = flat.sum(axis=0)
            q = np.square(flat).sum(axis=0)
            mn = flat.min(axis=0)
            mx = flat.max(axis=0)
        if self._sum is None:
            self._sum, self._sumsq, self._min, self._max = s, q, mn, mx
        else:
            self._sum += s
            self._sumsq += q
            self._min = np.minimum(self._min, mn)
            self._max = np.maximum(self._max, mx)
        self.frames += 1
        self.pixels += flat.shape[0]

    def result(self) -> dict:
        if self._sum is None:
            raise ValueError("ImageStats.result() before any update()")
        mean = self._sum / self.pixels
        var = np.maximum(self._sumsq / self.pixels - np.square(mean), 0.0)
        nest = lambda v: [[[float(x)]] for x in v]
        return {
            "min": nest(self._min),
            "max": nest(self._max),
            "mean": nest(mean),
            "std": nest(np.sqrt(var)),
            "count": [self.frames],
        }
