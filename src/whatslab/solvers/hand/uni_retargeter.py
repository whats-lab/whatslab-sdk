from __future__ import annotations

import os
from typing import Dict, List, Optional

import numpy as np
import onnxruntime as ort

from whatslab.paths import models_root

_ASSETS = os.path.join(os.path.dirname(__file__), "assets")

_ALIAS = {
    "base_hand": "human",
    "orca_hand": "orca",
    "allegro_hand": "allegro",
    "tesollo_dg5f": "tesollo",
    "robotis_hx5_d20": "robotis",
}

_URDF_STEM = {
    "human": ("base_hand", "{side}"),
    "orca": ("orca_hand", "{side}"),
    "allegro": ("allegro_hand", "allegro_hand_{side}"),
    "tesollo": ("tesollo_dg5f", "dg5f_{side}"),
    "robotis": ("robotis_hx5_d20", "hx5_d20_{side}"),
}


class UniRetargeter:

    def __init__(self, hand_type: str, config_name: str = "base_hand",
                 urdf_root=None, onnx_path: Optional[str] = None,
                 tables_path: Optional[str] = None, threads: int = 1):
        self.hand_type = hand_type.lower()
        self.robot = _ALIAS.get(config_name, config_name)
        self._urdf_root = urdf_root

        tables_path = tables_path or os.path.join(_ASSETS, "uni_tables.npz")
        onnx_path = onnx_path or os.path.join(_ASSETS, "uni_all.onnx")
        t = np.load(tables_path, allow_pickle=False)
        k = f"{self.robot}:{self.hand_type}"
        if f"{k}:qtok" not in t:
            have = sorted({x.rsplit(":", 2)[0] for x in t.files
                           if ":qtok" in x})
            raise ValueError(f"표에 없는 손: {k!r}. 가능한 로봇: {have}. "
                             f"새 손은 onboard_urdf.py 로 표를 만들어라")
        self._feed = {
            "sgn": t[f"{k}:sgn"].astype(np.float32),
            "qtok": t[f"{k}:qtok"].astype(np.float32),
            "lo": t[f"{k}:lo"].astype(np.float32),
            "hi": t[f"{k}:hi"].astype(np.float32),
        }
        self.joint_names: List[str] = [str(n) for n in t[f"{k}:joints"]]
        hk = f"human:{self.hand_type}:joints"
        self._human_names: List[str] = [
            str(n) for n in t[hk if hk in t else "human:joints"]]
        self._hidx = {n: i for i, n in enumerate(self._human_names)}
        for i, n in enumerate(self._human_names):
            for pfx in ("left_", "right_"):
                if n.startswith(pfx):
                    self._hidx.setdefault(n[len(pfx):], i)

        so = ort.SessionOptions()
        so.intra_op_num_threads = threads
        so.inter_op_num_threads = 1
        self._sess = ort.InferenceSession(
            onnx_path, so, providers=["CPUExecutionProvider"])
        self._q_h = np.zeros((1, len(self._human_names)), dtype=np.float32)

    @property
    def urdf_path(self) -> Optional[str]:
        sub, stem = _URDF_STEM.get(self.robot, (None, None))
        if sub is None:
            return None
        root = self._urdf_root or models_root()
        name = stem.format(side=self.hand_type) + ".urdf"
        for cand in (os.path.join(root, sub, "urdf", name),
                     os.path.join(root, sub, name)):
            if os.path.exists(cand):
                return cand
        return None

    @property
    def human_joint_names(self) -> List[str]:
        return list(self._human_names)

    def compute(self, joint_angles: Dict[str, float]) -> np.ndarray:
        q = self._q_h
        q[:] = 0.0
        for name, val in joint_angles.items():
            i = self._hidx.get(name)
            if i is not None:
                q[0, i] = val
        out = self._sess.run(None, {"q_human": q, **self._feed})[0]
        return np.asarray(out[0], dtype=float)
