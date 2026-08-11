import os
from typing import List, Mapping, Optional, Sequence

import numpy as np
import pinocchio as pin
import torch
import torch.nn as nn

from .hand_configs import CONFIG_REGISTRY
from .human_fk import FINGERS, HumanHandFK
from .keyvector import (FRAME_DIM, KV_DIM, MIRROR_Z, HandKeyvector,
                        finger_columns, human_chains, mirror_frames,
                        sensor_chains, sensor_prox)
from .net_losses import unit_to_joint

LIMIT_FALLBACK = 2.0
ACTS = {"leaky": nn.LeakyReLU, "gelu": nn.GELU, "silu": nn.SiLU,
        "tanh": nn.Tanh, "softplus": nn.Softplus}
NORMS = ("none", "layer", "batch")
INPUTS = {"kv": KV_DIM, "frames": FRAME_DIM}
LAYER_REMAP = {"%snets.%d.%d.%s" % (p, f, a, w):
               "%snets.%d.%d.%s" % (p, f, b, w)
               for p in ("", "net.") for f in range(5)
               for a, b in ((2, 3), (4, 6)) for w in ("weight", "bias")}
DORSUM_FRAME = "{side}_sensor_dorsum"


class HandNet(nn.Module):

    def __init__(self, in_dim: int, joint_counts: Sequence[int], hidden: int = 128,
                 dropout: float = 0.0, layers: int = 2, act: str = "leaky",
                 norm: str = "none"):
        super().__init__()
        if act not in ACTS:
            raise ValueError("act 는 %s 중 하나: %r" % (list(ACTS), act))
        if norm not in NORMS:
            raise ValueError("norm 는 %s 중 하나: %r" % (list(NORMS), norm))
        self.act = act
        self.norm = norm
        self.in_dim = int(in_dim)
        self.joint_counts = [int(n) for n in joint_counts]
        self.dropout = float(dropout)
        self.layers = int(layers)
        self._hidden = int(hidden)
        if self.layers < 1:
            raise ValueError("layers 는 1 이상: %d" % self.layers)
        self.nets = nn.ModuleList([self._stack(n) for n in self.joint_counts])

    def _stack(self, n_out: int) -> nn.Module:
        mods: List[nn.Module] = []
        d = self.in_dim
        for _ in range(self.layers):
            mods += [nn.Linear(d, self.hidden_of())]
            if self.norm == "layer":
                mods.append(nn.LayerNorm(self.hidden_of()))
            elif self.norm == "batch":
                mods.append(nn.BatchNorm1d(self.hidden_of()))
            mods += [ACTS[self.act](), nn.Dropout(self.dropout)]
            d = self.hidden_of()
        mods += [nn.Linear(d, n_out), nn.Tanh()]
        return nn.Sequential(*mods)

    def hidden_of(self) -> int:
        return self._hidden

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return torch.cat([net(x[:, i]) for i, net in enumerate(self.nets)], dim=1)


class NetHandRetargeter:

    def __init__(self, hand_type: str, config_name: str = "base_hand",
                 checkpoint: Optional[str] = None, urdf_root: Optional[str] = None,
                 hidden: int = 128, mirror_to: Optional[str] = None,
                 dropout: float = 0.0, layers: int = 2, act: str = "leaky",
                 norm: str = "none", input_mode: str = "kv"):
        if input_mode not in INPUTS:
            raise ValueError("input_mode 는 %s 중 하나: %r"
                             % (list(INPUTS), input_mode))
        if config_name not in CONFIG_REGISTRY:
            raise ValueError("알 수 없는 config '%s'. 가능: %s"
                             % (config_name, list(CONFIG_REGISTRY)))
        config = CONFIG_REGISTRY[config_name](urdf_root=urdf_root)
        self.hand_type = hand_type.lower()
        self.config_name = config_name
        self.mirror_to = None if mirror_to is None else mirror_to.lower()
        self.input_mode = input_mode

        root = getattr(config, "_models_root", None)
        fk_urdf = (os.path.join(root, "base_hand", "urdf", "%s.urdf" % self.hand_type)
                   if root else None)
        declared = getattr(config, "_HUMAN_CHAIN", None) or {}
        self.fingers = [f for f in FINGERS if f in declared] or list(FINGERS)
        self.fk = HumanHandFK(self.hand_type, urdf_path=fk_urdf)
        self.human_joint_names: List[str] = list(self.fk.joint_names)
        self.hkv = HandKeyvector(
            self.fk.model, self.fk.data, human_chains(self.fk, self.fingers),
            DORSUM_FRAME.format(side=self.hand_type),
            sensor_prox(self.fk.model, self.hand_type, self.fingers))

        self.urdf_path = config._get_urdf_path(self.hand_type)
        self.model = pin.buildModelFromUrdf(self.urdf_path)
        self.data = self.model.createData()
        chains = sensor_chains(self.model, self.hand_type, fingers=self.fingers)
        self.kv = HandKeyvector(
            self.model, self.data, chains,
            DORSUM_FRAME.format(side=self.hand_type),
            sensor_prox(self.model, self.hand_type, self.fingers))

        self._cols = finger_columns(self.model, {f: self.kv.fids[f][-1]
                                                for f in self.fingers})
        order = [c for f in self.fingers for c in self._cols[f]]
        by_v = {int(self.model.joints[j].idx_v): j for j in range(1, self.model.njoints)
                if self.model.joints[j].nq > 0}
        self._iv = [int(c) for c in order]
        self._iq = [int(self.model.joints[by_v[c]].idx_q) for c in order]
        self.joint_names = [self.model.names[by_v[c]] for c in order]

        lo = np.where(np.isfinite(self.model.lowerPositionLimit),
                      self.model.lowerPositionLimit, -LIMIT_FALLBACK)
        hi = np.where(np.isfinite(self.model.upperPositionLimit),
                      self.model.upperPositionLimit, LIMIT_FALLBACK)
        self.lower = lo[self._iq].copy()
        self.upper = hi[self._iq].copy()

        self.net = HandNet(INPUTS[input_mode],
                           [len(self._cols[f]) for f in self.fingers],
                           hidden=hidden, dropout=dropout, layers=layers,
                           act=act, norm=norm).double()
        self.net.eval()
        self.u_margin = 1.0
        if checkpoint is not None:
            self.load(checkpoint)
        self._q = pin.neutral(self.model)

    @property
    def _r_origin(self) -> np.ndarray:
        return self.kv.origin

    @property
    def _r_frame(self) -> np.ndarray:
        return self.kv.rot

    def _check_side(self, path: str, sd) -> None:
        if not isinstance(sd, dict):
            return
        want = self.mirror_to or self.hand_type
        got = sd.get("side")
        if got is not None and str(got).lower() != want:
            raise ValueError(
                "체크포인트 side 가 다르다: %s 는 '%s' 로 학습됐는데 '%s' 로 쓰려 한다."
                " 반대 손 모델을 쓰려면 mirror_to='%s' 를 명시해라 (사람 p95 25mm +"
                " 로봇 미러 오차를 감수하는 선택이다)" % (path, got, want, got))
        cfg = sd.get("config")
        if cfg is not None and str(cfg) != self.config_name:
            raise ValueError("체크포인트 config 가 다르다: %s 는 '%s' 용인데 '%s' 로"
                             " 쓰려 한다" % (path, cfg, self.config_name))

    def load(self, checkpoint: str) -> None:
        sd = torch.load(checkpoint, map_location="cpu")
        inner = sd["net"] if "net" in sd else sd
        self.u_margin = float(sd.get("u_margin", 1.0)) if isinstance(sd, dict) else 1.0
        self._check_side(checkpoint, sd)
        if isinstance(sd, dict) and sd.get("act") is not None:
            self._want_act = str(sd["act"])
        if isinstance(sd, dict) and sd.get("norm") is not None:
            self._want_norm = str(sd["norm"])
        if isinstance(sd, dict) and sd.get("input_mode") is not None:
            got = str(sd["input_mode"])
            if got != self.input_mode:
                raise ValueError("체크포인트 input_mode 가 다르다: %s 는 '%s' 인데"
                                 " '%s' 로 만들었다" % (checkpoint, got,
                                                     self.input_mode))
        if not (isinstance(sd, dict) and "dropout" in sd):
            inner = {LAYER_REMAP.get(k, k): v for k, v in inner.items()}
        self._rebuild_for(inner)
        want = next(iter(inner.values())).dtype
        self.net = self.net.to(dtype=want)
        self.net.load_state_dict(inner)
        self.net.eval()

    def _rebuild_for(self, inner) -> None:
        pre = ""
        first = inner.get("nets.0.0.weight")
        if first is None:
            return
        h = int(first.shape[0])
        lin = [k for k in inner if k.startswith("%snets.0." % pre)
               and k.endswith(".weight") and inner[k].dim() == 2]
        nl = len(lin) - 1
        cur = self.net
        act = getattr(self, "_want_act", cur.act)
        norm = getattr(self, "_want_norm", cur.norm)
        if (h == cur._hidden and nl == cur.layers and act == cur.act
                and norm == cur.norm):
            return
        self.net = HandNet(INPUTS[input_mode],
                           [len(self._cols[f]) for f in self.fingers],
                           hidden=h, dropout=cur.dropout, layers=nl,
                           act=act, norm=norm).double()

    def state_dict(self):
        return self.net.state_dict()

    def reset(self) -> None:
        self._q = pin.neutral(self.model)

    def to_joint(self, unit) -> np.ndarray:
        u = np.asarray(unit, dtype=float)
        return unit_to_joint(u, self.lower, self.upper, self.u_margin)

    def encode_human(self, joint_angles: Mapping[str, float]) -> np.ndarray:
        q = self.fk.q_from_named(joint_angles)
        flip = self.mirror_to is not None and self.mirror_to != self.hand_type
        if self.input_mode == "frames":
            x = self.hkv.encode_frames(q)
            return mirror_frames(x) if flip else x
        x = self.hkv.encode(q)
        return x * MIRROR_Z if flip else x

    @property
    def dtype(self) -> torch.dtype:
        return next(self.net.parameters()).dtype

    def compute(self, joint_angles: Mapping[str, float]) -> np.ndarray:
        x = self.encode_human(joint_angles)
        with torch.no_grad():
            unit = self.net(torch.as_tensor(x, dtype=self.dtype).unsqueeze(0))
        q_act = self.to_joint(unit.numpy()[0])
        self._q = pin.neutral(self.model)
        self._q[self._iq] = q_act
        return q_act
