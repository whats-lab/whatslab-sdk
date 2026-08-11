from typing import Dict, List, Sequence

import numpy as np
import pinocchio as pin
import torch
import torch.nn as nn

from .human_fk import FINGERS
from .keyvector import KV_DIM, HandKeyvector

AXIS_PROBE = 0.3


def joint_axis(model, data, jid: int) -> np.ndarray:
    q = pin.neutral(model)
    iq = int(model.joints[jid].idx_q)
    q[iq] = AXIS_PROBE
    pin.forwardKinematics(model, data, q)
    par = int(model.parents[jid])
    base = data.oMi[par].rotation @ model.jointPlacements[jid].rotation
    rel = base.T @ data.oMi[jid].rotation
    w = pin.log3(rel)
    n = float(np.linalg.norm(w))
    if abs(n - AXIS_PROBE) > 1e-6:
        raise ValueError("관절 %s 가 단순 회전이 아니다 (|w|=%.6f)"
                         % (model.names[jid], n))
    return w / n


def rodrigues(axis: torch.Tensor, angle: torch.Tensor) -> torch.Tensor:
    k = axis.view(1, 3, 1)
    kx = torch.zeros(1, 3, 3, dtype=axis.dtype, device=axis.device)
    kx[0, 0, 1], kx[0, 0, 2] = -axis[2], axis[1]
    kx[0, 1, 0], kx[0, 1, 2] = axis[2], -axis[0]
    kx[0, 2, 0], kx[0, 2, 1] = -axis[1], axis[0]
    s = torch.sin(angle).view(-1, 1, 1)
    c = torch.cos(angle).view(-1, 1, 1)
    eye = torch.eye(3, dtype=axis.dtype, device=axis.device).unsqueeze(0)
    return eye + s * kx + (1.0 - c) * (k @ k.transpose(1, 2) - eye)


class TorchKeyvectorFK(nn.Module):

    def __init__(self, kv: HandKeyvector, idx_q: Sequence[int],
                 joint_names: Sequence[str], dtype=torch.float64):
        super().__init__()
        model = kv.model
        data = model.createData()
        self.n_joint = int(model.njoints)
        self.parents = [int(model.parents[j]) for j in range(self.n_joint)]

        rot = np.zeros((self.n_joint, 3, 3))
        trans = np.zeros((self.n_joint, 3))
        axes = np.zeros((self.n_joint, 3))
        for j in range(1, self.n_joint):
            rot[j] = model.jointPlacements[j].rotation
            trans[j] = model.jointPlacements[j].translation
            if int(model.joints[j].nq) == 1:
                axes[j] = joint_axis(model, data, j)
        rot[0] = np.eye(3)

        want = {n: i for i, n in enumerate(joint_names)}
        self.slot = [-1] * self.n_joint
        for j in range(1, self.n_joint):
            n = model.names[j]
            if n in want:
                self.slot[j] = want[n]
            elif int(model.joints[j].nq) == 1:
                self.slot[j] = -2

        q0 = pin.neutral(model)
        self.fixed = [float(q0[int(model.joints[j].idx_q)])
                      if int(model.joints[j].nq) == 1 else 0.0
                      for j in range(self.n_joint)]

        self.fingers = list(kv.fingers)
        self.frames: Dict[str, List[int]] = {}
        fr_rot, fr_trans, fr_parent = [], [], []
        for f in self.fingers:
            ids = []
            for fid in list(kv.fids[f]) + [kv.prox_fids[f]]:
                ids.append(len(fr_parent))
                fr_parent.append(int(model.frames[fid].parent))
                fr_rot.append(model.frames[fid].placement.rotation)
                fr_trans.append(model.frames[fid].placement.translation)
            self.frames[f] = ids

        self.register_buffer("j_rot", torch.as_tensor(rot, dtype=dtype))
        self.register_buffer("j_trans", torch.as_tensor(trans, dtype=dtype))
        self.register_buffer("j_axis", torch.as_tensor(axes, dtype=dtype))
        self.register_buffer("f_rot", torch.as_tensor(np.asarray(fr_rot), dtype=dtype))
        self.register_buffer("f_trans", torch.as_tensor(np.asarray(fr_trans),
                                                        dtype=dtype))
        self.f_parent = fr_parent
        self.register_buffer("origin", torch.as_tensor(kv.origin, dtype=dtype))
        self.register_buffer("rot_t", torch.as_tensor(kv.rot.T.copy(), dtype=dtype))
        self.l_ref = float(kv.l_ref)
        self.dtype = dtype

    def forward(self, q_act: torch.Tensor) -> torch.Tensor:
        b = q_act.shape[0]
        dev = q_act.device
        eye = torch.eye(3, dtype=q_act.dtype, device=dev).expand(b, 3, 3)
        zero = torch.zeros(b, 3, dtype=q_act.dtype, device=dev)
        rot = [eye]
        pos = [zero]
        for j in range(1, self.n_joint):
            jr = self.j_rot[j].to(q_act.dtype).unsqueeze(0)
            jt = self.j_trans[j].to(q_act.dtype).unsqueeze(0)
            s = self.slot[j]
            if s >= 0:
                local = jr @ rodrigues(self.j_axis[j].to(q_act.dtype), q_act[:, s])
            elif s == -2:
                ang = torch.full((b,), self.fixed[j], dtype=q_act.dtype, device=dev)
                local = jr @ rodrigues(self.j_axis[j].to(q_act.dtype), ang)
            else:
                local = jr
            p = self.parents[j]
            rot.append(rot[p] @ local)
            pos.append(pos[p] + (rot[p] @ jt.unsqueeze(-1)).squeeze(-1))

        out = []
        for f in self.fingers:
            pts = []
            for k in self.frames[f]:
                pj = self.f_parent[k]
                off = self.f_trans[k].to(q_act.dtype).view(1, 3, 1)
                pts.append(pos[pj] + (rot[pj] @ off).squeeze(-1))
            prox = pts[-1]
            tip = pts[-2]
            out.append(torch.stack([self._local(tip), self._local(prox)], dim=1))
        return torch.cat(out, dim=1).view(-1, len(self.fingers), KV_DIM)

    def _local(self, p: torch.Tensor) -> torch.Tensor:
        r = self.rot_t.to(p.dtype)
        o = self.origin.to(p.dtype)
        return ((p - o) @ r.T) / self.l_ref
