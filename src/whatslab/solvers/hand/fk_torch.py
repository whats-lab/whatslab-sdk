from typing import Sequence

import numpy as np
import torch

from .human_fk import FINGERS
from .keyvector import KV_DIM, HandKeyvector


class _KeyvectorFKFn(torch.autograd.Function):

    @staticmethod
    def forward(ctx, q_act, fk):
        qa = q_act.detach().cpu().numpy().astype(float)
        out = np.empty((qa.shape[0], len(FINGERS), KV_DIM))
        jac = np.empty((qa.shape[0], len(FINGERS), KV_DIM, qa.shape[1]))
        for b in range(qa.shape[0]):
            q = fk.expand(qa[b])
            out[b] = fk.kv.encode(q)
            jac[b] = fk.kv.jacobian(q, fk.idx_v)
        ctx.save_for_backward(torch.as_tensor(jac, dtype=q_act.dtype,
                                              device=q_act.device))
        return torch.as_tensor(out, dtype=q_act.dtype, device=q_act.device)

    @staticmethod
    def backward(ctx, grad_out):
        (jac,) = ctx.saved_tensors
        return torch.einsum("bfk,bfkj->bj", grad_out.contiguous(), jac), None


class KeyvectorFK(torch.nn.Module):

    def __init__(self, kv: HandKeyvector, idx_q: Sequence[int],
                 idx_v: Sequence[int], q_template: np.ndarray):
        super().__init__()
        self.kv = kv
        self.idx_q = np.asarray(idx_q, dtype=int)
        self.idx_v = np.asarray(idx_v, dtype=int)
        if self.idx_q.size != self.idx_v.size:
            raise ValueError("idx_q %d 개와 idx_v %d 개가 다르다 — 형상 인덱스와"
                             " 속도 인덱스를 섞었다"
                             % (self.idx_q.size, self.idx_v.size))
        self._q0 = np.asarray(q_template, dtype=float).copy()

    def expand(self, q_act: np.ndarray) -> np.ndarray:
        q = self._q0.copy()
        q[self.idx_q] = q_act
        return q

    def forward(self, q_act: torch.Tensor) -> torch.Tensor:
        return _KeyvectorFKFn.apply(q_act, self)


def keyvector_fk(q_act: torch.Tensor, fk: KeyvectorFK) -> torch.Tensor:
    return fk(q_act)
