from typing import Optional

import torch
import torch.nn as nn
import torch.nn.functional as F

from .keyvector import KV_DIM

TIP = slice(0, 3)


def _nearest(a: torch.Tensor, b: torch.Tensor):
    with torch.no_grad():
        d = torch.cdist(a.detach(), b.detach())
        return d.argmin(1), d.argmin(0)


def chamfer_both(a: torch.Tensor, b: torch.Tensor) -> torch.Tensor:
    ia, ib = _nearest(a, b)
    return (((a - b[ia]) ** 2).sum(-1).mean()
            + ((a[ib] - b) ** 2).sum(-1).mean())


def chamfer_partial(a: torch.Tensor, b: torch.Tensor) -> torch.Tensor:
    ia, _ = _nearest(a, b)
    return ((a - b[ia]) ** 2).sum(-1).mean()


def coverage_loss(y: torch.Tensor, bank: torch.Tensor,
                  partial: bool = False) -> torch.Tensor:
    fn = chamfer_partial if partial else chamfer_both
    return sum(fn(y[:, i, TIP], bank[:, i, TIP]) for i in range(y.shape[1]))


def distance_loss(x: torch.Tensor, y: torch.Tensor) -> torch.Tensor:
    n = x.shape[1]
    out = torch.zeros((), dtype=x.dtype, device=x.device)
    for i in range(n):
        for j in range(i + 1, n):
            dx = torch.norm(x[:, i, TIP] - x[:, j, TIP], dim=-1)
            dy = torch.norm(y[:, i, TIP] - y[:, j, TIP], dim=-1)
            out = out + ((dy - dx) ** 2).mean()
    return out


def motion_loss_global(dx: torch.Tensor, dy: torch.Tensor) -> torch.Tensor:
    a = F.normalize(dx.reshape(-1, KV_DIM), dim=-1, eps=1e-5)
    b = F.normalize(dy.reshape(-1, KV_DIM), dim=-1, eps=1e-5)
    return -(a * b).sum(-1).mean()


def motion_loss_local(dx_a: torch.Tensor, dy_a: torch.Tensor,
                      dx_b: torch.Tensor, dy_b: torch.Tensor) -> torch.Tensor:
    def cos(u, v):
        un = F.normalize(u.reshape(-1, KV_DIM), dim=-1, eps=1e-5)
        vn = F.normalize(v.reshape(-1, KV_DIM), dim=-1, eps=1e-5)
        return (un * vn).sum(-1)
    return ((cos(dx_a, dx_b) - cos(dy_a, dy_b)) ** 2).mean()


def align_loss(y: torch.Tensor, target: torch.Tensor,
               tip_only: bool = False) -> torch.Tensor:
    if tip_only:
        return ((y[..., TIP] - target[..., TIP]) ** 2).sum(-1).mean()
    return ((y - target) ** 2).sum(-1).mean()


def bone_loss(x: torch.Tensor, y: torch.Tensor) -> torch.Tensor:
    hx = F.normalize(x[..., TIP] - x[..., 3:], dim=-1, eps=1e-9)
    hy = F.normalize(y[..., TIP] - y[..., 3:], dim=-1, eps=1e-9)
    return (1.0 - (hx * hy).sum(-1)).mean()


def flatness_loss(y_plus: torch.Tensor, y_minus: torch.Tensor,
                  y: torch.Tensor) -> torch.Tensor:
    return ((y_plus + y_minus - 2.0 * y) ** 2).mean()


def pinch_loss(x: torch.Tensor, y: torch.Tensor, threshold: float) -> torch.Tensor:
    out = torch.zeros((), dtype=x.dtype, device=x.device)
    for i in range(1, x.shape[1]):
        mask = torch.norm(x[:, 0, TIP] - x[:, i, TIP], dim=-1) < threshold
        if bool(mask.any()):
            out = out + ((y[mask, 0, TIP] - y[mask, i, TIP]) ** 2).sum(-1).mean()
    return out


def soft_pinch_loss(y: torch.Tensor, candidates: torch.Tensor,
                    top_k: int = 5, tau: float = 1.0) -> torch.Tensor:
    flat_y = y[..., TIP].reshape(y.shape[0], -1)
    flat_c = candidates[..., TIP].reshape(candidates.shape[0], -1)
    d = ((flat_y.unsqueeze(1) - flat_c.unsqueeze(0)) ** 2).sum(-1)
    k = min(top_k, d.shape[1])
    near, _ = torch.topk(-d, k, dim=1)
    near = -near
    w = torch.softmax(-near / tau, dim=1)
    return (w * near).sum(1).mean()


class ResidualAffine(nn.Module):

    def __init__(self, n_finger: int, dim: int = KV_DIM):
        super().__init__()
        self.weight = nn.Parameter(torch.zeros(n_finger, dim, dim))
        self.bias = nn.Parameter(torch.zeros(n_finger, dim))

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return x + torch.einsum("fij,bfj->bfi", self.weight, x) + self.bias


class AffineHandNet(nn.Module):

    def __init__(self, net: nn.Module, n_finger: int, dim: int = KV_DIM,
                 affine: Optional[ResidualAffine] = None):
        super().__init__()
        self.net = net
        self.affine = affine if affine is not None else ResidualAffine(n_finger, dim)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return self.net(self.affine(x))
