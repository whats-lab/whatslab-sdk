import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F

from .keyvector import KV_DIM

TIP = slice(0, 3)
U_MARGIN = 1.25
U_LEAK = 0.01


def unit_to_joint(u, lower, upper, margin: float = 1.0):
    half = (upper - lower) * 0.5
    v = lower + half + margin * u * half
    if margin <= 1.0:
        return v
    if isinstance(v, torch.Tensor):
        c = torch.maximum(torch.minimum(v, upper), lower)
    else:
        c = np.clip(v, lower, upper)
    return c + U_LEAK * (v - c)


def _nearest(a: torch.Tensor, b: torch.Tensor):
    with torch.no_grad():
        d = torch.cdist(a.detach(), b.detach())
        return d.argmin(1), d.argmin(0)


def chamfer_both(a: torch.Tensor, b: torch.Tensor) -> torch.Tensor:
    ia, ib = _nearest(a, b)
    return (((a - b[ia]) ** 2).sum(-1).mean()
            + ((a[ib] - b) ** 2).sum(-1).mean())


def coverage_loss(y: torch.Tensor, bank: torch.Tensor) -> torch.Tensor:
    return sum(chamfer_both(y[:, i, TIP], bank[:, i, TIP])
               for i in range(y.shape[1]))


def position_loss(x: torch.Tensor, y: torch.Tensor,
                  tip_only: bool = False) -> torch.Tensor:
    if tip_only:
        return ((y[..., TIP] - x[..., TIP]) ** 2).sum(-1).mean()
    return ((y - x) ** 2).sum(-1).mean()


def motion_loss_global(dx: torch.Tensor, dy: torch.Tensor) -> torch.Tensor:
    a = F.normalize(dx.reshape(-1, KV_DIM), dim=-1, eps=1e-5)
    b = F.normalize(dy.reshape(-1, KV_DIM), dim=-1, eps=1e-5)
    return -(a * b).sum(-1).mean()

def bone_loss(x: torch.Tensor, y: torch.Tensor) -> torch.Tensor:
    hx = F.normalize(x[..., TIP] - x[..., 3:], dim=-1, eps=1e-9)
    hy = F.normalize(y[..., TIP] - y[..., 3:], dim=-1, eps=1e-9)
    return (1.0 - (hx * hy).sum(-1)).mean()

def orientation_loss(a_robot: torch.Tensor, a_human: torch.Tensor,
                     offset: torch.Tensor) -> torch.Tensor:
    want = offset.unsqueeze(0) @ a_human
    return ((a_robot - want) ** 2).sum((-2, -1)).mean() * 0.25


def saturation_loss(u: torch.Tensor, knee: float = 0.9) -> torch.Tensor:
    return (F.relu(u.abs() - knee) ** 2).mean()


def posture_loss(u: torch.Tensor) -> torch.Tensor:
    return (u.mean(0) ** 2).mean()


def pinch_loss(x: torch.Tensor, y: torch.Tensor, threshold: float) -> torch.Tensor:
    out = torch.zeros((), dtype=x.dtype, device=x.device)
    for i in range(1, x.shape[1]):
        mask = torch.norm(x[:, 0, TIP] - x[:, i, TIP], dim=-1) < threshold
        if bool(mask.any()):
            out = out + ((y[mask, 0, TIP] - y[mask, i, TIP]) ** 2).sum(-1).mean()
    return out
