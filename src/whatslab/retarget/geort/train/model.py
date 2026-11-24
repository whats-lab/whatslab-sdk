from __future__ import annotations

import numpy as np
import torch
import torch.nn.functional as F  # noqa: N812
from torch import nn

from .chamfer import chamfer_distance
from .human import CHAINS, FINGERS, TIPS

HUMAN = "human"
POINTS = 21
INPUT_DIM = (POINTS - 1) * 3
CLOUD = 2048
LN_EPS = 1e-5
CHAIN_WEIGHTS = (0.0, 1.0 / 3.0, 2.0 / 3.0, 1.0)
WRIST = 0
INDEX_MCP = 5
MIDDLE_MCP = 9
RING_MCP = 13


Frame = tuple[torch.Tensor, torch.Tensor]


def cross_matrix(axis: torch.Tensor) -> torch.Tensor:
    x, y, z = axis.tolist()
    return torch.tensor([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]], dtype=axis.dtype)


class ExactFK:
    def __init__(self, kin: dict, device: str) -> None:
        f32 = torch.float32
        self.device = device
        inv = np.linalg.inv(np.array(kin["palm"], dtype=np.float64))
        self.base = (
            torch.tensor(inv[:3, :3], dtype=f32, device=device),
            torch.tensor(inv[:3, 3], dtype=f32, device=device),
        )
        self.fixed = {}
        for name, t in kin["fixed"].items():
            m = inv @ np.array(t, dtype=np.float64)
            self.fixed[name] = (
                torch.tensor(m[:3, :3], dtype=f32, device=device),
                torch.tensor(m[:3, 3], dtype=f32, device=device),
            )
        self.lower = torch.tensor(kin["lower"], dtype=f32, device=device)
        self.upper = torch.tensor(kin["upper"], dtype=f32, device=device)
        self.fingers = []
        for finger in kin["fingers"]:
            steps = []
            for s in finger["steps"]:
                origin = torch.tensor(s["origin"], dtype=f32)
                k = None
                if s["axis"] is not None:
                    k = cross_matrix(torch.tensor(s["axis"], dtype=f32))
                    k = (k.to(device), (k @ k).to(device))
                rotation, shift = origin[:3, :3].to(device), origin[:3, 3].to(device)
                steps.append((s["child"], rotation, shift, k, s["index"], s["scale"], s["bias"]))
            offset = torch.tensor(finger["offset"], dtype=f32, device=device)
            self.fingers.append((steps, offset))
        self.eye = torch.eye(3, device=device)

    def unnormalize(self, qn: torch.Tensor) -> torch.Tensor:
        return (qn * 0.5 + 0.5) * (self.upper - self.lower) + self.lower

    def frames(
        self, q: torch.Tensor, wanted: set[str] | None = None
    ) -> tuple[torch.Tensor, dict[str, Frame]]:
        b = q.shape[0]
        out: dict[str, Frame] = {}
        tips = []
        for steps, offset in self.fingers:
            r, p = self.base[0].expand(b, 3, 3), self.base[1].expand(b, 3)
            for child, rotation, shift, k, index, scale, bias in steps:
                p = p + r @ shift
                r = r @ rotation
                if k is not None:
                    angle = (q[:, index] * scale + bias)[:, None, None]
                    r = r @ (self.eye + torch.sin(angle) * k[0] + (1.0 - torch.cos(angle)) * k[1])
                if wanted is not None and child in wanted:
                    out[child] = (r, p)
            tips.append(r @ offset + p)
        if wanted is not None:
            for name in wanted & set(self.fixed):
                rotation, shift = self.fixed[name]
                out[name] = (rotation.expand(b, 3, 3), shift.expand(b, 3))
        return torch.stack(tips, dim=1), out

    def __call__(self, q: torch.Tensor) -> torch.Tensor:
        return self.frames(q)[0]

    def random_q(self, n: int) -> torch.Tensor:
        span = self.upper - self.lower
        return torch.rand(n, len(self.lower), device=self.device) * span + self.lower


def sliced(frames: dict[str, Frame], n: int) -> dict[str, Frame]:
    return {k: (r[:n], p[:n]) for k, (r, p) in frames.items()}


def link_points(
    frames: dict[str, Frame], points: dict[str, torch.Tensor], names: list[str]
) -> torch.Tensor:
    parts = [
        torch.einsum("bij,pj->bpi", frames[n][0], points[n]) + frames[n][1][:, None] for n in names
    ]
    return torch.cat(parts, dim=1)


class Collision:
    def __init__(self, spec: dict | None, arrays: dict, prefix: str, device: str) -> None:
        self.pairs = [] if spec is None else [(a, b) for a, b in spec["pairs"]]
        self.links = set() if spec is None else set(spec["links"])
        self.margin = 0.0 if spec is None else float(spec["margin_m"])
        self.device = device
        self.points = {
            n: torch.tensor(arrays[f"{prefix}{n}"], dtype=torch.float32, device=device)
            for n in self.links
        }

    def loss(self, frames: dict[str, Frame]) -> torch.Tensor:
        loss = torch.zeros((), device=self.device)
        for a, b in self.pairs:
            d = torch.cdist(
                link_points(frames, self.points, a), link_points(frames, self.points, b)
            )
            loss = loss + (F.relu(self.margin - d) ** 2).sum(dim=(1, 2)).mean()
        return loss


class PromptNet(nn.Module):
    def __init__(
        self,
        heads: dict[str, int],
        width: int,
        latent: int,
        prompt_dim: int,
        enc_layers: int,
        dec_layers: int,
    ) -> None:
        super().__init__()
        self.width = width
        dims = [INPUT_DIM] + [width] * enc_layers + [latent]
        self.encoder = nn.ModuleList(
            [nn.Linear(a, b) for a, b in zip(dims[:-1], dims[1:], strict=True)]
        )
        self.dec_in = nn.Linear(latent, width)
        self.dec_prompt = nn.Linear(prompt_dim, width, bias=False)
        self.decoder = nn.ModuleList([nn.Linear(width, width) for _ in range(dec_layers)])
        self.norms = nn.ModuleList([nn.LayerNorm(width, eps=LN_EPS) for _ in range(dec_layers)])
        self.film = nn.Linear(prompt_dim, 2 * width * dec_layers)
        nn.init.normal_(self.film.weight, std=0.01)
        nn.init.zeros_(self.film.bias)
        self.prompts = nn.ParameterDict(
            {n: nn.Parameter(torch.randn(prompt_dim) * 0.5) for n in heads}
        )
        self.heads = nn.ModuleDict({n: nn.Linear(width, d) for n, d in heads.items()})

    def encode(self, x: torch.Tensor) -> torch.Tensor:
        for k, layer in enumerate(self.encoder):
            x = layer(x)
            if k < len(self.encoder) - 1:
                x = F.silu(x)
        return x

    def decode(self, z: torch.Tensor, name: str) -> torch.Tensor:
        p = self.prompts[name]
        h = F.silu(self.dec_in(z) + self.dec_prompt(p))
        film = self.film(p).view(len(self.decoder), 2, self.width)
        for k, layer in enumerate(self.decoder):
            u = self.norms[k](layer(h)) * (1.0 + film[k, 0]) + film[k, 1]
            h = h + F.silu(u)
        return self.heads[name](h)

    def own_params(self, name: str) -> list[nn.Parameter]:
        return [self.prompts[name], *self.heads[name].parameters()]


class Robot:
    def __init__(self, entry: dict, arrays: dict, device: str) -> None:
        self.name = entry["name"]
        kin = entry["kinematics"]
        self.kin = kin
        self.fk = ExactFK(kin, device)
        self.scale = float(entry["scale"])
        self.tip_scale = float(entry["tip_scale"])
        self.abd_scale = float(entry["abd_scale"])
        self.prior = torch.tensor(arrays[f"prior_{self.name}"], dtype=torch.float32, device=device)
        self.mask = torch.tensor(arrays[f"mask_{self.name}"], dtype=torch.float32, device=device)
        self.span = self.fk.upper - self.fk.lower
        self.finger_joints = [
            torch.tensor(
                [s["index"] for s in f["steps"] if s["index"] >= 0],
                dtype=torch.long,
                device=device,
            )
            for f in kin["fingers"]
        ]
        names = [f["name"] for f in kin["fingers"]]
        self.human_fingers = torch.tensor(
            [FINGERS.index(n) for n in names], dtype=torch.long, device=device
        )
        self.tip_ids = [TIPS[FINGERS.index(n)] for n in names]
        self.thumb = names.index(FINGERS[0])
        self.others = [i for i in range(len(names)) if i != self.thumb]
        self.tip_weights = torch.ones(len(names), device=device)
        self.collision = Collision(entry["collision"], arrays, f"coll_{self.name}_", device)
        self.abd_links: list[tuple[str, str, tuple[int, ...]]] = []
        for i, f in enumerate(kin["fingers"]):
            if i != self.thumb:
                mcp = entry["mcp_links"][i]
                children = [s["child"] for s in f["steps"] if s["axis"] is not None]
                after = children[children.index(mcp) + 1]
                self.abd_links.append((mcp, after, CHAINS[FINGERS.index(f["name"])]))
        self.wanted = {n for a, b, _ in self.abd_links for n in (a, b)} | self.collision.links
        self.abd_human = torch.tensor(
            [c[:2] for _, _, c in self.abd_links], dtype=torch.long, device=device
        ).reshape(-1, 2)

    def q(self, raw: torch.Tensor) -> torch.Tensor:
        return self.fk.unnormalize(torch.tanh(raw))


def chain_matrix(device: str) -> torch.Tensor:
    m = torch.zeros(len(CHAINS), POINTS, device=device)
    for f, chain in enumerate(CHAINS):
        for p, w in zip(chain, CHAIN_WEIGHTS, strict=True):
            m[f, p] = w
    return m


def flat(points: torch.Tensor) -> torch.Tensor:
    return points[:, 1:].reshape(len(points), INPUT_DIM)


def mixed(
    values: torch.Tensor,
    idx: torch.Tensor,
    donors: torch.Tensor | None,
    columns: list[torch.Tensor],
) -> torch.Tensor:
    out = values[idx]
    if donors is None:
        return out
    for f, cols in enumerate(columns):
        out[:, cols] = values[donors[:, f]][:, cols]
    return out


def transplant(
    points: torch.Tensor, idx: torch.Tensor, donors: torch.Tensor | None
) -> torch.Tensor:
    out = points[idx]
    if donors is None:
        return out
    for f, chain in enumerate(CHAINS):
        donor = points[donors[:, f]]
        moved = list(chain[1:])
        out[:, moved] = donor[:, moved] - donor[:, chain[:1]] + out[:, chain[:1]]
    return out


def rotations(axis: torch.Tensor, angle: torch.Tensor) -> torch.Tensor:
    x, y, z = axis.unbind(-1)
    zero = torch.zeros_like(x)
    k = torch.stack([zero, -z, y, z, zero, -x, -y, x, zero], dim=-1).view(*axis.shape[:-1], 3, 3)
    eye = torch.eye(3, dtype=axis.dtype, device=axis.device)
    t = angle[..., None, None]
    return eye + torch.sin(t) * k + (1.0 - torch.cos(t)) * (k @ k)


def jiggled(x: torch.Tensor, noise: float, rot: float, gen: torch.Generator) -> torch.Tensor:
    b = x.shape[0]
    axis = F.normalize(torch.randn(b, 3, device=x.device, generator=gen), dim=-1)
    angle = torch.randn(b, device=x.device, generator=gen) * rot
    out = torch.einsum("bij,bpj->bpi", rotations(axis, angle), x)
    return out + torch.randn(out.shape, device=x.device, generator=gen) * noise


def lateral_axis(points: torch.Tensor) -> torch.Tensor:
    z = F.normalize(points[:, MIDDLE_MCP] - points[:, WRIST], dim=-1)
    across = points[:, INDEX_MCP] - points[:, RING_MCP]
    return F.normalize(across - (across * z).sum(-1, keepdim=True) * z, dim=-1)


def curl_weight(target: torch.Tensor, robot: Robot, full: float) -> torch.Tensor | float:
    if full <= 0.0:
        return 1.0
    chains = torch.tensor(
        [c for _, _, c in robot.abd_links], dtype=torch.long, device=target.device
    )
    p = target[:, chains]
    length = torch.linalg.norm(p[:, :, 1:] - p[:, :, :-1], dim=-1).sum(-1)
    straight = torch.linalg.norm(p[:, :, -1] - p[:, :, 0], dim=-1) / length.clamp(min=1e-6)
    return ((1.0 - straight) / full).clamp(0.0, 1.0)


def abduction_loss(
    robot: Robot, frames: dict[str, Frame], target: torch.Tensor, curl: float
) -> torch.Tensor:
    lat = lateral_axis(target)[:, None]
    mcp = torch.stack([frames[a][1] for a, _, _ in robot.abd_links], dim=1)
    pip = torch.stack([frames[b][1] for _, b, _ in robot.abd_links], dim=1)
    robot_dir = F.normalize(pip - mcp, dim=-1)
    human = target[:, robot.abd_human[:, 1]] - target[:, robot.abd_human[:, 0]]
    diff = ((robot_dir - F.normalize(human, dim=-1)) * lat).sum(-1)
    return (curl_weight(target, robot, curl) * diff**2).mean()


def robot_losses(
    net: PromptNet,
    robot: Robot,
    codes: torch.Tensor,
    delta: torch.Tensor,
    target: torch.Tensor,
    prior: torch.Tensor,
    hp,
) -> dict[str, torch.Tensor]:
    b = len(prior)
    q_all = robot.q(net.decode(codes, robot.name))
    tips_all, frames = robot.fk.frames(q_all[: 4 * b], robot.wanted)
    emb, emb_dir, emb_p, emb_n = tips_all.chunk(4, dim=0)
    q = q_all[:b]
    smooth = ((q_all[4 * b : 5 * b] - q) ** 2).sum(-1).mean()
    with torch.no_grad():
        cloud = robot.fk(robot.fk.random_q(CLOUD))
    chamfer = chamfer_distance(emb.transpose(0, 1), cloud.transpose(0, 1)) * emb.shape[1]
    human_dir = F.normalize(delta[:, robot.human_fingers].reshape(-1, 3), dim=-1, eps=1e-5)
    robot_dir = F.normalize((emb_dir - emb).reshape(-1, 3), dim=-1, eps=1e-5)
    direction = -(human_dir * robot_dir).sum(-1).mean()
    curvature = ((emb_p + emb_n - 2.0 * emb) ** 2).mean()
    tips = target[:, robot.tip_ids]
    thumb = slice(robot.thumb, robot.thumb + 1)
    human_gap = torch.linalg.norm(tips[:, robot.others] - tips[:, thumb], dim=-1)
    robot_gap = torch.linalg.norm(emb[:, robot.others] - emb[:, thumb], dim=-1)
    mask = (human_gap < hp.pinch_on).float()
    goal = F.relu(human_gap - hp.pinch_contact) * robot.scale
    pinch = (mask * (robot_gap - goal) ** 2).sum() / mask.sum().clamp(min=1.0)
    tip_sq = ((emb - tips * robot.scale) ** 2).sum(-1)
    tip = tip_sq.mean()
    tip_fit = (tip_sq * robot.tip_weights).mean()
    posture = (((q - prior) / robot.span) ** 2 * robot.mask).sum(
        -1
    ).mean() / robot.mask.sum().clamp(min=1.0)
    collision = robot.collision.loss(sliced(frames, hp.collision_batch))
    abd = abduction_loss(robot, sliced(frames, b), target, hp.abd_curl)
    total = (
        direction
        + hp.w_chamfer * chamfer
        + hp.w_curvature * curvature
        + hp.w_pinch * pinch
        + hp.w_posture * posture
        + hp.w_collision * collision
        + hp.w_tip * robot.tip_scale * tip_fit
        + hp.w_smooth * smooth
        + hp.w_abd * robot.abd_scale * abd
    )
    return {
        "total": total,
        "tip": tip,
        "pinch": pinch,
        "posture": posture,
        "collision": collision,
        "chamfer": chamfer,
        "direction": direction,
        "curvature": curvature,
        "smooth": smooth,
        "abd": abd,
    }


def view(
    x: torch.Tensor, noise: float, finger_scale: float, rot: float, gen: torch.Generator
) -> torch.Tensor:
    b = x.shape[0]
    out = x.clone()
    for chain in CHAINS:
        base = out[:, chain[:1]]
        scale = (
            1.0 + (torch.rand(b, 1, 1, device=x.device, generator=gen) * 2.0 - 1.0) * finger_scale
        )
        out[:, list(chain[1:])] = base + (out[:, list(chain[1:])] - base) * scale
    return jiggled(out, noise, rot, gen)


def dropped(x: torch.Tensor, rate: float, gen: torch.Generator) -> torch.Tensor:
    keep = torch.rand(x.shape[0], POINTS - 1, 1, device=x.device, generator=gen) >= rate
    return (x.view(x.shape[0], POINTS - 1, 3) * keep).view_as(x)


def soft_contrast(
    a: torch.Tensor, b: torch.Tensor, pose: torch.Tensor, temperature: float, sigma: float
) -> torch.Tensor:
    logits = F.normalize(a, dim=-1) @ F.normalize(b, dim=-1).T / temperature
    d2 = torch.cdist(pose, pose) ** 2 / (POINTS - 1)
    soft = torch.softmax(-d2 / sigma**2, dim=1)
    forward = -(soft * torch.log_softmax(logits, dim=1)).sum(1).mean()
    backward = -(soft * torch.log_softmax(logits.T, dim=1)).sum(1).mean()
    return 0.5 * (forward + backward)
