from __future__ import annotations

import copy
import dataclasses
import json
import math
import time
from pathlib import Path

import numpy as np
import torch
import torch.nn.functional as F  # noqa: N812

from ..preprocess import FRAME_V2
from ..prompt import PROMPT_FORMAT, load_prompt_model
from .model import (
    CHAINS,
    HUMAN,
    INPUT_DIM,
    LN_EPS,
    POINTS,
    PromptNet,
    Robot,
    chain_matrix,
    dropped,
    flat,
    jiggled,
    mixed,
    robot_losses,
    soft_contrast,
    transplant,
    view,
)

LICENSE_NOTE = "Trained with facebookresearch/GeoRT (CC BY-NC 4.0); non-commercial use"
MODEL_NAME = "geort_prompt"
SIDE = "right"
KIND_ROBOT = "robot"
KIND_HUMAN = "human"
FREEZE = ("none", "encoder", "body")
STD_FLOOR = 1e-4
CHECK_DATA = 96
CHECK_NOISY = 32
CHECK_NOISE_M = 0.01
LOG_EVERY = 200
DELTA_MIN = 0.001
DELTA_SPAN = 0.01
BUMP = 0.002


@dataclasses.dataclass
class Hyper:
    steps: int = 8000
    batch: int = 2048
    lr: float = 1e-3
    weight_decay: float = 1e-4
    clip: float = 1.0
    seed: int = 0
    width: int = 512
    latent: int = 128
    prompt_dim: int = 64
    enc_layers: int = 2
    dec_layers: int = 2
    finger_mix: float = 0.3
    w_human: float = 1000.0
    w_chamfer: float = 80.0
    w_curvature: float = 0.1
    w_pinch: float = 4000.0
    w_posture: float = 10.0
    w_collision: float = 60000.0
    w_tip: float = 1000.0
    w_thumb_tip: float = 3.0
    w_smooth: float = 45.0
    w_abd: float = 6.0
    abd_curl: float = 0.4
    smooth_noise: float = 0.001
    smooth_rot_deg: float = 0.5
    pinch_on: float = 0.03
    pinch_contact: float = 0.015
    collision_batch: int = 1024
    freeze: str = "none"
    encoder_lr_scale: float = 1.0
    w_contrast: float = 0.0
    contrast_temperature: float = 0.1
    contrast_sigma: float = 0.01
    view_noise: float = 0.003
    view_dropout: float = 0.05
    view_finger_scale: float = 0.05
    view_rot_deg: float = 3.0


@dataclasses.dataclass(frozen=True)
class Joints:
    names: tuple[str, ...]
    lower: np.ndarray
    upper: np.ndarray


@dataclasses.dataclass
class Init:
    state: dict[str, torch.Tensor]
    norm: dict[str, torch.Tensor]
    heads: dict[str, int]
    joints: dict[str, Joints]
    scales: dict[str, float]
    shape: dict[str, int]


def read_init(path: Path) -> Init:
    with np.load(path, allow_pickle=False) as d:
        a = {k: d[k] for k in d.files}
    names = [str(n) for n in a["embodiments"]]
    enc, dec = int(a["enc_count"]), int(a["dec_count"])
    state = {
        f"encoder.{k}.{p}": a[f"enc{k}_{p[0]}"] for k in range(enc) for p in ("weight", "bias")
    }
    state |= {
        f"decoder.{k}.{p}": a[f"dec{k}_{p[0]}"] for k in range(dec) for p in ("weight", "bias")
    }
    state |= {f"norms.{k}.{p}": a[f"ln{k}_{p[0]}"] for k in range(dec) for p in ("weight", "bias")}
    state |= {
        "dec_in.weight": a["dec_in_w"],
        "dec_in.bias": a["dec_in_b"],
        "dec_prompt.weight": a["dec_prompt_w"],
        "film.weight": a["film_w"],
        "film.bias": a["film_b"],
    }
    for n in names:
        state |= {
            f"prompts.{n}": a[f"e_{n}_prompt"],
            f"heads.{n}.weight": a[f"e_{n}_head_w"],
            f"heads.{n}.bias": a[f"e_{n}_head_b"],
        }
    joints = {
        n: Joints(
            tuple(str(j) for j in a[f"e_{n}_joint_names"]), a[f"e_{n}_lower"], a[f"e_{n}_upper"]
        )
        for n in names
        if n != HUMAN
    }
    meta = json.loads(str(a["meta_json"]))
    return Init(
        {k: torch.tensor(v, dtype=torch.float32) for k, v in state.items()},
        {
            "in_mean": torch.tensor(a["in_mean"], dtype=torch.float32),
            "in_std": torch.tensor(a["in_std"], dtype=torch.float32),
            "out_mean": torch.tensor(a["e_human_out_mean"], dtype=torch.float32),
            "out_scale": torch.tensor(a["e_human_out_scale"], dtype=torch.float32),
        },
        {n: int(a[f"e_{n}_head_b"].size) for n in names},
        joints,
        dict(meta.get("scales", {})),
        {
            "width": int(a["dec_in_b"].size),
            "latent": int(a["dec_in_w"].shape[1]),
            "prompt_dim": int(a["dec_prompt_w"].shape[1]),
            "enc_layers": enc - 1,
            "dec_layers": dec,
        },
    )


def load_data(path: Path) -> dict[str, np.ndarray]:
    with np.load(path, allow_pickle=False) as d:
        return {k: d[k] for k in d.files}


def build_net(hp: Hyper, heads: dict[str, int], init: Init | None) -> PromptNet:
    if init is not None:
        for key, value in init.shape.items():
            setattr(hp, key, value)
    net = PromptNet(heads, hp.width, hp.latent, hp.prompt_dim, hp.enc_layers, hp.dec_layers)
    if init is not None:
        state = net.state_dict()
        kept = {k: v for k, v in init.state.items() if k in state and state[k].shape == v.shape}
        net.load_state_dict(kept, strict=False)
    return net


def parameter_groups(net: PromptNet, hp: Hyper, robots: list[Robot]) -> list[dict]:
    own = [p for r in robots for p in net.own_params(r.name)]
    if hp.freeze == "body":
        return [{"params": own, "lr": hp.lr}]
    shared = (net.dec_in, net.dec_prompt, net.decoder, net.norms, net.film)
    body = [p for m in shared for p in m.parameters()]
    groups = [{"params": body + net.own_params(HUMAN) + own, "lr": hp.lr}]
    if hp.freeze != "encoder":
        encoder = list(net.encoder.parameters())
        groups.append({"params": encoder, "lr": hp.lr * hp.encoder_lr_scale})
    return groups


def train(
    data: dict[str, np.ndarray],
    hp: Hyper,
    device: str,
    init: Init | None = None,
    human_only: bool = False,
) -> tuple[PromptNet, dict[str, torch.Tensor], list[Robot], list[dict]]:
    torch.manual_seed(hp.seed)
    np.random.seed(hp.seed)
    spec = json.loads(str(data["spec_json"]))
    robots = [] if human_only else [Robot(e, data, device) for e in spec["hands"]]
    for robot in robots:
        thumb = torch.ones(len(robot.tip_weights), device=device)
        thumb[robot.thumb] = hp.w_thumb_tip
        robot.tip_weights = thumb
    if hp.freeze != "none" and init is None:
        raise SystemExit("--freeze 는 --init 과 함께 써야 합니다")
    clean = torch.tensor(data["clean"], dtype=torch.float32, device=device)
    noisy = torch.tensor(data["noisy"], dtype=torch.float32, device=device)
    heads = {HUMAN: INPUT_DIM, **({} if init is None else init.heads)}
    heads |= {r.name: len(r.kin["joint_order"]) for r in robots}
    net = build_net(hp, heads, init).to(device)
    if init is None:
        norm = {
            "in_mean": flat(noisy).mean(0),
            "in_std": flat(noisy).std(0).clamp(min=STD_FLOOR),
            "out_mean": flat(clean).mean(0),
            "out_scale": flat(clean).std(0).clamp(min=STD_FLOOR),
        }
    else:
        norm = {k: v.to(device) for k, v in init.norm.items()}
    groups = parameter_groups(net, hp, robots)
    for p in net.parameters():
        p.requires_grad_(False)
    for group in groups:
        for p in group["params"]:
            p.requires_grad_(True)
    params = [p for g in groups for p in g["params"]]
    opt = torch.optim.AdamW(groups, lr=hp.lr, weight_decay=hp.weight_decay)
    sched = torch.optim.lr_scheduler.OneCycleLR(
        opt,
        max_lr=[g["lr"] for g in groups],
        total_steps=hp.steps,
        pct_start=0.05,
        final_div_factor=20.0,
    )
    use_human = hp.freeze != "body" and hp.w_human > 0.0
    chains = chain_matrix(device)
    gen = torch.Generator(device=device)
    gen.manual_seed(hp.seed)
    n = clean.shape[0]
    history = []
    for step in range(hp.steps):
        idx = torch.randint(0, n, (hp.batch,), device=device, generator=gen)
        donors = None
        if hp.finger_mix > 0.0:
            shape = (hp.batch, len(CHAINS))
            pick = torch.randint(0, n, shape, device=device, generator=gen)
            use = torch.rand(*shape, device=device, generator=gen) < hp.finger_mix
            donors = torch.where(use, pick, idx[:, None])
        x = transplant(noisy, idx, donors)
        target = transplant(clean, idx, donors)

        def code(points: torch.Tensor) -> torch.Tensor:
            return net.encode((flat(points) - norm["in_mean"]) / norm["in_std"])

        z = code(x)
        loss = torch.zeros((), device=device)
        row: dict[str, float] = {"step": step}
        if robots:
            shape = (hp.batch, len(CHAINS))
            direction = F.normalize(torch.randn(*shape, 3, device=device, generator=gen), dim=-1)
            span = torch.rand(*shape, 1, device=device, generator=gen) * DELTA_SPAN
            delta = direction * (DELTA_MIN + span)
            bump = F.normalize(torch.randn(*shape, 3, device=device, generator=gen), dim=-1) * BUMP
            with torch.no_grad():
                x_jig = jiggled(x, hp.smooth_noise, math.radians(hp.smooth_rot_deg), gen)
            codes = torch.cat(
                [
                    z,
                    code(x + torch.einsum("bfi,fp->bpi", delta, chains)),
                    code(x + torch.einsum("bfi,fp->bpi", bump, chains)),
                    code(x - torch.einsum("bfi,fp->bpi", bump, chains)),
                    code(x_jig),
                ]
            )
            for robot in robots:
                prior = mixed(robot.prior, idx, donors, robot.finger_joints)
                parts = robot_losses(net, robot, codes, delta, target, prior, hp)
                loss = loss + parts["total"]
                if step % LOG_EVERY == 0 or step == hp.steps - 1:
                    row |= {f"{robot.name}.{k}": float(v) for k, v in parts.items()}
        if hp.w_contrast > 0.0:
            rot = math.radians(hp.view_rot_deg)
            with torch.no_grad():
                views = [view(x, hp.view_noise, hp.view_finger_scale, rot, gen) for _ in range(2)]
            a, b = (
                net.encode(
                    dropped((flat(v) - norm["in_mean"]) / norm["in_std"], hp.view_dropout, gen)
                )
                for v in views
            )
            contrast = soft_contrast(
                a, b, flat(target), hp.contrast_temperature, hp.contrast_sigma
            )
            loss = loss + hp.w_contrast * contrast
            row["contrast"] = float(contrast)
        if use_human:
            pred = net.decode(z, HUMAN) * norm["out_scale"] + norm["out_mean"]
            human = ((pred - flat(target)).view(-1, POINTS - 1, 3) ** 2).sum(-1).mean()
            loss = loss + hp.w_human * human
            row["human"] = float(human)
        opt.zero_grad()
        loss.backward()
        torch.nn.utils.clip_grad_norm_(params, hp.clip)
        opt.step()
        sched.step()
        if step % LOG_EVERY == 0 or step == hp.steps - 1:
            row["loss"] = float(loss)
            history.append(row)
            print(json.dumps(row), flush=True)
    net.eval()
    return net, norm, robots, history


def forward_all(
    net: PromptNet,
    norm: dict[str, torch.Tensor],
    points: torch.Tensor,
    joints: dict[str, Joints],
) -> dict[str, np.ndarray]:
    out = {}
    with torch.no_grad():
        z = net.encode((flat(points) - norm["in_mean"]) / norm["in_std"])
        for name in [HUMAN, *joints]:
            raw = net.decode(z, name)
            if name == HUMAN:
                y = raw * norm["out_scale"] + norm["out_mean"]
            else:
                lower = torch.as_tensor(joints[name].lower).to(raw)
                upper = torch.as_tensor(joints[name].upper).to(raw)
                y = (torch.tanh(raw) * 0.5 + 0.5) * (upper - lower) + lower
            out[name] = y.cpu().numpy().astype(np.float64)
    return out


def body_state(net: PromptNet) -> dict[str, np.ndarray]:
    def arr(t: torch.Tensor) -> np.ndarray:
        return t.detach().cpu().numpy()

    out: dict[str, np.ndarray] = {"enc_count": np.array(len(net.encoder))}
    for k, layer in enumerate(net.encoder):
        out[f"enc{k}_w"], out[f"enc{k}_b"] = arr(layer.weight), arr(layer.bias)
    for k, layer in enumerate(net.decoder):
        out[f"dec{k}_w"], out[f"dec{k}_b"] = arr(layer.weight), arr(layer.bias)
        out[f"ln{k}_w"], out[f"ln{k}_b"] = arr(net.norms[k].weight), arr(net.norms[k].bias)
    out |= {
        "dec_count": np.array(len(net.decoder)),
        "dec_in_w": arr(net.dec_in.weight),
        "dec_in_b": arr(net.dec_in.bias),
        "dec_prompt_w": arr(net.dec_prompt.weight),
        "film_w": arr(net.film.weight),
        "film_b": arr(net.film.bias),
        "ln_eps": np.array(LN_EPS),
    }
    return out


def export(
    net: PromptNet,
    norm: dict[str, torch.Tensor],
    joints: dict[str, Joints],
    data: dict[str, np.ndarray],
    out: Path,
    meta: dict,
    seed: int,
    device: str,
) -> None:
    rng = np.random.default_rng(seed)
    noisy = data["noisy"]
    picked = noisy[rng.choice(len(noisy), size=CHECK_DATA + CHECK_NOISY, replace=False)]
    picked = picked.astype(np.float64)
    picked[CHECK_DATA:, 1:] += rng.normal(0.0, CHECK_NOISE_M, size=picked[CHECK_DATA:, 1:].shape)
    torch.backends.cuda.matmul.allow_tf32 = False
    points32 = torch.from_numpy(picked.astype(np.float32)).to(device)
    out32 = forward_all(net, norm, points32, joints)
    net64 = copy.deepcopy(net).cpu().double()
    norm64 = {k: v.detach().cpu().double() for k, v in norm.items()}
    out64 = forward_all(net64, norm64, torch.from_numpy(picked), joints)
    names = [HUMAN, *joints]
    diff = max(float(np.max(np.abs(out32[n] - out64[n]))) for n in names)
    meta = {**meta, "float32_vs_float64_max": diff}
    arrays: dict[str, np.ndarray] = {
        "format": np.array(PROMPT_FORMAT),
        "input_frame": np.array(FRAME_V2),
        "frame_template": np.asarray(data["template"], dtype=np.float64),
        "frame_size": np.array(float(data["size"])),
        "name": np.array(MODEL_NAME),
        "side": np.array(SIDE),
        "in_mean": norm["in_mean"].detach().cpu().numpy(),
        "in_std": norm["in_std"].detach().cpu().numpy(),
        "embodiments": np.array(names),
        "check_points": picked,
        "meta_json": np.array(json.dumps(meta, ensure_ascii=False)),
        **body_state(net),
    }
    for name in names:
        p = f"e_{name}_"
        arrays[f"{p}prompt"] = net.prompts[name].detach().cpu().numpy()
        arrays[f"{p}head_w"] = net.heads[name].weight.detach().cpu().numpy()
        arrays[f"{p}head_b"] = net.heads[name].bias.detach().cpu().numpy()
        arrays[f"check_{name}"] = out64[name]
        arrays[f"check32_{name}"] = out32[name]
        if name == HUMAN:
            arrays[f"{p}kind"] = np.array(KIND_HUMAN)
            arrays[f"{p}out_mean"] = norm["out_mean"].detach().cpu().numpy()
            arrays[f"{p}out_scale"] = norm["out_scale"].detach().cpu().numpy()
        else:
            arrays[f"{p}kind"] = np.array(KIND_ROBOT)
            arrays[f"{p}joint_names"] = np.array(joints[name].names)
            arrays[f"{p}lower"] = np.asarray(joints[name].lower, dtype=np.float64)
            arrays[f"{p}upper"] = np.asarray(joints[name].upper, dtype=np.float64)
    np.savez(out, **arrays)


def verify(path: Path) -> float:
    model = load_prompt_model(path)
    worst = 0.0
    for emb in model.embodiments.values():
        worst = max(worst, float(np.max(np.abs(emb.forward(emb.check_points) - emb.check_qpos))))
    return worst


def run(
    data_path: Path,
    out: Path,
    hp: Hyper,
    device: str,
    init_path: Path | None,
    human_only: bool,
) -> dict:
    data = load_data(data_path)
    init = None if init_path is None else read_init(init_path)
    started = time.time()
    net, norm, robots, history = train(data, hp, device, init, human_only)
    seconds = round(time.time() - started, 1)
    joints = {} if init is None else dict(init.joints)
    joints |= {
        r.name: Joints(
            tuple(r.kin["joint_order"]), np.array(r.kin["lower"]), np.array(r.kin["upper"])
        )
        for r in robots
    }
    scales = {} if init is None else dict(init.scales)
    scales |= {r.name: r.scale for r in robots}
    meta = {
        **dataclasses.asdict(hp),
        "stage": "human" if human_only else ("tune" if hp.freeze == "body" else "joint"),
        "init": None if init_path is None else init_path.name,
        "trained": [r.name for r in robots],
        "scales": scales,
        "human_frames": int(len(data["clean"])),
        "torch": torch.__version__,
        "device": torch.cuda.get_device_name() if device.startswith("cuda") else device,
        "train_seconds": seconds,
        "license": LICENSE_NOTE,
        "final_losses": history[-1],
    }
    export(net, norm, joints, data, out, meta, hp.seed, device)
    out.with_suffix(".log.json").write_text(json.dumps(history, indent=1), encoding="utf-8")
    return {**meta, "numpy_vs_torch_max": verify(out)}
