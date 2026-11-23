from __future__ import annotations

import dataclasses
import json
from pathlib import Path

import numpy as np

from whatslab.core.constants import HAND21_COUNT

from .preprocess import FRAME_V2, PALM_MCPS, InputFrame

PROMPT_FORMAT = "geort-prompt-v1"
KIND_ROBOT = "robot"
KIND_HUMAN = "human"
KINDS = (KIND_ROBOT, KIND_HUMAN)
POINT_DIM = 3
INPUT_DIM = (HAND21_COUNT - 1) * POINT_DIM


def silu(x: np.ndarray) -> np.ndarray:
    return x / (1.0 + np.exp(-x))


def layer_norm(x: np.ndarray, weight: np.ndarray, bias: np.ndarray, eps: float) -> np.ndarray:
    mean = x.mean(axis=-1, keepdims=True)
    centered = x - mean
    var = (centered * centered).mean(axis=-1, keepdims=True)
    return centered / np.sqrt(var + eps) * weight + bias


@dataclasses.dataclass(frozen=True)
class Linear:
    weight: np.ndarray
    bias: np.ndarray

    def __call__(self, x: np.ndarray) -> np.ndarray:
        return x @ self.weight.T + self.bias


def normalized(points: np.ndarray, mean: np.ndarray, std: np.ndarray) -> np.ndarray:
    return (points[..., 1:, :].reshape(points.shape[:-2] + (INPUT_DIM,)) - mean) / std


@dataclasses.dataclass(frozen=True)
class Backbone:
    in_mean: np.ndarray
    in_std: np.ndarray
    encoder: tuple[Linear, ...]
    dec_in: Linear
    dec_prompt: np.ndarray
    decoder: tuple[Linear, ...]
    norms: tuple[tuple[np.ndarray, np.ndarray], ...]
    film: Linear
    ln_eps: float

    @property
    def width(self) -> int:
        return self.dec_in.bias.size

    def prompt_shape(self) -> tuple[int, ...]:
        return (self.dec_prompt.shape[1],)

    def encode(self, points: np.ndarray) -> np.ndarray:
        x = normalized(points, self.in_mean, self.in_std)
        for k, layer in enumerate(self.encoder):
            x = layer(x)
            if k < len(self.encoder) - 1:
                x = silu(x)
        return x

    def condition(self, prompt: np.ndarray) -> tuple[np.ndarray, ...]:
        shift = self.dec_prompt @ prompt + self.dec_in.bias
        film = self.film(prompt).reshape(len(self.decoder), 2, self.width)
        return shift, 1.0 + film[:, 0], film[:, 1]

    def raw(self, points: np.ndarray, cond: tuple[np.ndarray, ...], head: Linear) -> np.ndarray:
        shift, gamma, beta = cond
        h = silu(self.encode(points) @ self.dec_in.weight.T + shift)
        for k, layer in enumerate(self.decoder):
            weight, bias = self.norms[k]
            h = h + silu(layer_norm(layer(h), weight, bias, self.ln_eps) * gamma[k] + beta[k])
        return head(h)


@dataclasses.dataclass(frozen=True)
class Embodiment:
    name: str
    kind: str
    side: str
    joint_names: tuple[str, ...]
    lower: np.ndarray
    upper: np.ndarray
    out_mean: np.ndarray
    out_scale: np.ndarray
    prompt: np.ndarray
    head: Linear
    backbone: Backbone
    frame: InputFrame
    check_points: np.ndarray
    check_qpos: np.ndarray
    meta: dict[str, object]
    cond: tuple[np.ndarray, ...] = dataclasses.field(init=False)

    def __post_init__(self) -> None:
        object.__setattr__(self, "cond", self.backbone.condition(self.prompt))

    def raw(self, points: np.ndarray) -> np.ndarray:
        return self.backbone.raw(np.asarray(points, dtype=np.float64), self.cond, self.head)

    def forward(self, points: np.ndarray) -> np.ndarray:
        y = self.raw(points)
        if self.kind == KIND_HUMAN:
            return y * self.out_scale + self.out_mean
        return (np.tanh(y) * 0.5 + 0.5) * (self.upper - self.lower) + self.lower


@dataclasses.dataclass(frozen=True)
class PromptModel:
    name: str
    side: str
    embodiments: dict[str, Embodiment]
    meta: dict[str, object]


def _text(data: np.lib.npyio.NpzFile, key: str) -> str:
    return str(data[key].item())


def _array(data: np.lib.npyio.NpzFile, key: str) -> np.ndarray:
    return np.asarray(data[key], dtype=np.float64)


def _linear(data: np.lib.npyio.NpzFile, key: str) -> Linear:
    weight = _array(data, f"{key}_w")
    bias = _array(data, f"{key}_b")
    if weight.ndim != 2 or bias.shape != (weight.shape[0],):
        raise ValueError(f"{key} 층 모양이 맞지 않습니다")
    return Linear(weight, bias)


def _frame(data: np.lib.npyio.NpzFile) -> InputFrame:
    name = _text(data, "input_frame")
    template = _array(data, "frame_template")
    size = float(data["frame_size"])
    if name != FRAME_V2:
        raise ValueError(f"input_frame {name}을(를) 지원하지 않습니다 ({FRAME_V2})")
    if (
        template.shape != (len(PALM_MCPS), POINT_DIM)
        or not np.all(np.isfinite(template))
        or not size > 0.0
    ):
        raise ValueError("frame_template은 (4, 3), frame_size는 양수여야 합니다")
    template.setflags(write=False)
    return InputFrame(name, template, size)


def _input_norm(data: np.lib.npyio.NpzFile) -> tuple[np.ndarray, np.ndarray]:
    in_mean, in_std = _array(data, "in_mean"), _array(data, "in_std")
    if in_mean.shape != (INPUT_DIM,) or in_std.shape != (INPUT_DIM,) or np.any(in_std <= 0.0):
        raise ValueError("입력 정규화 값이 잘못됐습니다")
    return in_mean, in_std


def _backbone(data: np.lib.npyio.NpzFile) -> Backbone:
    encoder = tuple(_linear(data, f"enc{k}") for k in range(int(data["enc_count"])))
    decoder = tuple(_linear(data, f"dec{k}") for k in range(int(data["dec_count"])))
    norms = tuple(
        (_array(data, f"ln{k}_w"), _array(data, f"ln{k}_b")) for k in range(len(decoder))
    )
    dec_in = _linear(data, "dec_in")
    width = dec_in.bias.size
    in_mean, in_std = _input_norm(data)
    if not encoder or not decoder or encoder[0].weight.shape[1] != INPUT_DIM:
        raise ValueError(f"인코더 입력은 {INPUT_DIM}개여야 합니다")
    if dec_in.weight.shape[1] != encoder[-1].bias.size:
        raise ValueError("인코더 출력과 디코더 입력 크기가 다릅니다")
    if any(layer.weight.shape != (width, width) for layer in decoder):
        raise ValueError("디코더 층 크기가 다릅니다")
    film = _linear(data, "film")
    if film.bias.size != 2 * width * len(decoder):
        raise ValueError("FiLM 출력 크기가 디코더와 맞지 않습니다")
    return Backbone(
        in_mean,
        in_std,
        encoder,
        dec_in,
        _array(data, "dec_prompt_w"),
        decoder,
        norms,
        film,
        float(data["ln_eps"]),
    )


def _head(data: np.lib.npyio.NpzFile, p: str, backbone: Backbone) -> Linear:
    head = _linear(data, f"{p}head")
    if head.weight.shape[1] != backbone.width:
        raise ValueError(f"{p}head 입력 크기가 백본 폭 {backbone.width}와 맞지 않습니다")
    return head


def _embodiment(
    data: np.lib.npyio.NpzFile,
    name: str,
    side: str,
    backbone: Backbone,
    frame: InputFrame,
    check_points: np.ndarray,
    meta: dict[str, object],
) -> Embodiment:
    p = f"e_{name}_"
    kind = _text(data, f"{p}kind")
    if kind not in KINDS:
        raise ValueError(f"{name}의 종류 {kind}을(를) 알 수 없습니다")
    head = _head(data, p, backbone)
    prompt = _array(data, f"{p}prompt")
    if prompt.shape != backbone.prompt_shape():
        raise ValueError(f"{name}의 프롬프트 크기가 백본과 맞지 않습니다")
    dof = head.bias.size
    if kind == KIND_ROBOT:
        names = tuple(str(n) for n in data[f"{p}joint_names"])
        lower, upper = _array(data, f"{p}lower"), _array(data, f"{p}upper")
        if (
            len(names) != dof
            or lower.shape != (dof,)
            or upper.shape != (dof,)
            or np.any(upper <= lower)
        ):
            raise ValueError(
                f"{name}의 관절 한계가 관절 수와 맞지 않거나 상한이 하한보다 작습니다"
            )
        out_mean = out_scale = np.zeros(0)
    else:
        names = tuple(f"p{i}.{a}" for i in range(1, HAND21_COUNT) for a in "xyz")
        lower = upper = np.zeros(0)
        out_mean, out_scale = _array(data, f"{p}out_mean"), _array(data, f"{p}out_scale")
        if dof != INPUT_DIM or out_mean.shape != (dof,) or out_scale.shape != (dof,):
            raise ValueError(f"{name}의 사람 손 출력은 {INPUT_DIM}개여야 합니다")
    return Embodiment(
        name,
        kind,
        side,
        names,
        lower,
        upper,
        out_mean,
        out_scale,
        prompt,
        head,
        backbone,
        frame,
        check_points,
        _array(data, f"check_{name}"),
        meta,
    )


def load_prompt_model(path: Path) -> PromptModel:
    path = path.expanduser()
    with np.load(path, allow_pickle=False) as data:
        fmt = _text(data, "format")
        if fmt != PROMPT_FORMAT:
            raise ValueError(f"format {fmt}을(를) 지원하지 않습니다 ({PROMPT_FORMAT})")
        frame = _frame(data)
        names = [str(n) for n in data["embodiments"]]
        backbone = _backbone(data)
        side = _text(data, "side")
        meta = json.loads(_text(data, "meta_json"))
        check_points = _array(data, "check_points")
        embodiments = {
            n: _embodiment(data, n, side, backbone, frame, check_points, meta) for n in names
        }
        model = PromptModel(_text(data, "name"), side, embodiments, meta)
    return model

