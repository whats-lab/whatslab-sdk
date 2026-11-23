"""GeoRT 소프트 프롬프트 모델(.npz) 읽기·검사와 numpy 순전파."""
from __future__ import annotations

import dataclasses
import json
import zipfile
from pathlib import Path

import numpy as np

from whatslab.core.constants import FINGER_COUNT, HAND21_COUNT

from .errors import GeortConfigError
from .keypoints import FRAME_V2, PALM_MCPS, InputFrame

PROMPT_FORMAT = "geort-prompt-v1"
TOKEN_FORMAT = "geort-prompt-v2"
PROMPT_FORMATS = (PROMPT_FORMAT, TOKEN_FORMAT)
KIND_ROBOT = "robot"
KIND_HUMAN = "human"
KINDS = (KIND_ROBOT, KIND_HUMAN)
POINT_DIM = 3
INPUT_DIM = (HAND21_COUNT - 1) * POINT_DIM
ALL_POINTS = np.arange(HAND21_COUNT, dtype=np.intp)
CHAIN_POINTS = 4
CHAIN_DIM = CHAIN_POINTS * POINT_DIM
ROOT_DIM = FINGER_COUNT * POINT_DIM
INPUT_TOKENS = FINGER_COUNT + 1
_REMEDY = ("robotPlatform 의 tools/bake_geort_prompt.py 로 모델을 다시 만들거나 "
           "설정 파일의 model 경로를 확인하세요.")


def silu(x: np.ndarray) -> np.ndarray:
    """SiLU 활성화 x·σ(x)."""
    return x / (1.0 + np.exp(-x))


def layer_norm(x: np.ndarray, weight: np.ndarray, bias: np.ndarray, eps: float) -> np.ndarray:
    """마지막 축 LayerNorm (affine 포함)."""
    mean = x.mean(axis=-1, keepdims=True)
    centered = x - mean
    var = (centered * centered).mean(axis=-1, keepdims=True)
    return centered / np.sqrt(var + eps) * weight + bias


def softmax(x: np.ndarray) -> np.ndarray:
    """마지막 축 softmax (수치 안정형)."""
    e = np.exp(x - x.max(axis=-1, keepdims=True))
    return e / e.sum(axis=-1, keepdims=True)


@dataclasses.dataclass(frozen=True)
class Linear:
    """완전연결층 ``x @ Wᵀ + b``."""
    weight: np.ndarray
    bias: np.ndarray

    def __call__(self, x: np.ndarray) -> np.ndarray:
        return x @ self.weight.T + self.bias


@dataclasses.dataclass(frozen=True)
class TokenHead:
    """출력마다 지정한 토큰 하나를 읽는 출력층."""
    weight: np.ndarray
    bias: np.ndarray
    token: np.ndarray

    def __call__(self, h: np.ndarray) -> np.ndarray:
        return (h[..., self.token, :] * self.weight).sum(axis=-1) + self.bias


def normalized(points: np.ndarray, mean: np.ndarray, std: np.ndarray) -> np.ndarray:
    """손목을 뺀 20점을 펴서 입력 평균·표준편차로 정규화한다."""
    return (points[..., 1:, :].reshape(points.shape[:-2] + (INPUT_DIM,)) - mean) / std


@dataclasses.dataclass(frozen=True)
class Backbone:
    """MLP 인코더 + FiLM 조건 디코더 백본(geort-prompt-v1)."""
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
        """디코더 은닉 폭."""
        return self.dec_in.bias.size

    def prompt_shape(self) -> tuple[int, ...]:
        """이 백본이 받는 프롬프트 모양."""
        return (self.dec_prompt.shape[1],)

    def encode(self, points: np.ndarray) -> np.ndarray:
        """키포인트를 잠재 벡터로 인코딩한다."""
        x = normalized(points, self.in_mean, self.in_std)
        for k, layer in enumerate(self.encoder):
            x = layer(x)
            if k < len(self.encoder) - 1:
                x = silu(x)
        return x

    def condition(self, prompt: np.ndarray) -> tuple[np.ndarray, ...]:
        """프롬프트로 디코더 조건(shift, gamma, beta)을 만든다."""
        shift = self.dec_prompt @ prompt + self.dec_in.bias
        film = self.film(prompt).reshape(len(self.decoder), 2, self.width)
        return shift, 1.0 + film[:, 0], film[:, 1]

    def raw(
        self, points: np.ndarray, cond: tuple[np.ndarray, ...], head: Linear | TokenHead
    ) -> np.ndarray:
        """조건을 건 디코더 출력(출력층 적용, 범위 변환 전)."""
        shift, gamma, beta = cond
        h = silu(self.encode(points) @ self.dec_in.weight.T + shift)
        for k, layer in enumerate(self.decoder):
            weight, bias = self.norms[k]
            h = h + silu(layer_norm(layer(h), weight, bias, self.ln_eps) * gamma[k] + beta[k])
        return head(h)


def normalize(x: np.ndarray, eps: float) -> np.ndarray:
    """마지막 축 정규화(affine 없음 — 가중치는 다음 층에 접어 넣었다)."""
    centered = x - x.mean(axis=-1, keepdims=True)
    return centered / np.sqrt((centered * centered).mean(axis=-1, keepdims=True) + eps)


@dataclasses.dataclass(frozen=True)
class TokenBlock:
    """LayerNorm 을 접어 넣은 트랜스포머 블록 가중치."""
    qkv_w: np.ndarray
    qkv_b: np.ndarray
    out_w: np.ndarray
    out_b: np.ndarray
    fc1_w: np.ndarray
    fc1_b: np.ndarray
    fc2_w: np.ndarray
    fc2_b: np.ndarray


Norm = tuple[np.ndarray, np.ndarray]


def token_block(
    ln1: Norm, qkv: Linear, proj: Linear, ln2: Norm, fc1: Linear, fc2: Linear, heads: int
) -> TokenBlock:
    """트랜스포머 블록 가중치를 LayerNorm·스케일을 접어 추론용으로 만든다."""
    width = proj.bias.size
    size = width // heads
    scale = np.repeat([1.0 / np.sqrt(size), 1.0, 1.0], width)
    weight = qkv.weight * ln1[0] * scale[:, None]
    bias = (qkv.bias + qkv.weight @ ln1[1]) * scale
    qkv_w = weight.reshape(3 * heads, size, width).transpose(0, 2, 1)
    out_w = proj.weight.T.reshape(heads, size, width)
    fc1_w = np.ascontiguousarray((fc1.weight * ln2[0]).T)
    return TokenBlock(
        np.ascontiguousarray(qkv_w),
        bias.reshape(3 * heads, 1, size),
        np.ascontiguousarray(out_w),
        proj.bias,
        fc1_w,
        fc1.bias + fc1.weight @ ln2[1],
        np.ascontiguousarray(fc2.weight.T),
        fc2.bias,
    )


@dataclasses.dataclass(frozen=True)
class TokenBackbone:
    """손가락 토큰 트랜스포머 백본(geort-prompt-v2)."""
    in_mean: np.ndarray
    in_std: np.ndarray
    finger_w: np.ndarray
    finger_b: np.ndarray
    palm: Linear
    blocks: tuple[TokenBlock, ...]
    final: Norm
    heads: int
    ln_eps: float
    prompt_tokens: int

    @property
    def width(self) -> int:
        """토큰 폭."""
        return self.finger_b.shape[1]

    def prompt_shape(self) -> tuple[int, ...]:
        """이 백본이 받는 프롬프트 모양 (토큰 수, 폭)."""
        return (self.prompt_tokens, self.width)

    def condition(self, prompt: np.ndarray) -> tuple[np.ndarray, ...]:
        """프롬프트 토큰을 그대로 조건으로 쓴다."""
        return (prompt,)

    def attend(self, block: TokenBlock, h: np.ndarray) -> np.ndarray:
        """블록 하나(셀프 어텐션 + MLP, pre-norm 잔차)를 적용한다."""
        qkv = normalize(h, self.ln_eps)[..., None, :, :] @ block.qkv_w + block.qkv_b
        n = self.heads
        q, k, v = qkv[..., :n, :, :], qkv[..., n : 2 * n, :, :], qkv[..., 2 * n :, :, :]
        weights = softmax(q @ np.swapaxes(k, -1, -2))
        h = h + ((weights @ v) @ block.out_w).sum(axis=-3) + block.out_b
        return (
            h
            + silu(normalize(h, self.ln_eps) @ block.fc1_w + block.fc1_b) @ block.fc2_w
            + block.fc2_b
        )

    def raw(
        self, points: np.ndarray, cond: tuple[np.ndarray, ...], head: Linear | TokenHead
    ) -> np.ndarray:
        """손바닥·손가락·프롬프트 토큰을 블록에 통과시킨 출력(범위 변환 전)."""
        x = normalized(points, self.in_mean, self.in_std)
        lead = x.shape[:-1]
        fingers = (x.reshape(lead + (FINGER_COUNT, 1, CHAIN_DIM)) @ self.finger_w)[
            ..., 0, :
        ] + self.finger_b
        roots = x.reshape(lead + (FINGER_COUNT, CHAIN_POINTS, POINT_DIM))[..., 0, :].reshape(
            lead + (ROOT_DIM,)
        )
        prompt = np.broadcast_to(cond[0], lead + cond[0].shape)
        h = np.concatenate([self.palm(roots)[..., None, :], fingers, prompt], axis=-2)
        for block in self.blocks:
            h = self.attend(block, h)
        return head(normalize(h, self.ln_eps))


@dataclasses.dataclass(frozen=True)
class Embodiment:
    """프롬프트 모델 안의 손 하나(로봇 관절 또는 사람 손 키포인트 출력)."""
    name: str
    kind: str
    side: str
    joint_names: tuple[str, ...]
    lower: np.ndarray
    upper: np.ndarray
    out_mean: np.ndarray
    out_scale: np.ndarray
    prompt: np.ndarray
    head: Linear | TokenHead
    backbone: Backbone | TokenBackbone
    frame: InputFrame
    check_points: np.ndarray
    check_qpos: np.ndarray
    meta: dict[str, object]
    cond: tuple[np.ndarray, ...] = dataclasses.field(init=False)

    def __post_init__(self) -> None:
        object.__setattr__(self, "cond", self.backbone.condition(self.prompt))

    @property
    def human_ids(self) -> np.ndarray:
        """입력으로 쓰는 사람 손 21점 인덱스."""
        return ALL_POINTS

    def raw(self, points: np.ndarray) -> np.ndarray:
        """백본·출력층 원출력."""
        return self.backbone.raw(np.asarray(points, dtype=np.float64), self.cond, self.head)

    def forward(self, points: np.ndarray) -> np.ndarray:
        """키포인트 → 출력. 로봇은 관절 범위로, 사람 손은 역정규화한다."""
        y = self.raw(points)
        if self.kind == KIND_HUMAN:
            return y * self.out_scale + self.out_mean
        return (np.tanh(y) * 0.5 + 0.5) * (self.upper - self.lower) + self.lower


@dataclasses.dataclass(frozen=True)
class PromptModel:
    """공유 백본과 손별 프롬프트를 담은 GeoRT 프롬프트 모델."""
    name: str
    side: str
    embodiments: dict[str, Embodiment]
    meta: dict[str, object]

    def embodiment(self, name: str) -> Embodiment:
        """이름으로 손 하나를 꺼낸다."""
        return self.embodiments[name]


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


def _norm(data: np.lib.npyio.NpzFile, key: str, width: int) -> tuple[np.ndarray, np.ndarray]:
    weight, bias = _array(data, f"{key}_w"), _array(data, f"{key}_b")
    if weight.shape != (width,) or bias.shape != (width,):
        raise ValueError(f"{key} 정규화 크기가 {width}가 아닙니다")
    return weight, bias


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


def _token_block(data: np.lib.npyio.NpzFile, k: int, width: int, heads: int) -> TokenBlock:
    p = f"blk{k}_"
    qkv, proj, fc1, fc2 = (_linear(data, f"{p}{n}") for n in ("qkv", "proj", "fc1", "fc2"))
    hidden = fc1.bias.size
    if (
        qkv.weight.shape != (3 * width, width)
        or proj.weight.shape != (width, width)
        or fc1.weight.shape != (hidden, width)
        or fc2.weight.shape != (width, hidden)
    ):
        raise ValueError(f"블록 {k}의 층 크기가 폭 {width}와 맞지 않습니다")
    ln1, ln2 = _norm(data, f"{p}ln1", width), _norm(data, f"{p}ln2", width)
    return token_block(ln1, qkv, proj, ln2, fc1, fc2, heads)


def _token_backbone(data: np.lib.npyio.NpzFile, prompt_tokens: int) -> TokenBackbone:
    finger_w, finger_b = _array(data, "finger_w"), _array(data, "finger_b")
    if finger_w.ndim != 3 or finger_w.shape[0] != FINGER_COUNT or finger_w.shape[2] != CHAIN_DIM:
        raise ValueError(f"손가락 토큰 입력은 ({FINGER_COUNT}, 폭, {CHAIN_DIM})이어야 합니다")
    width = finger_w.shape[1]
    heads = int(data["head_count"])
    if finger_b.shape != (FINGER_COUNT, width) or heads < 1 or width % heads:
        raise ValueError("손가락 토큰 치우침이나 머리 수가 폭과 맞지 않습니다")
    palm = _linear(data, "palm")
    if palm.weight.shape != (width, ROOT_DIM):
        raise ValueError(f"손바닥 토큰 입력은 {ROOT_DIM}개여야 합니다")
    blocks = tuple(_token_block(data, k, width, heads) for k in range(int(data["block_count"])))
    if not blocks:
        raise ValueError("트랜스포머 블록이 없습니다")
    in_mean, in_std = _input_norm(data)
    return TokenBackbone(
        in_mean,
        in_std,
        np.ascontiguousarray(finger_w.transpose(0, 2, 1)),
        finger_b,
        palm,
        blocks,
        _norm(data, "lnf", width),
        heads,
        float(data["ln_eps"]),
        prompt_tokens,
    )


def _head(
    data: np.lib.npyio.NpzFile, p: str, backbone: Backbone | TokenBackbone
) -> Linear | TokenHead:
    head = _linear(data, f"{p}head")
    if head.weight.shape[1] != backbone.width:
        raise ValueError(f"{p}head 입력 크기가 백본 폭 {backbone.width}와 맞지 않습니다")
    if isinstance(backbone, Backbone):
        return head
    token = np.asarray(data[f"{p}token"])
    if token.shape != head.bias.shape or not np.issubdtype(token.dtype, np.integer):
        raise ValueError(f"{p}token은 출력마다 정수 하나여야 합니다")
    if np.any(token < 0) or np.any(token >= INPUT_TOKENS):
        raise ValueError(f"{p}token은 0~{INPUT_TOKENS - 1}이어야 합니다")
    gain, shift = backbone.final
    return TokenHead(head.weight * gain, head.bias + head.weight @ shift, token.astype(np.intp))


def _embodiment(
    data: np.lib.npyio.NpzFile,
    name: str,
    side: str,
    backbone: Backbone | TokenBackbone,
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


def _any_backbone(
    data: np.lib.npyio.NpzFile, fmt: str, names: list[str]
) -> Backbone | TokenBackbone:
    if fmt == PROMPT_FORMAT:
        return _backbone(data)
    shapes = {np.shape(data[f"e_{n}_prompt"]) for n in names}
    if len(shapes) != 1 or len(next(iter(shapes))) != 2:
        raise ValueError("프롬프트 토큰 모양이 손마다 같은 (토큰 수, 폭)이어야 합니다")
    return _token_backbone(data, next(iter(shapes))[0])


def load_prompt_model(path: Path) -> PromptModel:
    """GeoRT 프롬프트 모델(.npz)을 읽고 검사한다. 실패하면 GeortConfigError."""
    path = path.expanduser()
    try:
        with np.load(path, allow_pickle=False) as data:
            fmt = _text(data, "format")
            if fmt not in PROMPT_FORMATS:
                raise ValueError(
                    f"format {fmt}을(를) 지원하지 않습니다 ({', '.join(PROMPT_FORMATS)})"
                )
            frame = _frame(data)
            names = [str(n) for n in data["embodiments"]]
            backbone = _any_backbone(data, fmt, names)
            side = _text(data, "side")
            meta = json.loads(_text(data, "meta_json"))
            check_points = _array(data, "check_points")
            embodiments = {
                n: _embodiment(data, n, side, backbone, frame, check_points, meta) for n in names
            }
            model = PromptModel(_text(data, "name"), side, embodiments, meta)
    except FileNotFoundError as e:
        raise GeortConfigError(
            "GEORT_MODEL_MISSING", f"GeoRT 프롬프트 모델 파일을 찾을 수 없습니다: {path}", _REMEDY
        ) from e
    except OSError as e:
        raise GeortConfigError(
            "GEORT_MODEL_MISSING",
            f"GeoRT 프롬프트 모델 파일을 읽을 수 없습니다: {path} ({e.strerror or e})",
            _REMEDY,
        ) from e
    except (KeyError, ValueError, TypeError, IndexError, zipfile.BadZipFile) as e:
        raise GeortConfigError(
            "GEORT_MODEL_INVALID",
            f"GeoRT 프롬프트 모델 파일 형식이 잘못됐습니다: {path} ({e})",
            _REMEDY,
        ) from e
    return model
