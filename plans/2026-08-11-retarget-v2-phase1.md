# 리타게팅 v2 — P0 + Phase 1 구현 계획

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 사람 keyvector 를 입력받아 로봇 관절각을 직접 내는 학습 리타게터를 만들고, 형상 추종에서 kp 기준선을 이기는지 판정한다.

**Architecture:** `ref/planing.md` II 확정 구조. 사람 URDF FK(pinocchio) → keyvector 6D/손가락 → 손가락별 MLP `F` → q_robot(tanh→joint_roms). 학습 시에만 로봇 pinocchio FK 를 `torch.autograd.Function` 으로 미분 통과시켜 keypoint 공간 손실을 역전파한다. GeoRT 와 같은 "q 직접 출력" 구조이되 **frozen neural FK 대신 정확 FK** 를 쓴다. 추론 경로에 FK 없음(forward 1회). GeoRT 코드를 벤더링하지 않는다 — 손실 공식만 가져오고 구현은 저장소 안에 새로 쓴다.

**Tech Stack:** numpy 1.26, pinocchio(pip `pin`), torch(기존 `hand` extra), pytest.

## 구현 결과 (2026-08-11) — 이 계획은 실행됐다

**Task 0~6 구현·테스트 완료.** 테스트 143 passed / 6 skipped. 측정 결과와 판정은
`plans/RETARGET_V2_PLAN.md` §4~5 가 정본이고, 이 문서는 **작업 단위와 코드 근거**로 남긴다.

계획 대비 바뀐 것 넷 (전부 의도적):

1. **`sensor_chains` 를 `keyvector.py` 로 빼서 `KPHandRetargeter` 와 같은 규약을 쓴다.**
   처음엔 `net_retargeter` 안에서 사슬을 따로 유도했는데 robotis 에서 5점, orca 에서 5점이
   나와 kp 의 4점과 달랐다 → 팜 프레임 회전이 어긋나 **두 백엔드를 공정 비교할 수 없게 된다.**
   지금은 팜 회전이 kp 와 `0.00e+00` 일치한다. (원점은 설계상 다르다: kp 는 너클 평균,
   net 은 dorsum.)
2. **`net_losses.py` 를 추가**했다(계획의 File Structure 에 없던 파일). 손실을 함수로
   분리해야 회전 불변성 같은 성질을 단위 테스트로 고정할 수 있다.
3. **`chain_weights` 테스트를 표현이 아니라 결과 위치로 검사**하도록 고쳤다. 계획의 테스트는
   `chain_weights([1,1]) == (1, 0.0)` 을 기대했는데 구현은 `(0, 1.0)` 을 낸다 — **같은 점**이다.
   표현을 고정하면 등가 구현을 거짓 실패로 만든다.
4. **`--traj flex`/`abd` 를 Task 4 보다 먼저 넣었다.** 핀치 램프만으로는 미해결 문제가
   측정에 안 나타나서, 지표보다 궤적이 먼저 필요했다.

**측정 중 잡은 함정 하나**: stale `__pycache__` 로 재현 불가한 런이 나왔다(orca kp 굽힘
형상 32.5° → 실제 43.4°). 측정 전 pycache 제거를 규칙으로 박았다.

**아직 안 한 것**: Task 7(orca 교차 확인)은 robotis 학습이 끝난 뒤 순차 실행 중.

---

## Global Constraints

- **파이썬 코드에 독스트링·주석을 쓰지 않는다** (`src`·`tools`·`examples`·`tests`). 예외는 `# noqa`/`# type:`/`# pragma`/shebang, 그리고 `argparse(description=__doc__)` 를 쓰는 `tools/` 파일의 모듈 독스트링.
- 커밋 메시지는 한국어, `type(scope): 요약`. Claude 트레일러 금지.
- `numpy>=1.24,<2` 상한 유지. pip 전용 스택 유지(casadi/IPOPT/conda-forge pinocchio 금지).
- **lazy import 금지** — 함수 안에서 import 하지 않는다. 전부 모듈 최상단.
- 레이어 규칙: `solvers/` 는 수치 해법만. 리시버·rig 를 모른다.
- 새 공개 심볼은 `solvers/hand/__init__.py` 와 `solvers/__init__.py` 의 `__all__` 양쪽에 등록하고, `README.md`·`README.ko.md`·`docs/API.md` 3종을 함께 맞춘다.
- 테스트 실행은 `WHATSLAB_MODELS_ROOT=/home/whatslab09/whatslab-models/dexhand_description /home/whatslab09/micromamba/envs/dex_mj/bin/python -m pytest -q -rs`. 동봉 패키지는 관절명이 구버전이라 env 를 반드시 건다.
- 판정 기준선은 `plans/RETARGET_V2_PLAN.md` §0 표. **구 43.3° 는 무효** (사람 URDF 관절명 불일치 상태의 측정).
- 목표 기반 지표로 백엔드를 비교하지 않는다. 한 번에 하나만 바꾸고 ablation 한다.

---

## 선행 게이트 (코드 작업 아님 — URDF 담당)

**Phase 1 은 왼손 단일 side 로 진행하므로 이 게이트가 열리지 않아도 Task 0~7 을 끝낼 수 있다.**
게이트는 **좌우 통합 모델**(§좌우 통합)의 전제다. 수용 기준은 Task 0 의 `check_mirror.py` 출력.

1. **오른손 `right_{index,middle,ring,pinky}_mcp_abd` 축.** `axis="0 0 1"` → `"0 0 -1"`. 축은 의사벡터라 미러 시 `a' = -M a`. 실측 미러오차 15°에서 index 43.9 / middle 49.6 / ring 43.0 / pinky 35.4mm, 25°에서 최대 81mm. 반전하면 5·15·25° 전부 0.00mm.
2. **`sensor_dorsum` 통일** — 6개 URDF 에서 손목 중앙으로. 현재 robotis 좌우 50mm 불일치, orca 는 카팔 원점, human 은 wrist+(0.05, ∓0.01, 0.025).
3. **로봇 손 좌우 미러** — orca 는 외전축(`*_to_Carpals`) 3/16 관절이 최악 8.53mm(그 관절 변위 22.2mm 중). robotis 는 **20/20 관절**이 어긋나고 최악 69.6mm(변위 28.8mm 중), 중립 사슬도 전 손가락 균일 9.79mm — 전면 재검토가 필요하다.
4. **`regenerate.py` 를 실행하지 않는다** — models root 직접 수정을 덮어쓴다.

---

## 좌우 통합 여부 — 결론

**통합을 목표로 설계하되, 도입은 손별 미러 게이트 통과 후.** 근거와 판단은
`plans/RETARGET_V2_PLAN.md` §2.5. 요지 세 줄:

- 좌우가 참 미러면 왼손 keyvector = 오른손 keyvector 의 z 부호 반전이고 q 는 동일하다 →
  `q = F(mirror_z(kv))` 가 근사가 아니라 **엄밀한 대칭**이다.
- 통합하면 좌우 대칭이 공짜로 보장된다. **따로 학습하면 비지도 해의 비유일성 때문에
  두 손이 다른 국소해에 앉아 조작자가 좌우에서 다른 감각을 느낀다**(GeoRT 는 Ability Hand
  LMC 36.2±11.2 를 보고했다). 부수 이득으로 데이터 2배, 체크포인트·학습시간 절반.
- 지금은 human 4/21, orca 3/16, robotis 20/20 이 미러가 아니라 도입 불가. 미러가 깨진 채
  통합하면 한쪽 오차를 반대쪽에 떠넘긴다. **손별로** 전 관절 2mm 이하가 되면 그 손만 전환한다.

Phase 1 의 산출물은 **side 별 모델**이다. 통합은 최적화이지 전제가 아니므로, 영구히 미러가
안 되는 손이 나오면 그 손만 side 별로 남긴다.

---

## Task 0: 미러 검사 도구

**Files:**
- Create: `tools/check_mirror.py`

**Interfaces:**
- Produces: `mirror_error(cfg_or_human, side_pair) -> Dict[str, float]` 와 CLI 표 출력

- [ ] **Step 1: 도구를 쓴다**

`tools/check_mirror.py`:

```python
#!/usr/bin/env python3
"""좌우 URDF 가 참 미러인지 관절별로 잰다 (좌우 통합 모델의 수용 기준)."""
import argparse

import numpy as np
import pinocchio as pin

from whatslab.solvers.hand import KPHandRetargeter
from whatslab.solvers.hand.human_fk import (BONE_LINKS, FINGERS, HumanHandFK,
                                            link_candidates, palm_frame_from_fingers)

MIRROR = np.diag([1.0, 1.0, -1.0])
PROBE_DEG = (5.0, 15.0, 25.0)


def human_rig(side):
    fk = HumanHandFK(side)
    chains = {f: [next(c for c in link_candidates(side, jn)
                       if fk.model.existFrame(c)) for jn in BONE_LINKS[f]]
              for f in FINGERS}
    return fk.model, fk.data, chains, list(fk.joint_names), fk._idx_q


def robot_rig(side, cfg):
    kp = KPHandRetargeter(side, cfg)
    chains = {f: list(kp.keypoints[f]) for f in FINGERS}
    return kp.model, kp.data, chains, list(kp.joint_names), dict(kp._out)


def chain_local(model, data, chains, q):
    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)
    fid = {f: [model.getFrameId(n, pin.FrameType.BODY) for n in chains[f]]
           for f in FINGERS}
    pts = {f: np.array([data.oMf[i].translation.copy() for i in fid[f]])
           for f in FINGERS}
    o, R = palm_frame_from_fingers(pts)
    return {f: np.array([R.T @ (p - o) for p in pts[f]]) for f in FINGERS}


def report(label, rigs):
    (ml, dl, cl, nl, il), (mr, dr, cr, nr, ir) = rigs
    base = {s: chain_local(m, d, c, pin.neutral(m))
            for s, (m, d, c) in (("left", (ml, dl, cl)), ("right", (mr, dr, cr)))}
    neutral = max(float(np.abs(base["left"][f] - base["right"][f] @ MIRROR).max())
                  for f in FINGERS) * 1e3
    if len(nl) != len(nr):
        print("%-16s 관절 수 불일치 L=%d R=%d — 통합 불가" % (label, len(nl), len(nr)))
        return
    rows = []
    for a, b in zip(nl, nr):
        worst = 0.0
        for deg in PROBE_DEG:
            loc = {}
            for s, (m, d, c, idx, name) in (
                    ("left", (ml, dl, cl, il, a)), ("right", (mr, dr, cr, ir, b))):
                q = pin.neutral(m)
                q[idx[name]] = np.deg2rad(deg)
                loc[s] = chain_local(m, d, c, q)
            worst = max(worst, max(
                float(np.abs(loc["left"][f] - loc["right"][f] @ MIRROR).max())
                for f in FINGERS) * 1e3)
        rows.append((worst, a, b))
    rows.sort(reverse=True)
    bad = [r for r in rows if r[0] > 2.0]
    print("%-16s 중립 %6.2fmm   관절 미러오차>2mm %2d/%-2d   최악 %7.2fmm (%s)" % (
        label, neutral, len(bad), len(rows), rows[0][0], rows[0][1][-24:]))
    for w, a, _ in bad[:6]:
        print("    %-40s %7.2fmm" % (a[-40:], w))
    return len(bad) == 0 and neutral <= 2.0


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--configs", nargs="+",
                    default=["human", "orca_hand", "robotis_hx5_d20"])
    args = ap.parse_args()
    ok = {}
    for cfg in args.configs:
        if cfg == "human":
            rigs = (human_rig("left"), human_rig("right"))
        else:
            rigs = (robot_rig("left", cfg), robot_rig("right", cfg))
        ok[cfg] = report(cfg, rigs)
    print()
    for cfg, passed in ok.items():
        print("%-16s 통합 게이트 %s" % (cfg, "통과" if passed else "미통과"))


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: 현재 상태를 기록한다**

```bash
WHATSLAB_MODELS_ROOT=/home/whatslab09/whatslab-models/dexhand_description \
  /home/whatslab09/micromamba/envs/dex_mj/bin/python tools/check_mirror.py
```
Expected: human/orca/robotis 세 줄 + 통합 게이트 판정. 현재는 셋 다 미통과여야 한다
(`plans/RETARGET_V2_PLAN.md` §0.1 과 값이 일치하는지 확인 — 다르면 이 도구가 틀렸다).

- [ ] **Step 3: 커밋**

```bash
git add tools/check_mirror.py
git commit -m "feat(tools): 좌우 URDF 미러 검사 — 좌우 통합 모델의 수용 기준

같은 q 를 좌우에 넣고 팜프레임 로컬 사슬 좌표를 (x, y, -z) 로 비교한다.
5·15·25도 세 지점에서 재는 이유는 축 부호가 틀린 경우 오차가 각도에 비례해
커지므로 한 지점만 보면 크기를 오판한다.

현재: human 4/21(mcp_abd), orca 3/16(외전축), robotis 20/20 미통과."
```

---

## File Structure

| 파일 | 책임 |
|---|---|
| `src/whatslab/solvers/hand/keyvector.py` (신규) | 전처리 전담. dorsum 원점 + 손가락방향 축 + 사슬 50% prox + `L_ref` 상수 → `(5,6)` keyvector. 사람·로봇 공용. torch 무관, numpy·pinocchio 만. |
| `src/whatslab/solvers/hand/fk_torch.py` (신규) | `keyvector.py` 의 encode/jacobian 을 `torch.autograd.Function` 으로 감싼다. 학습 전용. |
| `src/whatslab/solvers/hand/net_retargeter.py` (신규) | `F` 정의(손가락별 MLP + tanh→roms)와 추론 리타게터. `compute(joint_angles) -> q` 로 `KPHandRetargeter` 와 같은 계약. |
| `tools/bench_hand_retarget.py` (수정) | 현재의 전역 방식을 `GMC` 로 명확히 하고 **LMC 를 추가**. `net` 백엔드 연결. |
| `tools/train_hand_net.py` (신규) | 학습 루프 + GeoRT 5원칙 손실 + 에폭별 체크포인트/resume. |
| `tests/test_hand_keyvector.py` (신규) | 사슬 가중, `L_ref` 정규화, prox 위치, 좌우 대칭. |
| `tests/test_hand_fk_torch.py` (신규) | 해석 야코비안 vs 수치 미분. |
| `tests/test_hand_net.py` (신규) | 출력이 관절범위 안, 계약 일치. |
| `tests/test_hand_metrics.py` (신규) | LMC/GMC 를 합성 입력으로 검증. |

**핵심 설계 근거 — prox 는 고정 아핀 조합이다.** 강체 사슬은 뼈 길이가 q 에 무관하므로 "호길이 50% 지점"이 들어가는 구간 `k` 와 보간 비율 `t` 는 **q=0 에서 한 번 계산하면 상수**다. 따라서 `prox = (1-t)·p_k + t·p_{k+1}` 는 미분 가능하고 자세에 따라 구간이 튀지 않는다. 프레임(회전)과 `L_ref`, dorsum 원점도 전부 q=0 에서 한 번 계산해 상수로 고정한다 → 인위적 스케일 jitter 가 없고 체크포인트에 데이터셋 통계를 딸려보낼 필요가 없다.

---

## Task 1: keyvector 전처리 모듈

**Files:**
- Create: `src/whatslab/solvers/hand/keyvector.py`
- Test: `tests/test_hand_keyvector.py`

**Interfaces:**
- Consumes: `whatslab.solvers.hand.human_fk.FINGERS`, `palm_frame_from_fingers`
- Produces:
  - `chain_weights(seg_lengths, frac=0.5) -> Tuple[int, float]`
  - `HandKeyvector(model, data, chains: Dict[str, Sequence[str]], dorsum_frame: str, frac: float = 0.5)`
    - `.l_ref: float`, `.origin: np.ndarray (3,)`, `.rot: np.ndarray (3,3)`, `.mid: Dict[str, Tuple[int, float]]`
    - `.encode(q: np.ndarray) -> np.ndarray (5, 6)`
    - `.jacobian(q: np.ndarray, cols: Sequence[int]) -> np.ndarray (5, 6, len(cols))`
    - `.points(q) -> Dict[str, np.ndarray (n, 3)]`

- [ ] **Step 1: 실패하는 테스트를 쓴다**

`tests/test_hand_keyvector.py`:

```python
import numpy as np
import pytest

pin = pytest.importorskip("pinocchio")

from whatslab.solvers.hand.human_fk import FINGERS, BONE_LINKS, HumanHandFK, link_candidates
from whatslab.solvers.hand.keyvector import HandKeyvector, chain_weights


def test_chain_weights_uniform_segments():
    assert chain_weights([1.0, 1.0]) == (1, 0.0)
    assert chain_weights([1.0, 1.0, 2.0]) == (2, 0.0)
    k, t = chain_weights([3.0, 1.0])
    assert (k, round(t, 6)) == (0, 0.5)


def test_chain_weights_rejects_zero():
    with pytest.raises(ValueError):
        chain_weights([0.0, 0.0])


def _human_kv(side):
    fk = HumanHandFK(side)
    chains = {}
    for f in FINGERS:
        names = []
        for jn in BONE_LINKS[f]:
            names.append(next(c for c in link_candidates(side, jn)
                              if fk.model.existFrame(c)))
        chains[f] = names
    return fk, HandKeyvector(fk.model, fk.data, chains, side + "_sensor_dorsum")


def test_encode_shape_and_lref_normalisation():
    fk, kv = _human_kv("right")
    q0 = pin.neutral(fk.model)
    x = kv.encode(q0)
    assert x.shape == (5, 6)
    mid = FINGERS.index("middle")
    assert np.linalg.norm(x[mid, :3]) == pytest.approx(1.0, abs=1e-9)


def test_prox_lies_on_chain_between_ends():
    fk, kv = _human_kv("right")
    q0 = pin.neutral(fk.model)
    pts = kv.points(q0)
    for f in FINGERS:
        k, t = kv.mid[f]
        p = pts[f][k] * (1.0 - t) + pts[f][k + 1] * t
        segs = np.linalg.norm(np.diff(pts[f], axis=0), axis=1)
        walked = float(segs[:k].sum() + segs[k] * t)
        assert walked == pytest.approx(0.5 * float(segs.sum()), rel=1e-9)
        assert np.linalg.norm(p - pts[f][0]) < np.linalg.norm(pts[f][-1] - pts[f][0])


def test_left_right_keyvectors_mirror_in_x():
    _, kvl = _human_kv("left")
    fkr, kvr = _human_kv("right")
    q0 = pin.neutral(fkr.model)
    xl, xr = kvl.encode(q0), kvr.encode(q0)
    flip = np.array([1.0, 1.0, -1.0] * 2)
    assert np.abs(xl - xr * flip).max() < 2e-3
```

- [ ] **Step 2: 실패를 확인한다**

```bash
cd /home/whatslab09/whatslab-sdk
WHATSLAB_MODELS_ROOT=/home/whatslab09/whatslab-models/dexhand_description \
  /home/whatslab09/micromamba/envs/dex_mj/bin/python -m pytest tests/test_hand_keyvector.py -q
```
Expected: `ModuleNotFoundError: No module named 'whatslab.solvers.hand.keyvector'`

- [ ] **Step 3: 구현한다**

`src/whatslab/solvers/hand/keyvector.py`:

```python
from typing import Dict, Sequence, Tuple

import numpy as np
import pinocchio as pin

from .human_fk import FINGERS, palm_frame_from_fingers


def chain_weights(seg_lengths: Sequence[float], frac: float = 0.5) -> Tuple[int, float]:
    lengths = np.asarray(seg_lengths, dtype=float)
    if lengths.size == 0 or not np.all(lengths >= 0.0):
        raise ValueError("사슬 구간 길이가 비었거나 음수다: %s" % (lengths,))
    total = float(lengths.sum())
    if total <= 1e-12:
        raise ValueError("사슬 전체 길이가 0 이다 — 같은 위치의 프레임을 사슬로 줬다")
    target = total * float(frac)
    acc = 0.0
    for k, seg in enumerate(lengths):
        if seg > 0.0 and acc + seg >= target - 1e-12:
            return k, float(min(max((target - acc) / seg, 0.0), 1.0))
        acc += seg
    return int(lengths.size) - 1, 1.0


class HandKeyvector:

    def __init__(self, model, data, chains: Dict[str, Sequence[str]],
                 dorsum_frame: str, frac: float = 0.5):
        self.model = model
        self.data = data
        self.fids = {f: [self._bid(n) for n in chains[f]] for f in FINGERS}
        for f in FINGERS:
            if len(self.fids[f]) < 2:
                raise ValueError("%s 사슬이 2점 미만이다: %s" % (f, chains[f]))
        self.dorsum = self._bid(dorsum_frame)

        q0 = pin.neutral(model)
        pts = self.points(q0)
        self.mid = {f: chain_weights(
            np.linalg.norm(np.diff(pts[f], axis=0), axis=1), frac) for f in FINGERS}
        self.origin = self._pos(self.dorsum).copy()
        self.rot = palm_frame_from_fingers(pts)[1]
        self.l_ref = float(np.linalg.norm(pts["middle"][-1] - self.origin))
        if self.l_ref <= 1e-9:
            raise ValueError("L_ref 가 0 이다 — dorsum 과 중지 끝이 같은 위치다")

    def _bid(self, name: str) -> int:
        if not self.model.existFrame(name):
            raise ValueError("URDF 에 프레임이 없다: %s" % name)
        return self.model.getFrameId(name, pin.FrameType.BODY)

    def _pos(self, fid: int) -> np.ndarray:
        return self.data.oMf[fid].translation

    def _fk(self, q: np.ndarray) -> None:
        pin.forwardKinematics(self.model, self.data, np.asarray(q, dtype=float))
        pin.updateFramePlacements(self.model, self.data)

    def points(self, q: np.ndarray) -> Dict[str, np.ndarray]:
        self._fk(q)
        return {f: np.array([self._pos(i).copy() for i in self.fids[f]])
                for f in FINGERS}

    def prox(self, pts: Dict[str, np.ndarray], finger: str) -> np.ndarray:
        k, t = self.mid[finger]
        return pts[finger][k] * (1.0 - t) + pts[finger][k + 1] * t

    def encode(self, q: np.ndarray) -> np.ndarray:
        pts = self.points(q)
        out = np.zeros((len(FINGERS), 6))
        for i, f in enumerate(FINGERS):
            out[i, :3] = self.rot.T @ (pts[f][-1] - self.origin) / self.l_ref
            out[i, 3:] = self.rot.T @ (self.prox(pts, f) - self.origin) / self.l_ref
        return out

    def jacobian(self, q: np.ndarray, cols: Sequence[int]) -> np.ndarray:
        qa = np.asarray(q, dtype=float)
        pin.computeJointJacobians(self.model, self.data, qa)
        pin.updateFramePlacements(self.model, self.data)
        idx = np.asarray(cols, dtype=int)
        out = np.zeros((len(FINGERS), 6, idx.size))
        for i, f in enumerate(FINGERS):
            k, t = self.mid[f]
            jt = self._frame_jac(self.fids[f][-1])
            ja = self._frame_jac(self.fids[f][k])
            jb = self._frame_jac(self.fids[f][k + 1])
            jp = ja * (1.0 - t) + jb * t
            out[i, :3] = (self.rot.T @ jt)[:, idx] / self.l_ref
            out[i, 3:] = (self.rot.T @ jp)[:, idx] / self.l_ref
        return out

    def _frame_jac(self, fid: int) -> np.ndarray:
        return pin.getFrameJacobian(self.model, self.data, fid,
                                    pin.LOCAL_WORLD_ALIGNED)[:3]
```

- [ ] **Step 4: 통과를 확인한다**

```bash
WHATSLAB_MODELS_ROOT=/home/whatslab09/whatslab-models/dexhand_description \
  /home/whatslab09/micromamba/envs/dex_mj/bin/python -m pytest tests/test_hand_keyvector.py -q -rs
```
Expected: `5 passed`

`test_left_right_keyvectors_mirror_in_x` 가 실패하면 그것은 코드 버그가 아니라 **선행 게이트 P0-1 의 `mcp_abd` 축 문제**다. 그 경우 테스트에 `pytest.mark.xfail(reason="P0-1 mcp_abd 축 미러 미수정")` 을 붙이고 게이트가 열린 뒤 제거한다 — 스킵하거나 지우지 말 것.

- [ ] **Step 5: `__init__` 등록**

`src/whatslab/solvers/hand/__init__.py` 와 `src/whatslab/solvers/__init__.py` 의 import 와 `__all__` 에 `HandKeyvector`, `chain_weights` 를 추가한다. `docs/API.md` 공개 심볼 표에 두 줄 추가하고 `README.md`·`README.ko.md` 의 심볼 목록도 맞춘다.

- [ ] **Step 6: 커밋**

```bash
git add src/whatslab/solvers/hand/keyvector.py src/whatslab/solvers/hand/__init__.py \
        src/whatslab/solvers/__init__.py tests/test_hand_keyvector.py \
        docs/API.md README.md README.ko.md
git commit -m "feat(hand-kv): dorsum 원점 keyvector 전처리 — 사슬 50% prox 를 고정 아핀으로

강체 사슬은 뼈 길이가 q 에 무관하므로 호길이 50% 지점의 구간 k 와 보간비 t 를
q=0 에서 한 번 구하면 상수다. 자세에 따라 구간이 튀지 않고 미분도 가능하다.
회전·L_ref·dorsum 원점도 q=0 에서 고정해 스케일 jitter 를 없앤다."
```

---

## Task 2: pinocchio FK 를 torch 미분 경로로 연결 + P0-3 성능 측정

**Files:**
- Create: `src/whatslab/solvers/hand/fk_torch.py`
- Test: `tests/test_hand_fk_torch.py`

**Interfaces:**
- Consumes: `HandKeyvector` (Task 1)
- Produces:
  - `KeyvectorFK(kv: HandKeyvector, idx_q: Sequence[int], idx_v: Sequence[int], q_template: np.ndarray)`
    - `.forward(q_act: torch.Tensor (B, n_act)) -> torch.Tensor (B, 5, 6)`
  - `keyvector_fk(q_act, fk) -> torch.Tensor` (autograd 진입점)

**주의 — `idx_q` 와 `idx_v` 를 섞지 말 것.** `HandKeyvector.jacobian` 이 인덱싱하는 것은
야코비안의 **열**이고 그건 속도 공간(`joint.idx_v`)이다. 반면 `expand` 가 채우는 것은
형상 벡터(`joint.idx_q`)다. 이 손들은 전부 1-DoF 회전관절이라 두 값이 우연히 같지만,
같은 리스트를 양쪽에 넘기면 다른 로봇에서 조용히 틀린다. **두 인덱스를 분리해서 받고,
길이가 다르면 즉시 에러를 낸다.**

- [ ] **Step 1: 실패하는 테스트를 쓴다**

`tests/test_hand_fk_torch.py`:

```python
import numpy as np
import pytest

pin = pytest.importorskip("pinocchio")
torch = pytest.importorskip("torch")

from whatslab.solvers.hand import KPHandRetargeter
from whatslab.solvers.hand.fk_torch import KeyvectorFK
from whatslab.solvers.hand.keyvector import HandKeyvector


def _robot_fk(side="right", cfg="robotis_hx5_d20"):
    kp = KPHandRetargeter(side, cfg)
    chains = {f: list(kp.keypoints[f]) for f in kp.keypoints}
    kv = HandKeyvector(kp.model, kp.data, chains, side + "_sensor_dorsum")
    iq = dict(kp._out)
    iv = {kp.model.names[j]: int(kp.model.joints[j].idx_v)
          for j in range(1, kp.model.njoints) if kp.model.joints[j].nq > 0}
    return kp, KeyvectorFK(kv, [iq[n] for n in kp.joint_names],
                           [iv[n] for n in kp.joint_names], pin.neutral(kp.model))


def test_forward_matches_numpy_encode():
    kp, fk = _robot_fk()
    q = torch.zeros(1, len(kp.joint_names), dtype=torch.float64)
    got = fk(q).detach().numpy()[0]
    want = fk.kv.encode(pin.neutral(kp.model))
    assert np.abs(got - want).max() < 1e-12


def test_gradient_matches_finite_difference():
    kp, fk = _robot_fk()
    n = len(kp.joint_names)
    rng = np.random.default_rng(0)
    q0 = rng.uniform(-0.2, 0.2, (2, n))
    q = torch.tensor(q0, dtype=torch.float64, requires_grad=True)
    w = torch.tensor(rng.normal(size=(2, 5, 6)), dtype=torch.float64)
    (fk(q) * w).sum().backward()
    got = q.grad.numpy()
    eps, num = 1e-6, np.zeros_like(q0)
    for b in range(q0.shape[0]):
        for j in range(n):
            d = np.zeros_like(q0)
            d[b, j] = eps
            a = (fk(torch.tensor(q0 + d)).detach().numpy() * w.numpy()).sum()
            c = (fk(torch.tensor(q0 - d)).detach().numpy() * w.numpy()).sum()
            num[b, j] = (a - c) / (2 * eps)
    assert np.abs(got - num).max() < 1e-5
```

- [ ] **Step 2: 실패를 확인한다**

```bash
WHATSLAB_MODELS_ROOT=/home/whatslab09/whatslab-models/dexhand_description \
  /home/whatslab09/micromamba/envs/dex_mj/bin/python -m pytest tests/test_hand_fk_torch.py -q
```
Expected: `ModuleNotFoundError: No module named 'whatslab.solvers.hand.fk_torch'`

- [ ] **Step 3: 구현한다**

`src/whatslab/solvers/hand/fk_torch.py`:

```python
from typing import Sequence

import numpy as np
import torch

from .keyvector import HandKeyvector


class _KeyvectorFKFn(torch.autograd.Function):

    @staticmethod
    def forward(ctx, q_act, fk):
        qa = q_act.detach().cpu().numpy().astype(float)
        out = np.empty((qa.shape[0], 5, 6))
        jac = np.empty((qa.shape[0], 5, 6, qa.shape[1]))
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
        return torch.einsum("bfk,bfkj->bj", grad_out, jac), None


class KeyvectorFK(torch.nn.Module):

    def __init__(self, kv: HandKeyvector, idx_q: Sequence[int],
                 idx_v: Sequence[int], q_template: np.ndarray):
        super().__init__()
        self.kv = kv
        self.idx_q = np.asarray(idx_q, dtype=int)
        self.idx_v = np.asarray(idx_v, dtype=int)
        if self.idx_q.size != self.idx_v.size:
            raise ValueError("idx_q %d 개와 idx_v %d 개가 다르다" % (
                self.idx_q.size, self.idx_v.size))
        self._q0 = np.asarray(q_template, dtype=float).copy()

    def expand(self, q_act: np.ndarray) -> np.ndarray:
        q = self._q0.copy()
        q[self.idx_q] = q_act
        return q

    def forward(self, q_act: torch.Tensor) -> torch.Tensor:
        return _KeyvectorFKFn.apply(q_act, self)


def keyvector_fk(q_act: torch.Tensor, fk: KeyvectorFK) -> torch.Tensor:
    return fk(q_act)
```

- [ ] **Step 4: 통과를 확인한다**

```bash
WHATSLAB_MODELS_ROOT=/home/whatslab09/whatslab-models/dexhand_description \
  /home/whatslab09/micromamba/envs/dex_mj/bin/python -m pytest tests/test_hand_fk_torch.py -q -rs
```
Expected: `2 passed`

- [ ] **Step 5: P0-3 — 배치 FK 처리량을 측정하고 기록한다**

```bash
WHATSLAB_MODELS_ROOT=/home/whatslab09/whatslab-models/dexhand_description \
/home/whatslab09/micromamba/envs/dex_mj/bin/python - <<'EOF'
import time
import numpy as np
import pinocchio as pin
import torch
from whatslab.solvers.hand import KPHandRetargeter
from whatslab.solvers.hand.fk_torch import KeyvectorFK
from whatslab.solvers.hand.keyvector import HandKeyvector

kp = KPHandRetargeter("right", "robotis_hx5_d20")
kv = HandKeyvector(kp.model, kp.data, {f: list(v) for f, v in kp.keypoints.items()},
                   "right_sensor_dorsum")
iq = dict(kp._out)
iv = {kp.model.names[j]: int(kp.model.joints[j].idx_v)
      for j in range(1, kp.model.njoints) if kp.model.joints[j].nq > 0}
fk = KeyvectorFK(kv, [iq[n] for n in kp.joint_names],
                 [iv[n] for n in kp.joint_names], pin.neutral(kp.model))
for B in (64, 256, 1024):
    q = torch.zeros(B, len(kp.joint_names), dtype=torch.float64, requires_grad=True)
    t0 = time.perf_counter()
    fk(q).sum().backward()
    dt = time.perf_counter() - t0
    print("batch %5d  forward+backward %7.1f ms  %8.0f samples/s" % (B, dt * 1e3, B / dt))
EOF
```

측정값을 `plans/RETARGET_V2_PLAN.md` §3 P0-3 행에 적는다. **판정**: 배치 256 에서 100ms 를 넘으면 학습 스텝의 지배 항이 된다 → Task 5 에서 `n_random` 을 줄이고 로봇 keyvector bank 를 사전계산으로 돌린다. 그래도 부족하면 `pytorch_kinematics` 를 학습에만 병행하고 결과를 정확 FK 와 대조 검증한다(추론은 계속 FK 없음이라 무관).

- [ ] **Step 6: 커밋**

```bash
git add src/whatslab/solvers/hand/fk_torch.py tests/test_hand_fk_torch.py \
        src/whatslab/solvers/hand/__init__.py src/whatslab/solvers/__init__.py \
        docs/API.md README.md README.ko.md plans/RETARGET_V2_PLAN.md
git commit -m "feat(hand-fk): pinocchio 정확 FK 를 autograd.Function 으로 미분 통과

dL/dq = (dL/dx)^T J. 프레임 야코비안을 LOCAL_WORLD_ALIGNED 로 받아 keyvector
정의(고정 회전·L_ref·prox 아핀)에 맞춰 변환한다. 신경 FK 근사 오차를 구조적으로
없앤다. 수치 미분 대조 테스트로 검증했다."
```

---

## Task 3: F 네트워크 + 추론 리타게터

**Files:**
- Create: `src/whatslab/solvers/hand/net_retargeter.py`
- Test: `tests/test_hand_net.py`

**Interfaces:**
- Consumes: `HandKeyvector` (Task 1), `HumanHandFK`
- Produces:
  - `HandNet(in_dim: int, joint_counts: Sequence[int], hidden: int = 128)` — `nn.Module`, `.forward(x: (B,5,in_dim)) -> (B, sum(joint_counts))` 출력은 `[-1,1]`
  - `NetHandRetargeter(side, hand_type, checkpoint=None, config_name="base_hand", urdf_root=None)`
    - `.joint_names: List[str]`, `.human_joint_names: List[str]`, `._iq: List[int]`(idx_q), `._iv: List[int]`(idx_v)
    - `.compute(joint_angles: Mapping[str, float]) -> np.ndarray`
    - `.reset() -> None`
    - `.state_dict()`, `.load_state_dict(sd)`

- [ ] **Step 1: 실패하는 테스트를 쓴다**

`tests/test_hand_net.py`:

```python
import numpy as np
import pytest

pin = pytest.importorskip("pinocchio")
torch = pytest.importorskip("torch")

from whatslab.solvers.hand.net_retargeter import HandNet, NetHandRetargeter


def test_handnet_output_range_and_width():
    net = HandNet(6, [4, 4, 4, 4, 4]).double()
    x = torch.randn(3, 5, 6, dtype=torch.float64) * 5.0
    y = net(x)
    assert y.shape == (3, 20)
    assert float(y.abs().max()) <= 1.0


def test_retargeter_contract_and_limits():
    r = NetHandRetargeter("right", "robotis_hx5_d20")
    zero = {n: 0.0 for n in r.human_joint_names}
    q = r.compute(zero)
    assert q.shape == (len(r.joint_names),)
    assert np.all(q >= r.lower - 1e-9) and np.all(q <= r.upper + 1e-9)


def test_retargeter_is_deterministic():
    r = NetHandRetargeter("right", "robotis_hx5_d20")
    zero = {n: 0.0 for n in r.human_joint_names}
    a = r.compute(zero)
    r.reset()
    b = r.compute(zero)
    assert np.abs(a - b).max() == 0.0
```

- [ ] **Step 2: 실패를 확인한다**

```bash
WHATSLAB_MODELS_ROOT=/home/whatslab09/whatslab-models/dexhand_description \
  /home/whatslab09/micromamba/envs/dex_mj/bin/python -m pytest tests/test_hand_net.py -q
```
Expected: `ModuleNotFoundError: No module named 'whatslab.solvers.hand.net_retargeter'`

- [ ] **Step 3: 구현한다**

`src/whatslab/solvers/hand/net_retargeter.py`:

```python
import os
from typing import List, Mapping, Optional, Sequence

import numpy as np
import pinocchio as pin
import torch
import torch.nn as nn

from ...paths import models_root
from .hand_configs import CONFIG_REGISTRY
from .human_fk import BONE_LINKS, FINGERS, HumanHandFK, link_candidates
from .keyvector import HandKeyvector

LIMIT_FALLBACK = 2.0


class HandNet(nn.Module):

    def __init__(self, in_dim: int, joint_counts: Sequence[int], hidden: int = 128):
        super().__init__()
        self.joint_counts = list(joint_counts)
        self.nets = nn.ModuleList([
            nn.Sequential(nn.Linear(in_dim, hidden), nn.LeakyReLU(),
                          nn.Linear(hidden, hidden), nn.LeakyReLU(),
                          nn.Linear(hidden, n), nn.Tanh())
            for n in self.joint_counts])

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return torch.cat([net(x[:, i]) for i, net in enumerate(self.nets)], dim=1)


class NetHandRetargeter:

    def __init__(self, side: str, hand_type: str, checkpoint: Optional[str] = None,
                 config_name: str = "base_hand", urdf_root: Optional[str] = None):
        self.side = side.lower()
        root = urdf_root or models_root()
        cfg = CONFIG_REGISTRY[hand_type]()
        self.urdf_path = cfg.urdf_path(self.side, root)
        self.model = pin.buildModelFromUrdf(self.urdf_path)
        self.data = self.model.createData()

        chains = {f: list(v) for f, v in cfg.keypoint_links(self.side, root).items()}
        self.kv = HandKeyvector(self.model, self.data, chains,
                                "%s_sensor_dorsum" % self.side)

        self.fk = HumanHandFK(self.side, os.path.join(
            root, config_name, "urdf", "%s.urdf" % self.side))
        hchains = {}
        for f in FINGERS:
            hchains[f] = [next(c for c in link_candidates(self.side, jn)
                               if self.fk.model.existFrame(c))
                          for jn in BONE_LINKS[f]]
        self.hkv = HandKeyvector(self.fk.model, self.fk.data, hchains,
                                 "%s_sensor_dorsum" % self.side)
        self.human_joint_names: List[str] = list(self.fk.joint_names)

        self._cols = {f: sorted({int(self.model.joints[j].idx_v)
                                 for j in self.model.supports[
                                     int(self.model.frames[self.kv.fids[f][-1]].parent)]
                                 if j > 0}) for f in FINGERS}
        common = set.intersection(*[set(v) for v in self._cols.values()])
        self._cols = {f: [c for c in v if c not in common] for f, v in self._cols.items()}
        order = [c for f in FINGERS for c in self._cols[f]]
        vidx = {int(self.model.joints[j].idx_v): int(self.model.joints[j].idx_q)
                for j in range(1, self.model.njoints)
                if self.model.joints[j].nq > 0}
        self._iv = [int(c) for c in order]
        self._iq = [vidx[c] for c in order]
        self.joint_names = [self.model.names[j] for c in order
                            for j in range(1, self.model.njoints)
                            if self.model.joints[j].idx_v == c]

        lo = np.where(np.isfinite(self.model.lowerPositionLimit),
                      self.model.lowerPositionLimit, -LIMIT_FALLBACK)
        hi = np.where(np.isfinite(self.model.upperPositionLimit),
                      self.model.upperPositionLimit, LIMIT_FALLBACK)
        self.lower = lo[self._iq].copy()
        self.upper = hi[self._iq].copy()

        self.net = HandNet(6, [len(self._cols[f]) for f in FINGERS]).double()
        self.net.eval()
        if checkpoint is not None:
            self.net.load_state_dict(torch.load(checkpoint, map_location="cpu"))
        self._q = pin.neutral(self.model)

    def state_dict(self):
        return self.net.state_dict()

    def load_state_dict(self, sd) -> None:
        self.net.load_state_dict(sd)

    def reset(self) -> None:
        self._q = pin.neutral(self.model)

    def to_joint(self, unit: np.ndarray) -> np.ndarray:
        return self.lower + (np.asarray(unit, dtype=float) + 1.0) * 0.5 * (
            self.upper - self.lower)

    def compute(self, joint_angles: Mapping[str, float]) -> np.ndarray:
        x = self.hkv.encode(self.fk.q_from_named(joint_angles))
        with torch.no_grad():
            unit = self.net(torch.as_tensor(x, dtype=torch.float64).unsqueeze(0))
        q_act = self.to_joint(unit.numpy()[0])
        self._q = pin.neutral(self.model)
        self._q[self._iq] = q_act
        return q_act
```

`cfg.keypoint_links(side, root)` 가 없으면 `hand_configs/_base.py` 의 유도 결과를 그대로 노출하는 얇은 메서드를 거기에 추가한다(`_derive` 가 이미 손가락별 사슬을 만들고 있으므로 반환만 한다). 이 경우 그 파일 변경도 같은 커밋에 넣는다.

- [ ] **Step 4: 통과를 확인한다**

```bash
WHATSLAB_MODELS_ROOT=/home/whatslab09/whatslab-models/dexhand_description \
  /home/whatslab09/micromamba/envs/dex_mj/bin/python -m pytest tests/test_hand_net.py -q -rs
```
Expected: `3 passed`

- [ ] **Step 5: 전체 테스트가 깨지지 않았는지 확인한다**

```bash
WHATSLAB_MODELS_ROOT=/home/whatslab09/whatslab-models/dexhand_description \
  /home/whatslab09/micromamba/envs/dex_mj/bin/python -m pytest -q -rs
```
Expected: `124 passed, 6 skipped` (기존 116 + 신규 8)

- [ ] **Step 6: 커밋**

```bash
git add src/whatslab/solvers/hand/net_retargeter.py tests/test_hand_net.py \
        src/whatslab/solvers/hand/hand_configs/_base.py \
        src/whatslab/solvers/hand/__init__.py src/whatslab/solvers/__init__.py \
        docs/API.md README.md README.ko.md
git commit -m "feat(hand-net): 손가락별 MLP 리타게터 — tanh 를 joint_roms 로 선형 매핑

추론 경로에 FK 가 없다(forward 1회). 데이터셋 통계 정규화를 쓰지 않으므로
체크포인트에 통계를 딸려보낼 필요가 없고 추론 시 통계 불일치가 없다.
계약은 KPHandRetargeter 와 같은 compute(joint_angles) -> q 다."
```

---

## Task 4: LMC 추가 + 현재 지표를 GMC 로 명확화

**Files:**
- Modify: `tools/bench_hand_retarget.py`
- Test: `tests/test_hand_metrics.py`

**Interfaces:**
- Produces (`tools/bench_hand_retarget.py` 안):
  - `motion_consistency(h_prev, h_cur, r_prev, r_cur, r_local) -> Tuple[float, float]` — `(gmc_cos, lmc_cos)`
  - `local_frames(pts: Dict[str, np.ndarray]) -> Dict[str, np.ndarray]` — 손가락별 3x3 로컬 회전
  - `measure(...)` 반환에 `lmc_pct`, `lmc_cos` 추가

**LMC 정의 (구현 확정).** GMC 는 공유(팜) 좌표계에서 변위 방향을 비교한다 — 현재 구현이 이것이다. LMC 는 **각 손가락의 로컬 프레임에서** 비교한다. 로컬 프레임은 그 손가락의 마지막 뼈 방향을 기준으로 만든다: `z = (distal − prox)/|·|`, `x = z × palm_y` 정규화(특이하면 `palm_x` 로 대체), `y = z × x`. 사람·로봇 각각 자기 로컬 프레임으로 변위를 옮긴 뒤 코사인을 잰다. 이렇게 하면 사람↔로봇의 전역 정렬 오차(캘리 품질)가 상쇄되고, **GMC 와 LMC 의 격차가 곧 정렬 품질의 진단값**이 된다.

- [ ] **Step 1: 실패하는 테스트를 쓴다**

`tests/test_hand_metrics.py`:

```python
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))), "tools"))

pytest.importorskip("pinocchio")
import bench_hand_retarget as B  # noqa: E402


def test_local_frames_are_orthonormal_right_handed():
    pts = {f: np.array([[0.0, 0.0, 0.0], [0.0, 0.01, 0.0], [0.0, 0.02, 0.0],
                        [0.0, 0.03, 0.0]]) for f in B.FINGERS}
    frames = B.local_frames(pts)
    for f in B.FINGERS:
        R = frames[f]
        assert np.abs(R.T @ R - np.eye(3)).max() < 1e-9
        assert np.linalg.det(R) == pytest.approx(1.0, abs=1e-9)


def test_gmc_penalises_rotation_but_lmc_does_not():
    h_prev = {f: np.zeros(3) for f in B.FINGERS}
    h_cur = {f: np.array([0.0, 0.001, 0.0]) for f in B.FINGERS}
    theta = np.deg2rad(40.0)
    rot = np.array([[np.cos(theta), -np.sin(theta), 0.0],
                    [np.sin(theta), np.cos(theta), 0.0], [0.0, 0.0, 1.0]])
    r_prev = {f: np.zeros(3) for f in B.FINGERS}
    r_cur = {f: rot @ h_cur[f] for f in B.FINGERS}
    r_local = {f: rot for f in B.FINGERS}
    h_local = {f: np.eye(3) for f in B.FINGERS}
    gmc, lmc = B.motion_consistency(h_prev, h_cur, r_prev, r_cur, h_local, r_local)
    assert gmc == pytest.approx(np.cos(theta), abs=1e-9)
    assert lmc == pytest.approx(1.0, abs=1e-9)
```

- [ ] **Step 2: 실패를 확인한다**

```bash
WHATSLAB_MODELS_ROOT=/home/whatslab09/whatslab-models/dexhand_description \
  /home/whatslab09/micromamba/envs/dex_mj/bin/python -m pytest tests/test_hand_metrics.py -q
```
Expected: `AttributeError: module 'bench_hand_retarget' has no attribute 'local_frames'`

- [ ] **Step 3: 구현한다**

`tools/bench_hand_retarget.py` 의 `LMC_MIN_MM = 0.5` 아래에 추가한다:

```python
def local_frames(pts):
    out = {}
    ref = np.array([1.0, 0.0, 0.0])
    for f in FINGERS:
        z = pts[f][-1] - pts[f][-2]
        nz = float(np.linalg.norm(z))
        if nz < 1e-9:
            raise ValueError("%s 마지막 뼈 길이가 0 이다" % f)
        z = z / nz
        x = np.cross(z, np.array([0.0, 1.0, 0.0]))
        if float(np.linalg.norm(x)) < 1e-3:
            x = np.cross(z, ref)
        x = x / float(np.linalg.norm(x))
        out[f] = np.column_stack([x, np.cross(z, x), z])
    return out


def motion_consistency(h_prev, h_cur, r_prev, r_cur, h_local, r_local):
    gmc, lmc = [], []
    for f in FINGERS:
        u = h_cur[f] - h_prev[f]
        if float(np.linalg.norm(u)) * 1e3 < LMC_MIN_MM:
            continue
        v = r_cur[f] - r_prev[f]
        nv = float(np.linalg.norm(v))
        if nv < 1e-9:
            gmc.append(-1.0)
            lmc.append(-1.0)
            continue
        un, vn = u / float(np.linalg.norm(u)), v / nv
        gmc.append(float(un @ vn))
        lmc.append(float((h_local[f].T @ un) @ (r_local[f].T @ vn)))
    if not gmc:
        return None, None
    return float(np.mean(gmc)), float(np.mean(lmc))
```

`measure` 안에서: 사람 사슬 점(`fair.chain_points(ang)`)과 로봇 사슬 점(`bones_of()`)으로 매 프레임 `local_frames` 를 만들고, 기존 `acc["lmc"]` 블록을 `motion_consistency` 호출로 바꾼다. 누적기는 `acc["gmc"]`, `acc["lmc"]` 두 개로 나눈다. 반환 튜플에 `100*mean(gmc>0)`, `100*mean(gmc)`, `100*mean(lmc>0)`, `100*mean(lmc)` 를 넣고, 표 헤더를 `GMC% / gcos / LMC% / lcos` 로 바꾼다. `Fair` 에 `chain_points(angles)` 를 추가한다(`self.fk.points(angles)` 를 고정 팜 프레임으로 옮긴 것).

- [ ] **Step 4: 통과를 확인한다**

```bash
WHATSLAB_MODELS_ROOT=/home/whatslab09/whatslab-models/dexhand_description \
  /home/whatslab09/micromamba/envs/dex_mj/bin/python -m pytest tests/test_hand_metrics.py -q -rs
```
Expected: `2 passed`

- [ ] **Step 5: 기준선을 다시 재고 계획서 표를 갱신한다**

```bash
export WHATSLAB_MODELS_ROOT=/home/whatslab09/whatslab-models/dexhand_description
D=/home/whatslab09/Desktop/Visualizer/calib_dumps/calib_base_left_20260810_154406.json
P=/home/whatslab09/Desktop/Visualizer/profiles
for t in pinch flex abd; do
  /home/whatslab09/micromamba/envs/dex_mj/bin/python tools/bench_hand_retarget.py \
    --dump $D --profiles $P --traj $t
done
```

세 표를 `plans/RETARGET_V2_PLAN.md` §0 에 LMC 열을 포함해 다시 적는다. **GMC 와 LMC 의 격차를 함께 기록한다** — 이 격차가 캘리브레이션 품질의 진단값이다.

- [ ] **Step 6: 커밋**

```bash
git add tools/bench_hand_retarget.py tests/test_hand_metrics.py plans/RETARGET_V2_PLAN.md
git commit -m "feat(bench-hand): LMC 추가 — 기존 지표는 GMC 로 명확히 한다

기존 구현은 공유 팜 프레임에서 변위를 비교하는 전역 방식이라 GMC 다.
LMC 는 손가락별 로컬 프레임(마지막 뼈 방향)으로 옮긴 뒤 비교해 사람↔로봇
전역 정렬 오차를 상쇄한다. 두 값의 격차가 정렬 품질 진단값이 된다.
40도 회전만 준 합성 입력에서 GMC=cos40, LMC=1 임을 테스트로 고정했다."
```

---

## Task 5: 학습 스크립트 + GeoRT 5원칙 손실 + 크래시 복원

**Files:**
- Create: `tools/train_hand_net.py`

**Interfaces:**
- Consumes: `NetHandRetargeter`(Task 3), `KeyvectorFK`(Task 2), `HandKeyvector`(Task 1), `bench_hand_retarget.load_poses`
- Produces: 체크포인트 `<out>/last.pt` (`{"net": state_dict, "epoch": int, "cfg": dict}`)

**손실 (Phase 1 = GeoRT 5원칙 그대로, 수정 없음).** `x` = 사람 keyvector `(B,5,6)`, `y = fk(to_joint(net(x)))` = 로봇 keyvector `(B,5,6)`.

| | 식 | 가중 |
|---|---|---|
| motion | `-mean(cos(Δx, Δy))`, `Δ` 는 `x` 에 `U(0.001,0.011)` 크기 랜덤 섭동 | 1.0 |
| coverage | `y[:, i, :3]` 와 로봇 keyvector bank 의 양방향 Chamfer, 손가락별 합 | 80.0 |
| flatness | `mean((y(x+δ) + y(x-δ) - 2y(x))²)`, `δ` 크기 0.002 | 0.1 |
| pinch | `x` 의 엄지-손가락 distal 거리 < 0.015/L_ref 인 쌍에 `y` 거리 제곱 | 1.0 |
| collision | 0 (Phase 1 미사용, 자리만 둔다) | 0.0 |

- [ ] **Step 1: 스크립트를 쓴다**

`tools/train_hand_net.py`:

```python
#!/usr/bin/env python3
"""사람 keyvector → 로봇 관절각 네트워크를 GeoRT 5원칙 손실로 학습한다 (정확 FK)."""
import argparse
import json
import os

import numpy as np
import pinocchio as pin
import torch
import torch.nn.functional as F

from whatslab.solvers.hand.fk_torch import KeyvectorFK
from whatslab.solvers.hand.human_fk import FINGERS
from whatslab.solvers.hand.net_retargeter import NetHandRetargeter

import bench_hand_retarget as B

PINCH_MM = 15.0


def robot_bank(r, n, seed=0):
    rng = np.random.default_rng(seed)
    u = rng.uniform(-1.0, 1.0, (n, len(r.joint_names)))
    out = np.empty((n, len(FINGERS), 6))
    for i in range(n):
        q = pin.neutral(r.model)
        q[r._iq] = r.to_joint(u[i])
        out[i] = r.kv.encode(q)
    return out


def human_batch(r, trajs, n_random, seed=0):
    rows = [r.hkv.encode(r.fk.q_from_named(a))
            for traj in trajs.values() for a in traj]
    rng = np.random.default_rng(seed)
    lo, hi = r.fk.model.lowerPositionLimit, r.fk.model.upperPositionLimit
    idx = [r.fk._idx_q[n] for n in r.fk.joint_names]
    for _ in range(n_random):
        q = pin.neutral(r.fk.model)
        for i in idx:
            q[i] = rng.uniform(lo[i], hi[i])
        rows.append(r.hkv.encode(q))
    return np.asarray(rows)


def chamfer(a, b):
    d = ((a.unsqueeze(1) - b.unsqueeze(0)) ** 2).sum(-1)
    return d.min(1).values.mean() + d.min(0).values.mean()


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--config", default="robotis_hx5_d20")
    ap.add_argument("--side", default="right")
    ap.add_argument("--dump", required=True)
    ap.add_argument("--profiles", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--epochs", type=int, default=200)
    ap.add_argument("--batch", type=int, default=256)
    ap.add_argument("--bank", type=int, default=20000)
    ap.add_argument("--random", type=int, default=6000)
    ap.add_argument("--lr", type=float, default=1e-4)
    ap.add_argument("--w-coverage", type=float, default=80.0)
    ap.add_argument("--w-flatness", type=float, default=0.1)
    ap.add_argument("--w-pinch", type=float, default=1.0)
    args = ap.parse_args()

    r = NetHandRetargeter(args.side, args.config)
    fk = KeyvectorFK(r.kv, r._iq, r._iv, pin.neutral(r.model))
    _, trajs = B.load_poses(args.dump, args.profiles, 20, ("pinch", "flex", "abd"))
    X = torch.as_tensor(human_batch(r, trajs, args.random), dtype=torch.float64)
    bank = torch.as_tensor(robot_bank(r, args.bank), dtype=torch.float64)
    lo = torch.as_tensor(r.lower, dtype=torch.float64)
    hi = torch.as_tensor(r.upper, dtype=torch.float64)
    pinch_thr = PINCH_MM * 1e-3 / r.hkv.l_ref

    os.makedirs(args.out, exist_ok=True)
    ckpt = os.path.join(args.out, "last.pt")
    net = r.net.double()
    opt = torch.optim.AdamW(net.parameters(), lr=args.lr)
    start = 0
    if os.path.exists(ckpt):
        sd = torch.load(ckpt, map_location="cpu")
        net.load_state_dict(sd["net"])
        opt.load_state_dict(sd["opt"])
        start = int(sd["epoch"]) + 1
        print("[resume] epoch %d 부터" % start, flush=True)

    def to_kv(x):
        return fk(lo + (net(x) + 1.0) * 0.5 * (hi - lo))

    for epoch in range(start, args.epochs):
        perm = torch.randperm(X.shape[0])
        acc = np.zeros(4)
        nb = 0
        for s in range(0, X.shape[0] - args.batch + 1, args.batch):
            x = X[perm[s:s + args.batch]]
            y = to_kv(x)

            d = F.normalize(torch.randn_like(x), dim=-1)
            scale = 0.001 + torch.rand(x.shape[0], 1, 1, dtype=x.dtype) * 0.01
            yp = to_kv(x + d * scale)
            a = F.normalize((d * scale).reshape(-1, 6), dim=-1, eps=1e-5)
            b = F.normalize((yp - y).reshape(-1, 6), dim=-1, eps=1e-5)
            motion = -(a * b).sum(-1).mean()

            d2 = F.normalize(torch.randn_like(x), dim=-1) * 0.002
            flat = ((to_kv(x + d2) + to_kv(x - d2) - 2.0 * y) ** 2).mean()

            sel = torch.randint(0, bank.shape[0], (min(2048, bank.shape[0]),))
            cover = sum(chamfer(y[:, i, :3], bank[sel][:, i, :3])
                        for i in range(len(FINGERS)))

            pinch = torch.zeros((), dtype=x.dtype)
            for i in range(1, len(FINGERS)):
                mask = (torch.norm(x[:, 0, :3] - x[:, i, :3], dim=-1) < pinch_thr)
                if bool(mask.any()):
                    pinch = pinch + ((y[mask, 0, :3] - y[mask, i, :3]) ** 2).sum(-1).mean()

            loss = (motion + cover * args.w_coverage + flat * args.w_flatness
                    + pinch * args.w_pinch)
            opt.zero_grad()
            loss.backward()
            opt.step()
            acc += [float(motion), float(cover), float(flat), float(pinch)]
            nb += 1

        acc /= max(nb, 1)
        print("epoch %3d  motion %+.4f  cover %.3e  flat %.3e  pinch %.3e"
              % (epoch, *acc), flush=True)
        torch.save({"net": net.state_dict(), "opt": opt.state_dict(),
                    "epoch": epoch, "cfg": vars(args)}, ckpt + ".tmp")
        os.replace(ckpt + ".tmp", ckpt)
        torch.save(net.state_dict(), os.path.join(args.out, "net.pt"))
    with open(os.path.join(args.out, "meta.json"), "w") as fh:
        json.dump(vars(args), fh, indent=2)


if __name__ == "__main__":
    main()
```

`tools/` 를 import 경로에 넣기 위해 스크립트 최상단 import 전에 `sys.path` 조작이 필요하면 `tools/train_hand_net.py` 를 `tools/` 안에서 실행한다(`cd tools && python train_hand_net.py …`). lazy import 금지 규칙 때문에 함수 안에서 `sys.path` 를 만지지 않는다.

- [ ] **Step 2: 1 에폭만 돌려 형태를 확인한다**

```bash
export WHATSLAB_MODELS_ROOT=/home/whatslab09/whatslab-models/dexhand_description
cd /home/whatslab09/whatslab-sdk/tools
/home/whatslab09/micromamba/envs/dex_mj/bin/python train_hand_net.py \
  --config robotis_hx5_d20 --side left \
  --dump /home/whatslab09/Desktop/Visualizer/calib_dumps/calib_base_left_20260810_154406.json \
  --profiles /home/whatslab09/Desktop/Visualizer/profiles \
  --out /home/whatslab09/geort-lab/run/robotis_left_p1 \
  --epochs 1 --bank 2000 --random 500
```
Expected: `epoch   0  motion ...` 한 줄 + `last.pt` 생성

- [ ] **Step 3: resume 이 실제로 동작하는지 확인한다**

같은 명령을 `--epochs 2` 로 다시 실행한다.
Expected: 첫 줄이 `[resume] epoch 1 부터`, 그 다음 `epoch   1 ...` 만 출력.

이 확인이 중요한 이유: 이 PC 는 GPU 부하 중 **하드 리셋**이 재현된다(08:30·08:38 두 번, 커널 로그가 정상 종료 없이 끊김). 리셋 한 번에 런 전체를 잃지 않으려면 resume 이 반드시 동작해야 한다.

- [ ] **Step 4: 본 학습을 detach 로 돌린다**

```bash
cat > /home/whatslab09/geort-lab/run_p1.sh <<'EOF'
#!/bin/bash
export WHATSLAB_MODELS_ROOT=/home/whatslab09/whatslab-models/dexhand_description
cd /home/whatslab09/whatslab-sdk/tools
PY=/home/whatslab09/micromamba/envs/dex_mj/bin/python
for i in $(seq 1 20); do
  $PY train_hand_net.py --config robotis_hx5_d20 --side left \
    --dump /home/whatslab09/Desktop/Visualizer/calib_dumps/calib_base_left_20260810_154406.json \
    --profiles /home/whatslab09/Desktop/Visualizer/profiles \
    --out /home/whatslab09/geort-lab/run/robotis_left_p1 --epochs 200 \
    >> /home/whatslab09/geort-lab/run/robotis_left_p1.log 2>&1 && break
  echo "[retry $i] 비정상 종료 — resume 한다" >> /home/whatslab09/geort-lab/run/robotis_left_p1.log
  sleep 10
done
echo DONE >> /home/whatslab09/geort-lab/run/robotis_left_p1.log
EOF
chmod +x /home/whatslab09/geort-lab/run_p1.sh
mkdir -p /home/whatslab09/geort-lab/run
setsid nohup /home/whatslab09/geort-lab/run_p1.sh >/dev/null 2>&1 < /dev/null &
```

재부팅으로 죽으면 재부팅 후 같은 명령을 한 번 더 실행하면 `last.pt` 에서 이어진다.

- [ ] **Step 5: 커밋**

```bash
cd /home/whatslab09/whatslab-sdk
git add tools/train_hand_net.py
git commit -m "feat(hand-net): GeoRT 5원칙 손실 학습 스크립트 — 에폭별 체크포인트/resume

이 PC 는 GPU 부하 중 하드 리셋이 재현되므로(커널 로그가 정상 종료 없이 끊김)
에폭마다 원자적으로 last.pt 를 쓰고 재시작 시 옵티마이저까지 이어받는다.
손실은 Phase 1 이므로 GeoRT 원본 그대로 두고, 단방향 Chamfer·L_dist·로컬
모션은 Phase 2 에서 바꾼다."
```

---

## Task 6: 벤치에 `net` 백엔드 연결 후 Phase 1 판정

**Files:**
- Modify: `tools/bench_hand_retarget.py`

**Interfaces:**
- Consumes: `NetHandRetargeter`(Task 3), `measure`/`kp_probe`(Task 4)
- Produces: `--backends net` + `--net-checkpoint` 인자

- [ ] **Step 1: 백엔드를 연결한다**

`tools/bench_hand_retarget.py` 의 `main` 에서:

```python
    ap.add_argument("--net-checkpoint", default=None)
```
그리고 백엔드 분기에 추가한다:

```python
            elif be == "net":
                if args.net_checkpoint is None:
                    ap.error("--backends net 은 --net-checkpoint 가 필요하다")
                eng = NetHandRetargeter(side, cfg, checkpoint=args.net_checkpoint)
                r = measure(eng, *net_probe(eng), side, trajs)
```

`net_probe(eng)` 는 `kp_probe` 와 같은 형태로 `eng.model`/`eng.data`/`eng.kv.fids`/`eng._q` 를 쓴다. 최상단에 `from whatslab.solvers.hand.net_retargeter import NetHandRetargeter` 를 추가하고 `--backends` 의 선택지에 `net` 을 넣는다.

- [ ] **Step 2: 세 궤적으로 판정한다**

```bash
export WHATSLAB_MODELS_ROOT=/home/whatslab09/whatslab-models/dexhand_description
D=/home/whatslab09/Desktop/Visualizer/calib_dumps/calib_base_left_20260810_154406.json
P=/home/whatslab09/Desktop/Visualizer/profiles
for t in pinch flex abd; do
  /home/whatslab09/micromamba/envs/dex_mj/bin/python tools/bench_hand_retarget.py \
    --dump $D --profiles $P --traj $t --configs robotis_hx5_d20 \
    --backends kp net \
    --net-checkpoint /home/whatslab09/geort-lab/run/robotis_left_p1/net.pt
done
```

- [ ] **Step 3: 판정을 기록한다**

`plans/RETARGET_V2_PLAN.md` §4 에 결과 표와 판정을 적는다.

**Phase 1 판정 기준** (`ref/planing.md` 의 "43.3° → 20°" 는 무효 — 그 값은 사람 URDF 관절명이 프로파일과 13/21 어긋난 상태의 측정이다):
- **주 판정**: 굽힘 전용 궤적의 형상 오차가 왼손 `robotis kp 31.0°` / `orca kp 32.5°` 보다 낮은가. 이것이 "prox 추가가 형상 추종을 개선하는가"에 대한 답이다.
- **부 판정**: 굽힘 전용 벌림 오차가 왼손 `robotis kp 8.6mm` / `orca kp 10.7mm` 보다 낮은가(미해결 외전 흔들림).
- **참고**: `|dq|p95` 는 왼손 `kp 0.071~0.073`(핀치) 대비 나빠질 것으로 사전 예측한다 — F 는 프레임 독립이라 DLS warm start 가 없다. 나쁘면 ① EMA 입력 필터 → ② Lipschitz 페널티 → ③ 과거 사람 입력 윈도우(k=2~3) 순서로 대응하고, 자기 출력 되먹임은 드리프트 위험이라 쓰지 않는다.
- 기각이라도 근거를 남긴다.

- [ ] **Step 4: 커밋**

```bash
git add tools/bench_hand_retarget.py plans/RETARGET_V2_PLAN.md
git commit -m "feat(bench-hand): net 백엔드 연결 + Phase 1 판정 기록"
```

---

## Task 7: orca 로 교차 확인

**Files:** 없음 (실행만)

- [ ] **Step 1: orca 로 학습한다**

Task 5 Step 4 의 스크립트에서 `--config orca_hand`, `--out …/orca_left_p1` 로 바꿔 같은 방식으로 돌린다.

- [ ] **Step 2: 두 손을 같은 표에서 비교한다**

```bash
for t in pinch flex abd; do
  /home/whatslab09/micromamba/envs/dex_mj/bin/python tools/bench_hand_retarget.py \
    --dump $D --profiles $P --traj $t --configs orca_hand --backends kp net \
    --net-checkpoint /home/whatslab09/geort-lab/run/orca_left_p1/net.pt
done
```

**왜 필요한가**: 비지도 학습은 해가 유일하지 않아 시드·손마다 흔들린다(GeoRT 는 Ability Hand LMC 36.2±11.2 를 보고했다). 한 손에서만 좋아진 것은 Phase 1 통과 근거가 못 된다. AnyDexRT 를 못 믿는 이유 중 하나가 핵심 ablation 이 한 손뿐이라는 점인데, 같은 실수를 반복하지 않는다.

- [ ] **Step 3: 커밋**

```bash
git add plans/RETARGET_V2_PLAN.md
git commit -m "docs(hand-net): Phase 1 을 orca·robotis 두 손에서 판정"
```

---

## 이 계획에서 하지 않는 것

- **Phase 2**(단방향 Chamfer / `L_dist` / 로컬 프레임 motion / `L_align` 앵커 / residual affine)와 **Phase 3**(셀슈바 백본, 다중 임베디먼트 헤드)는 별도 계획으로 뺀다. Phase 1 이 자체로 동작하는 소프트웨어를 내고, 앵커 기여를 측정하려면 앵커 없는 Phase 1 수치가 먼저 있어야 한다.
- **splay 캘리 자세**(`ref/planing.md` VIII-1)는 Phase 2 앵커의 전제이지 Phase 1 의 전제가 아니다. 다만 외전 축 앵커가 없으면 Phase 2 가 굽힘 축만 커버하므로, Phase 2 착수 전에 결론이 나야 한다.
- **좌우 통합 모델** — 손별 미러 게이트(Task 0)가 열린 뒤. Phase 1 산출물은 side 별 모델이다.
- **EMA 입력 필터와 노이즈 aug** — `ref/planing.md` III 의 확정 전처리이지만 Phase 1 에서는
  넣지 않는다. 둘 다 `|dq|` 와 강건성에 영향을 주는 변경이라, 필터 없는 `|dq|` 를 먼저
  측정해야 필터의 기여를 분리할 수 있다(한 번에 하나만 바꾼다). Task 6 Step 3 의 대응
  순서 ①이 EMA 이고, 그때 켠다.
- **GeoRT 벤더링 유지** — Phase 1 은 정확 FK 로 가므로 `geort-lab` 의 신경 FK 경로는 더 쓰지 않는다. 진행 중이던 tip/tip_prox 런은 키포인트 정의와 FK 가 둘 다 바뀌어 무효이므로 폐기한다.
