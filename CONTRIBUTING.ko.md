<p align="center"><a href="CONTRIBUTING.md">English</a> | <b>한국어</b></p>

# 기여 가이드

whatslab-sdk 에 코드를 넣기 전에 읽을 규칙과 그 이유. 설치·사용법은 [README](README.ko.md),
공개 심볼은 [API.md](docs/API.md) 에 있다.

## 범위

이 SDK 는 **손**만 다룬다 — 손 입력 수신, 손 리타게팅, 관절 안전 필터, 시각화.

- 팔 IK, 손목 자세, 팔 제어는 넣지 않는다. 손목 자세는 사용하는 쪽이 자기 트래커에서 받는다.
- 데이터 녹화는 넣지 않는다. SDK 는 단순한 데이터(OpenXR 26관절, `{관절: rad}`, 시각)만 주고 저장 형식은
  사용하는 쪽이 고른다.
- 무언가를 더하고 싶으면 "SDK 를 쓰는 쪽에서 할 수 없는가"를 먼저 따진다. 범위를 넓힌 것이 0.4 까지의
  복잡도의 원인이었다.

## SDK 가 기준이다

API 는 이 저장소가 정한다. SDK 를 쓰는 저장소(whatslab-ros2 등)가 SDK 에 맞춘다.
특정 소비자 하나를 위해 SDK 에 호환 코드·옵션·별칭을 남기지 않는다. 깨지는 변경이면 커밋 제목에 `!` 를 붙이고
소비자 쪽을 고친다.

## 구조

```
core      ← receiver
core      ← retarget
receiver, retarget ← hand.py
```

- `core` 는 다른 하위 패키지를 import 하지 않는다.
- `receiver` 와 `retarget` 은 서로 모른다. 둘을 잇는 것은 `core.types` 의 `InputSample`·`HandPose` 뿐이다.
- 모든 입력은 OpenXR 26관절 `(26, 8)` 배열(`HandPose.xr_joints`, 손목 기준)로 들어온다. 새 입력 장치도 이
  형식으로 바꿔서 넣는다.
- 무거운 선택 의존성(pin, viser)은 그것을 쓰는 모듈에서만 import 한다.
  `whatslab.retarget` 의 GeoRT 경로는 numpy 만으로 import 돼야 한다.

### 리타게팅 엔진

`HandRetargeter` 가 창구이고, 엔진은 계산만 한다. 엔진의 약속은 `retarget/retargeter.py` 의 `Engine` Protocol 이다
(구조적 타이핑, 상속 없음). 엔진마다 이것을 만족하는지 테스트로 확인한다.

| 엔진이 할 일 | `HandRetargeter` 가 할 일 |
|---|---|
| `solve(xr_joints) -> q`, `joint_names`, `joint_limits`, `reset()` | 접두어(`dex:`, `geort:`)로 엔진 선택 |
| 이번 프레임을 풀 수 없으면 `solve` 가 `None` 을 돌려주고 이유는 `last_failure` 에 | 추적 끊김 시 `reset()`, `solve` 가 `None` 이면 직전 명령 유지 |

엔진마다 입력 전처리(OpenXR 26관절 → 그 엔진의 입력)는 `<엔진>/preprocess.py` 에 둔다. 새 엔진을 넣을 때
추적 끊김·직전 값 유지 로직을 엔진 안에 다시 만들지 않는다.

## 코드 규칙

- **docstring·주석을 쓰지 않는다.** 이름과 구조로 읽히게 쓰고, 설명이 필요하면 README·API.md 에 쓴다.
  pyproject·설정 YAML 도 마찬가지다. (`# noqa` 처럼 도구가 읽는 지시는 예외)
- **try/except 를 쓰지 않는다. 예외는 없다.** 예외를 잡아 넘기면 버그가 가려진다. 예상되는 실패는 예외가
  아니라 값이다: 입력에 따라 실패할 수 있는 함수는 `None` 을 돌려주고(호출한 쪽에 이유가 필요하면
  `last_failure` 속성에 문자열로), 무엇을 할지는 호출한 쪽이 정한다 — `HandRetargeter.compute` 는 엔진이
  `None` 을 돌려주면 직전 명령을 유지한다. `raise` 는 프로그래밍·설정 오류(잘못된 YAML, 없는 URDF, 잘못된
  `side`, 배열 모양 오류)에만 쓰고 아무도 잡지 않는다. 실패를 예외로 알리는 외부 라이브러리 호출은 상태를
  돌려주는 것으로 바꾼다(예: nlopt 대신 scipy `minimize`). 테스트는 `pytest.raises` 를 쓴다.
- 방어 코드를 넣지 않는다. 이미 검증된 경로에서 다시 검사하지 않고, 일어날 수 없는 경우를 처리하지 않는다.
- 한 번만 쓰는 래퍼·클래스·옵션을 만들지 않는다. `X = Y` 별칭과 `import … as …`(np 등 관례 제외)를 쓰지 않는다.
- 이름: 엔진은 `…Engine`, 수신기는 `…Receiver`, 엔진의 계산 메서드는 `solve`, 창구의 메서드는 `compute`.
  사람 손 좌우 인자는 `side`.
- PEP 8 을 ruff 로 검사한다(pycodestyle·pep8-naming·isort·pyflakes, 줄 길이 99). 공개 API 에는 타입 힌트를 단다.

## 개발

```bash
pip install -e '.[all,dev]'
ruff check .
pytest -q
```

CI(`.github/workflows/ci.yml`)가 PR 마다 ruff 와 테스트를 Python 3.10·3.12 에서 돌린다.

- 변경은 테스트와 함께 넣는다. 버그 수정은 그 버그를 재현하는 테스트부터 쓴다.
- 실기 확인: 글러브는 `examples/airglove_hand.py --viz`, Quest 맨손은 `WebXRHandReceiver` 를
  `HandModel(hand_source=…)` 에 넣어 헤드셋 브라우저로 연다.

## 커밋과 릴리스

- 커밋 제목: `종류(범위): 무엇을` — 종류는 `feat`, `fix`, `refactor`, `docs`, `chore`, `test`.
  API 를 깨면 `!` 를 붙인다(`refactor!: …`). 본문에는 왜 바꿨는지를 쓴다.
- CHANGELOG 는 두지 않는다. 변경 기록은 커밋과 PR 이 맡는다.
- 버전은 `pyproject.toml` 의 `version` 하나이고, 릴리스는 `vX.Y.Z` 태그다. 소비자는 태그로 고정한다.
- PyPI 에 올리지 않는다(`Private :: Do Not Upload` 분류자가 막는다).
- 문서를 같은 커밋에서 고친다. 공개 이름·시그니처가 바뀌면 API.md, README 두 언어판, QUICKSTART 를 함께 본다.
