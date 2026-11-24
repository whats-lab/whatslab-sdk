<p align="center"><img src="banner.jpg" alt="WHATs LAB" width="100%"></p>

<h1 align="center">whatslab</h1>

<p align="center"><a href="README.md">English</a> | <b>한국어</b></p>

<p align="center">
  <a href="LICENSE"><img src="https://img.shields.io/badge/license-CC%20BY--NC--ND%204.0%20based-lightgrey.svg" alt="License"/></a>
  <img src="https://img.shields.io/badge/python-3.10%2B-blue.svg" alt="Python"/>
  <img src="https://img.shields.io/badge/Spine-3.0.0-green.svg" alt="Spine"/>
</p>

**whatslab** 은 주식회사 왓츠랩(WHATs LAB Corp)의 손 SDK 입니다. 사람 손(데이터 글러브 AirGlove, 또는
Quest 핸드트래킹)을 로봇 손 관절각으로 바꾸는 순수 Python 라이브러리입니다.

ROS 에 의존하지 않아 MuJoCo, Isaac Sim, ROS 2 어디서든 프로세스 안에서 씁니다. 범위는 손입니다 — 입력 수신,
손 리타게팅, 관절 안전 필터, 시각화. **팔 IK, 손목 추적, 팔 제어는 들어 있지 않습니다.**
SDK 는 손목 기준 손 관절을 주고, 손목 자세는 사용하는 쪽이 별도 트래커(Vive, Quest 컨트롤러, 모션캡처 등)에서
받아 팔 제어기에 넘깁니다.

## 목차

- [주요 기능](#주요-기능)
- [설치](#설치)
- [호환성](#호환성)
- [빠른 시작](#빠른-시작)
- [패키지 구성](#패키지-구성)
- [예제와 도구](#예제와-도구)
- [문서](#문서)
- [감사의 말](#감사의-말)
- [라이선스](#라이선스)

## 주요 기능

- 모든 입력이 같은 손 형식: OpenXR 26관절(`HandPose.xr_joints`). AirGlove(Spine 경유)와 Quest
  핸드트래킹(헤드셋 브라우저 WebXR) 모두 이 형식으로 들어옵니다.
- 리타게팅 엔진 두 가지
  - DexPilot 벡터 리타게팅([dex-retargeting](https://github.com/dexsuite/dex-retargeting) 의 numpy 이식,
    torch 없음, 손목→손끝 벡터 항을 섞을 수 있음) — 손당 프레임 1~2ms, URDF 가 있는 로봇 손이면 어느 것이든
  - **GeoRT 소프트 프롬프트** 신경망 — numpy 순전파 한 번(약 0.2ms), 학습 모델 하나를 여러 로봇 손이 공유
- 출력은 `{side: {관절 이름: rad}}` 로 바로 발행할 수 있고, 선택으로 `SafetyFilter` 가 위치·속도를 제한합니다.
- 순수 Python, ROS 의존 없음. 의존 방향이 한쪽으로 정해져 있습니다
  (`receiver → core`, `retarget → core`, `hand → core·receiver·retarget`).

### 지원하는 손

**DexPilot** — `dex:<이름>`. `<이름>` 은 `configs/hands/` 의 파일입니다(`dex:orca` 는 `orca_{side}`, 파일
경로도 됩니다). 한쪽 손 설정으로 반대쪽 손도 입력을 거울상으로 바꿔 씁니다.

| 손 | 엔진 이름 | 설정 |
|---|---|---|
| ORCA Hand | `dex:orca` | `orca_{left,right}.yaml` (손목 관절 제외, 0 고정) |
| Tesollo DG-5F | `dex:dg5f` | `dg5f_{left,right}.yaml` (PIP·DIP −5..90° 제한) |
| ROBOTIS HX5-D20 | `dex:robotis` | `robotis_{left,right}.yaml` |
| Allegro Hand | `dex:allegro` | `allegro_{left,right}.yaml` (손가락 4개, 사람 새끼손가락 미사용) |
| LEAP Hand | `dex:leap` | `leap_{left,right}.yaml` (손가락 4개, 사람 새끼손가락 미사용) |
| PSYONIC Ability Hand | `dex:ability` | `ability_{left,right}.yaml` (구동 6축, PIP 는 URDF mimic) |
| Schunk SVH | `dex:schunk` | `schunk_{left,right}.yaml` (구동 9축, 연동 관절은 URDF mimic) |

**GeoRT 소프트 프롬프트** — `geort:<이름>`. `<이름>` 은 `configs/geort/` 의 파일입니다(`.yaml` 로 끝나는
경로도 됩니다). 학습된 모델이 패키지에 들어 있고(`retarget/geort/assets/geort_prompt.npz`, 약 5MB),
모델 파일은 설정의 `model:` → `$WHATSLAB_GEORT_MODEL` → 동봉 파일 순서로 찾습니다. 모델은 오른손용이고
왼손은 거울상으로 씁니다.

두 설정 형식의 키 설명은 [API.md](docs/API.md) 에 있습니다.

| 손 | 엔진 이름 | `dexhand-description` URDF |
|---|---|---|
| ORCA Hand (오른손) | `geort:orca_right` | `orca_hand/urdf/right.urdf` |
| ROBOTIS HX5-D20 (오른손) | `geort:hx5_right` | `robotis_hx5_d20/urdf/hx5_d20_right.urdf` |

모델에는 Allegro·DG-5F-S 프롬프트도 들어 있지만, 아직 dexhand-description 에 없는 URDF 를 기준으로
학습했습니다. 학습 도구는 이 SDK 에 포함하지 않습니다.

## 설치

source-available 라이선스로 공개합니다. PyPI 에는 없고 소스에서 설치합니다. `Private :: Do Not Upload`
분류자가 있어 실수로 PyPI 에 올려도 거부됩니다.

```bash
pip install '.[all]'      # receiver + hand + viz + webxr
pip install '.[receiver]' # 또는 일부만: receiver / hand / viz / webxr
pip install -e '.[all]'   # 개발용 editable
```

| extra | 설치되는 것 | 용도 |
|---|---|---|
| `receiver` | `airglove-client`(git 태그 v1.0.0) | AirGlove(Spine) 입력 |
| `webxr` | `websockets` | Quest 핸드트래킹(헤드셋 브라우저) 입력 |
| `hand` | `pin`, `dexhand-description` | DexPilot 엔진 |
| `viz` | `viser`, `trimesh`, `pin` | 로봇 손 시각화 |

GeoRT 추론(`geort:`)은 numpy 만 있으면 됩니다. DexPilot 엔진은 dex-retargeting 최적화기를 numpy 로 옮긴
자체 구현이라 torch·dex-retargeting 이 필요 없고 `pin>=2.7` 만 더 씁니다(Python 3.14 까지 휠이 있습니다).
SLSQP 는 기본 의존성인 scipy 의 것을 씁니다. numpy 상한은 두지 않습니다. URDF·메쉬는 별도 패키지
[`dexhand-description`](https://github.com/whats-lab/dexterous-hand-urdf)(`hand` extra 가 설치)에서 오고,
다른 자산 폴더를 쓰려면 `WHATSLAB_MODELS_ROOT` 를 지정합니다.

## 호환성

| 입력 | 필요한 것 |
|---|---|
| AirGlove | 같은 PC 에서 도는 **Spine 3.0.0**(OSC 프로토콜 3) |
| Quest 핸드트래킹 | 헤드셋 브라우저(WebXR). 앱 설치도 Spine 도 필요 없습니다 |

`AirGloveReceiver` 는 [AirGlove-Client](https://github.com/whats-lab/AirGlove-Client)(`airglove-client`
1.0.0)를 감쌉니다. whatslab 1.0.0 부터 Spine 2.x 는 지원하지 않으며, Spine 2.x 에는 whatslab 0.4.x 를 쓰세요.
Spine 은 PC 당 클라이언트 하나(UDP 4040)에만 보내므로 Spine PC 에서 글러브를 받을 수 있는 프로세스는
하나뿐입니다. 손마다 `HandPose.xr_joints`(OpenXR 26관절, 손목 기준 `(26, 8)` 배열)를 줍니다.
손목 자세는 주지 않습니다 — Spine 이 보내는 손목 값은 손목 자세가 아니라 캘리브레이션되지 않은 IMU 쿼터니언입니다. 필드 세부는 [API.md](docs/API.md) 「OpenXR 글러브」에 있습니다.

글러브 원시 데이터만 필요하거나, 다른 언어(C/C++, C#, Unity, Unreal)를 쓰거나, 리타게팅을 직접 하려면
[AirGlove-Client](https://github.com/whats-lab/AirGlove-Client) 를 바로 쓰세요. ROS 2 는
[whatslab-ros2](https://github.com/whats-lab/whatslab-ros2) 를 보세요.

녹화는 사용하는 쪽 파이프라인에 맡깁니다. SDK 는 단순한 데이터(OpenXR 26관절 손, `{관절 이름: rad}`, 시각)를
주므로 LeRobot, HDF5, 자체 포맷 어디에든 붙습니다. ROS 2 에서는 whatslab-ros2 토픽을 rosbag2 로 녹화하면 됩니다.

## 빠른 시작

```python
from whatslab.hand import HandModel

m = HandModel("geort:orca_right", side="right")   # AirGlove → ORCA 오른손
m.start()

while True:
    q = m.get_q()             # {"right": {joint_name: rad, ...}} — 손이 안 잡히면 {}
    publish_joint_states(q)   # 사용하는 쪽에서 sim/ROS 관절 순서로 재배열
```

글러브 대신 Quest 핸드트래킹:

```python
from whatslab.receiver import WebXRHandReceiver
m = HandModel("dex:orca", hand_source=WebXRHandReceiver())   # 출력되는 https 주소를 헤드셋에서 연다
```

직접 만든 손 입력: `InputSample` 을 만들어 `HandRetargeter` 를 바로 부릅니다.

```python
from whatslab.core.types import HandPose, InputSample
from whatslab.retarget import HandRetargeter

r = HandRetargeter("geort:orca_right", side="right")
cmd = r.compute(InputSample(hand=HandPose(xr_joints=xr, tracked=True)))   # xr: (26, 8) OpenXR 관절
cmd.joint_names, cmd.joint_angles
```

`HandRetargeter` 는 접두어로 엔진을 고르고(`dex:` → `DexEngine`, `geort:` → `GeortEngine`), 추적이 끊기면
엔진을 초기화하고, 풀 수 없는 입력이 오면 직전 명령을 유지합니다. 계산만 필요하면 두 엔진의
`solve(xr_joints)` 를 바로 써도 됩니다.

## 패키지 구성

```
whatslab/
├── core/             타입(HandPose, InputSample, HandCommand), Receiver 인터페이스, OpenXR 관절
├── receiver/
│   ├── glove/        AirGloveReceiver (airglove-client, 연결 상태·알람)
│   └── webxr/        WebXRHandReceiver, websockets 서버, TLS 인증서, 헤드셋 페이지
├── retarget/
│   ├── retargeter.py HandRetargeter, Engine Protocol
│   ├── dex/          engine.py DexEngine (DexPilot) · preprocess.py OpenXR → MANO, 손바닥 좌표계, 거울상
│   └── geort/        engine.py GeortEngine · preprocess.py Kabsch 입력 좌표계 · prompt.py 모델 · 동봉 npz
├── hand.py           HandModel: 수신 → 리타게팅 → (선택) SafetyFilter
├── safety.py         SafetyFilter
├── viz/              HandViz, URDFScene (viser)
├── paths.py          configs·models 경로
└── configs/          hands/ (DexPilot, 14개), geort/ (2개)
```

## 예제와 도구

```bash
python examples/airglove_hand.py --config geort:orca_right               # 글러브 → 로봇 손, q 출력
python examples/airglove_hand.py --config dex:orca --viz                 # viser 로 로봇 손 보기
python tools/bench_hand_retarget.py --help                               # 리타게팅 품질 지표
```

테스트는 `pip install -e '.[all,dev]' && pytest`, 린트는 `ruff check .` 로 돌립니다(ruff 로 PEP 8 검사:
pycodestyle·pep8-naming·isort·pyflakes, 줄 길이 99).

## 문서

- [**빠른 시작**](docs/QUICKSTART.md) — 클론부터 로봇 손이 글러브를 따라오기까지, 자주 걸리는 것
- [**API 레퍼런스**](docs/API.md) — 하위 패키지별 공개 심볼과 시그니처
- [**기여 가이드**](CONTRIBUTING.ko.md) — 범위, 설계·코드 규칙

## 감사의 말

whatslab 은 훌륭한 오픈소스 위에 만들었습니다:
[Pinocchio](https://github.com/stack-of-tasks/pinocchio)(기구학),
[dex-retargeting](https://github.com/dexsuite/dex-retargeting)(DexPilot 손 리타게팅, MIT — DexPilot 엔진의
최적화기는 이것을 numpy 로 옮긴 것이다. [THIRD_PARTY_LICENSE](src/whatslab/retarget/dex/THIRD_PARTY_LICENSE) 참고),
[GeoRT](https://zhaohengyin.github.io/geort)(기하 기반 손 리타게팅, 소프트 프롬프트 모델 학습의 바탕),
[viser](https://github.com/nerfstudio-project/viser)(웹 3D 시각화).

## 라이선스

**주식회사 왓츠랩(WHATs LAB Corp) 소스 코드 라이선스**(CC BY-NC-ND 4.0 기반)를 따릅니다
— source-available, 비영리, 2차 저작물 금지. 자세한 내용은 [LICENSE](LICENSE) 를 참고하세요.

Copyright © 주식회사 왓츠랩(WHATs LAB Corp). All rights reserved.
