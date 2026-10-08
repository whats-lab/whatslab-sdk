<p align="center"><img src="banner.jpg" alt="WHATs LAB" width="100%"></p>

<h1 align="center">whatslab</h1>

<p align="center"><a href="README.md">English</a> | <b>한국어</b></p>

<p align="center">
  <a href="LICENSE"><img src="https://img.shields.io/badge/license-CC%20BY--NC--ND%204.0%20based-lightgrey.svg" alt="License"/></a>
  <img src="https://img.shields.io/badge/python-3.9%2B-blue.svg" alt="Python"/>
  <img src="https://img.shields.io/badge/numpy-%3C2-blue.svg" alt="numpy"/>
  <img src="https://img.shields.io/badge/tests-88%20passing-brightgreen.svg" alt="Tests"/>
</p>

**whatslab** 은 주식회사 왓츠랩(WHATs LAB Corp)의 텔레오퍼레이션 코어입니다. 사람의
동작(VR 컨트롤러·핸드트래킹·데이터 글러브)을 로봇 팔·손의 관절각으로 변환하는
순수 파이썬 SDK로, 왓츠랩 시뮬레이터(MuJoCo·Isaac Sim)와 ROS2 스택이 공통으로
사용하는 로직 계층입니다.

ROS 에 의존하지 않아 어디서든 in-process 로 동작합니다. 입력 수신·캘리브레이션·
손/팔 리타게팅·시각화·데이터 기록 같은 *부품*을 제공하고, 이를 시뮬레이터나 로봇의
파이프라인으로 엮는 *조립*은 이를 사용하는 쪽이 맡습니다. 모든 입력은 하나의 정준
좌표계(x=앞, z=위, 오른손 좌표계)로 정규화되므로, 하위 코드에서 축을 다시 맞출
필요가 없습니다.

## 목차

- [주요 특징](#주요-특징)
- [설치](#설치)
- [호환성](#호환성)
- [빠른 시작](#빠른-시작)
- [예제 & 도구](#예제--도구)
- [문서](#문서)
- [감사의 말](#감사의-말)
- [라이선스](#라이선스)

## 주요 특징

**프레임워크 비의존**
- 순수 파이썬으로 ROS 의존이 없어, MuJoCo·Isaac Sim·ROS2 어디서든 in-process 로 사용
- 부품만 제공하고 파이프라인은 사용하는 쪽이 소유
- 명확한 단방향 의존 구조 (`receiver → core`, `model → core·robot`)

**리타게팅 중심**
- 손: 글러브의 OpenXR 26관절 → DexPilot 벡터 리타게팅
  ([dex-retargeting](https://github.com/dexsuite/dex-retargeting) 0.4.6), 손당 프레임
  1~2ms. 레거시 사람 손 URDF 관절각 입력은 학습된 통합 ONNX 모델 한 번의 순전파
- 팔: pinocchio 해석 야코비안 + 감쇠 최소자승(DLS)
- 출력은 그대로 발행 가능한 `{side: {joint_name: rad}}` 형태

**캘리브레이션 내장**
- 손목 yaw 정렬(머리 기준 스냅샷)
- 사용자별 팔 도달 범위(reach) 보정 — rig config 에 저장되어 다음 세션에 재사용

### 지원하는 손

**현재 글러브(OpenXR `xr_joints`)** — rig 의 `retarget:` 에 `dex:<이름>` 을 넣는다.
`<이름>` 은 `configs/hands/` 의 파일이다(`dex:orca` → `orca_{side}`, 파일 경로도 된다).
한쪽 손 config 로 반대쪽을 쓰면 입력을 거울상으로 바꿔 쓴다.

| 손 | `retarget:` | config |
|---|---|---|
| ORCA Hand | `dex:orca` | `orca_{left,right}.yaml` (손목 관절은 빼고 0 고정) |
| Tesollo DG-5F | `dex:dg5f` | `dg5f_{left,right}.yaml` (PIP·DIP 를 −5..90° 로 자름) |
| ROBOTIS HX5-D20 | `dex:robotis` | `robotis_{left,right}.yaml` |
| Allegro Hand | `dex:allegro` | `allegro_{left,right}.yaml` (손가락 4개 — 사람 소지는 안 쓴다) |
| LEAP Hand | `dex:leap` | `leap_{left,right}.yaml` (손가락 4개 — 사람 소지는 안 쓴다) |
| PSYONIC Ability Hand | `dex:ability` | `ability_{left,right}.yaml` (구동 6축, PIP 는 URDF mimic) |
| Schunk SVH | `dex:schunk` | `schunk_{left,right}.yaml` (구동 9축, 연동 관절은 URDF mimic) |

**레거시 관절각 입력(Spine ≤ 2.3.1)** — 아래 손 전부와 좌우가 ONNX 그래프 하나를
공유한다. 첫 열의 이름이 `retarget:` 에 넣는 값이고, 괄호 안 별칭도 같이 인식한다.

| 손 | `retarget:` | `dexhand-description` 안의 URDF |
|---|---|---|
| 사람 기준 손 (리타게팅 입력) | `human` (`base_hand`) | `base_hand/urdf/{side}.urdf` |
| ORCA Hand | `orca` (`orca_hand`) | `orca_hand/urdf/{side}.urdf` |
| Allegro Hand | `allegro` (`allegro_hand`) | `allegro_hand/allegro_hand_{side}.urdf` |
| Tesollo DG-5F | `tesollo` (`tesollo_dg5f`) | `tesollo_dg5f/dg5f_{side}.urdf` |
| ROBOTIS HX5-D20 | `robotis` (`robotis_hx5_d20`) | `robotis_hx5_d20/urdf/hx5_d20_{side}.urdf` |

그 밖의 손은 생성 시점에 에러가 나고, 표에 있는 이름을 함께 알려준다. 새 손을 넣으려면
retarget_net 의 `tools/onboard_urdf.py` 로 표를 만든다 — URDF 가 센서 프레임 계약
(`{side}_sensor_dorsum` + 손가락마다 `_proximal`/`_distal`)을 지켜야 한다. 학습에 없던
손은 몇백 스텝 미세조정 후 그래프를 재수출해야 성능이 난다.

## 설치

공개 소스(source-available) 라이선스로 배포하며, PyPI 에는 올리지 않고 소스에서
설치합니다.

```bash
pip install '.[all]'      # receiver + hand + arm + viz + data
pip install '.[hand]'     # 부분 설치: hand / arm / receiver / viz / data
pip install -e '.[all]'   # 개발용 editable
```

실물 로봇(nero 팔 + ORCA 손)으로 텔레옵하려면 드라이버가 더 필요합니다.
`scripts/install_robot_deps.sh` 가 한 번에 처리합니다.

```bash
PY=$(which python) ./scripts/install_robot_deps.sh
python examples/quest_arm.py --rig rigs/nero_orca_right.yaml --viz --robot
```

`pyAgxArm`(CAN) 은 `robot` extra 로 들어가지만 `orca_core` 는 extra 에 넣을 수
없습니다 — 전 태그가 `numpy>=2.2.6` 을 선언하는데 이 저장소는 `numpy<2` 핀이라
의존성 해석이 실패합니다(실제 코드는 numpy 1.26 에서 동작). 또 `orca_core` 의
wheel 에는 손 설정(`models/<버전>/<모델>/config.yaml`)이 들어가지 않으므로,
스크립트는 소스를 클론해 editable 로 설치합니다. 손 모델은 `orca_core` 기본값을
쓰고, 다른 손을 쓰면 `--hand-model` 로 지정합니다.

`hand` extra 는 `dex-retargeting==0.4.6` 으로 고정합니다 — `numpy<2` 를 받는 마지막
버전입니다. 모듈 최상단에서 `torch` 를 import 하고 `nlopt<2.8`·`pin>=2.7`·
Python < 3.13 이 필요하며, `import whatslab.teleop` 이 약 0.9초 걸립니다(없을 때 0.25초).

robot/rig config 는 패키지에 함께 들어 있습니다. URDF·메쉬는 별도의 단일 소스 패키지
[`dexhand-description`](https://github.com/whats-lab/dexterous-hand-urdf) 이 제공하며,
`hand`/`arm` extra 가 의존성으로 가져옵니다. 자산 경로는 `WHATSLAB_MODELS_ROOT` 로
바꿀 수 있습니다.

## 호환성

데이터 글러브 텔레옵은 왓츠랩의 글러브 미들웨어 **Spine** 을 거칩니다. 컨트롤러·
핸드트래킹(Quest) 경로는 Spine 이 필요 없습니다.

**현재 Spine (OpenXR 손 출력)** — `GloveModel`·`HandModel` 의 기본 글러브 소스인
`GloveOpenXRReceiver` 가 [`airglove-client`](https://github.com/whats-lab/AirGlove-Client)
라이브러리(`receiver` extra 가 설치)를 감쌉니다. 클라이언트 API 는 그 저장소의
`python/README.md`, 전송 계약은 AirGlove-Client-Core `docs/spine_protocol.md` 가
정본입니다. 손마다 다음을 냅니다.

- `HandPose.xr_joints` — `(26, 8)` 배열. OpenXR `XrHandJointEXT` 26관절(palm, wrist,
  엄지 4, 검지~소지 각 5)의 `[qx qy qz qw px py pz radius]` 를 받은 그대로 담습니다:
  손목 공간(손목 = 단위), OpenXR 관절 축(−Z = 뼈 따라 손끝, +Y = 손등, 양손 같은 규칙),
  미터.
- `HandPose.wrist` — 글러브의 캘리브된 OpenXR 손목 자세(Spine 의 +Y 위 world, heading 은
  IMU 그대로라 드리프트)를 WebXR 경로와 같은 행렬로 정준 프레임에 옮긴 값입니다.
  옛 raw 손등 IMU 손목으로 맞춘 rig `ee_align` 값은 실기에서 다시 확인해야 합니다.

rig 의 `retarget:` 가 `dex:<이름>` 이면 손가락을 `xr_joints` 에서 리타게팅합니다
(`XRHandRetargeter`, DexPilot). 위치를 손목 기준으로 잡아 MANO 21 키포인트로 옮기고,
매 프레임 사람 손바닥 프레임(손목 + 검지·중지·소지 MCP)을 로봇의 q=0 손바닥 프레임에
맞춰 돌립니다 — 손목 자세가 손가락으로 새지 않습니다. `retarget:` 가 ONNX 이름
(`orca_hand` 등)이면 `joint_angles` 를 읽는데 현재 Spine 은 그것을 보내지 않으므로
손이 직전 명령을 유지합니다.

**레거시 (Spine 2.3.1 이하)** — `GloveHumanAnglesReceiver`(`/{side}/joint_angles/get`
+ `/{side}/wrist/get`), `GloveHumanHandReceiver`(`/{side}/quat/get`),
`GloveRobotHandReceiver` 는 옛 Spine `docs/OSC_Protocol.md` 를 따르며 현재 Spine 에서는
아무것도 받지 못합니다. `GloveModel(rig, hand_source="angles")` / `"robot"` 으로 고릅니다.

## 빠른 시작

```python
from whatslab.teleop import GloveModel

m = GloveModel("rigs/nero_orca_right.yaml")   # 팔 = 컨트롤러 IK, 손 = 글러브 리타게팅
m.start()

while True:
    q = m.get_q()             # {"right": {joint_name: rad, ...}}  — 팔 + 손 합침
    publish_joint_states(q)   # 사용하는 쪽 담당: sim/ROS 관절 순서로 재배열
```

프리셋: `QuestModel`(핸드트래킹) · `GloveModel`(컨트롤러 + 글러브) · `HandModel`(손 단독).
리시버 옵션은 dict 로 넘긴다 — `GloveModel(rig, hand_kwargs={"glove_port": 4040},
arm_kwargs={...})`, `HandModel(hand_kwargs={...})`.
`GloveModel` 의 팔 목표는 전송 경로를 고를 수 있다 — `arm_source="quest"`(OSC/UDP, 기본)
또는 `arm_source="webxr"`(브라우저 WebXR + WebSocket, APK 사이드로딩 불필요).
WebXR 경로는 **무선이 기본**이다 — WebXR 이 보안 컨텍스트를 요구하므로 이 머신의
LAN IP 에 자체 서명 HTTPS 를 연다. 유선 `adb reverse` 로 쓰려면 `tls=False`.
둘은 **병행 설치**되며 자세한 것은 `docs/API.md` 의 「WebXR 경로」에 있다.
직접 만든 하드웨어 조합은 `TeleopModel` 을 상속해 추상 훅 `_get_raw_target()`
**하나만** 구현하면 됩니다 — 어느 소스를 팔 EE 목표로 쓸지만 정하면 캘리브·IK·
리타게팅·안전필터 배선은 이미 되어 있습니다.

## 예제 & 도구

```bash
python examples/quest_arm.py --rig rigs/nero_orca_right.yaml              # 컨트롤러 + 글러브
python examples/quest_arm.py --rig rigs/nero_orca_right.yaml --arm wrist  # Quest 핸드트래킹
python examples/verify_rig.py --rig rigs/nero_orca_right.yaml             # rig 기구학 점검

python tools/align_frames.py robot --robot robots/nero.yaml              # 로봇을 정준 축에 정렬
```

팔 IK 의 **정확도는 유효 관절각 → FK 로 만든 목표**로만 판정한다 — 오차 하한이
정확히 0 이라 남는 오차가 전부 솔버 탓이다. 좌표계로 합성한 궤적은 도달 불가
구간을 지나 솔버 품질과 도달성을 섞는다. (고정 기준선 벤치는 내부 도구이고 이
저장소에 포함하지 않는다.)

테스트: `pip install -e '.[all,dev]' && pytest`

## 문서

- [**실물 텔레옵 빠른 시작**](docs/QUICKSTART.md) — 새 PC 에서 클론부터 nero 팔 +
  ORCA 손 텔레옵까지의 절차와, 자주 걸리는 것들
- [**사용 가이드**](docs/GUIDE.md) — 새 로봇 올리기, 캘리브레이션, 팔 IK 튜닝과
  변경 판정법, 진단, 실물 전송
- [**API 레퍼런스**](docs/API.md) — 서브패키지별 공개 심볼과 시그니처
- [**변경 이력**](CHANGELOG.md) — **0.2.0 에 호환 없는 변경이 있습니다**
  (`model.ik[s]` → `model.sides[s].ik`, `RobotModel.solve` 제거)

## 감사의 말

whatslab 은 다음 오픈소스 위에서 만들어졌습니다:
[Pinocchio](https://github.com/stack-of-tasks/pinocchio)(강체 기구학/IK),
[dex-retargeting](https://github.com/dexsuite/dex-retargeting)(DexPilot 손 리타게팅, MIT),
[viser](https://github.com/nerfstudio-project/viser)(웹 3D 시각화),
[LeRobot](https://github.com/huggingface/lerobot)(데이터셋 포맷).

## 라이선스

**주식회사 왓츠랩(WHATs LAB Corp) 소스 코드 라이선스**(CC BY-NC-ND 4.0 기반)를 따릅니다
— source-available, 비영리, 2차 저작물 금지. 자세한 내용은 [LICENSE](LICENSE) 를 참고하세요.

Copyright © 주식회사 왓츠랩(WHATs LAB Corp). All rights reserved.
