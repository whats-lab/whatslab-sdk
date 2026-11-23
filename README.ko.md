<p align="center"><img src="banner.jpg" alt="WHATs LAB" width="100%"></p>

<h1 align="center">whatslab</h1>

<p align="center"><a href="README.md">English</a> | <b>한국어</b></p>

<p align="center">
  <a href="LICENSE"><img src="https://img.shields.io/badge/license-CC%20BY--NC--ND%204.0%20based-lightgrey.svg" alt="License"/></a>
  <img src="https://img.shields.io/badge/python-3.9%2B-blue.svg" alt="Python"/>
  <img src="https://img.shields.io/badge/Spine-3.0.0-green.svg" alt="Spine"/>
</p>

**whatslab** 은 주식회사 왓츠랩(WHATs LAB Corp)의 텔레오퍼레이션 코어입니다. 사람의 동작(VR 컨트롤러,
핸드트래킹, 데이터 글러브)을 로봇 팔·손의 관절각으로 바꾸는 순수 파이썬 SDK이며, 왓츠랩의 시뮬레이터
(MuJoCo, Isaac Sim)와 ROS 2 스택이 함께 쓰는 로직 계층입니다.

ROS 에 의존하지 않아 어디서든 같은 프로세스 안에서 동작합니다. 입력 수신, 캘리브레이션, 손·팔 리타게팅,
시각화, 데이터셋 기록 같은 *부품*을 제공하고, 이를 시뮬레이터나 로봇 파이프라인으로 엮는 *조립*은 사용하는
쪽에 맡깁니다. 모든 입력은 하나의 정준 좌표계(x 앞, z 위, 오른손 좌표계)로 맞춰지므로, 이후 코드에서 축을
다시 맞출 필요가 없습니다.

## 목차

- [주요 특징](#주요-특징)
- [설치](#설치)
- [호환성](#호환성)
- [빠른 시작](#빠른-시작)
- [예제와 도구](#예제와-도구)
- [문서](#문서)
- [감사의 말](#감사의-말)
- [라이선스](#라이선스)

## 주요 특징

**프레임워크에 묶이지 않음**
- 순수 파이썬이고 ROS 의존이 없어, MuJoCo·Isaac Sim·ROS 2 어디서든 같은 프로세스 안에서 씁니다
- 부품만 제공하고 파이프라인은 사용하는 쪽이 가집니다
- 의존 방향이 한쪽으로 정해져 있습니다 (`receiver → core`, `teleop → core·robot·solvers`)

**리타게팅 중심**
- 손: 입력은 OpenXR 26관절 손(`HandPose.xr_joints`) 하나이고(AirGlove 또는 Quest 핸드트래킹), 엔진은
  두 가지입니다
  - DexPilot 벡터 리타게팅([dex-retargeting](https://github.com/dexsuite/dex-retargeting)) — 손당 프레임
    1~2ms, URDF 만 있으면 어떤 로봇 손이든 됩니다
  - **GeoRT 소프트 프롬프트** 신경망 — numpy 로 한 번 순전파, 약 0.2ms, 학습된 모델 하나를 여러 로봇
    손이 함께 씁니다
- 팔: pinocchio 해석 야코비안 + 감쇠 최소제곱(DLS)
- 출력은 그대로 발행할 수 있는 `{side: {joint_name: rad}}` 형태입니다

**캘리브레이션 내장**
- 손목 yaw 정렬(머리 기준 스냅샷)
- 사용자별 팔 도달 범위(reach) 보정 — rig 설정에 저장해 다음 세션에 다시 씁니다

### 지원하는 손

로봇 설정의 `retarget:` 이 엔진과 손을 고릅니다.

**DexPilot** — `retarget: dex:<이름>`. `<이름>` 은 `configs/hands/` 의 파일입니다(`dex:orca` →
`orca_{side}`, 파일 경로도 됩니다). 한쪽 손 설정으로 반대쪽 손을 쓰면 입력을 거울상으로 바꿔 씁니다.

| 손 | `retarget:` | 설정 |
|---|---|---|
| ORCA Hand | `dex:orca` | `orca_{left,right}.yaml` (손목 관절은 빼고 0 으로 고정) |
| Tesollo DG-5F | `dex:dg5f` | `dg5f_{left,right}.yaml` (PIP·DIP 를 −5..90° 로 제한) |
| ROBOTIS HX5-D20 | `dex:robotis` | `robotis_{left,right}.yaml` |
| Allegro Hand | `dex:allegro` | `allegro_{left,right}.yaml` (손가락 4개 — 사람 소지는 쓰지 않음) |
| LEAP Hand | `dex:leap` | `leap_{left,right}.yaml` (손가락 4개 — 사람 소지는 쓰지 않음) |
| PSYONIC Ability Hand | `dex:ability` | `ability_{left,right}.yaml` (구동 6축, PIP 는 URDF mimic 을 따름) |
| Schunk SVH | `dex:schunk` | `schunk_{left,right}.yaml` (구동 9축, 연동 관절은 URDF mimic 을 따름) |

**GeoRT 소프트 프롬프트** — `retarget: geort:<이름>`. `<이름>` 은 `configs/geort/` 의 파일입니다
(`.yaml` 로 끝나는 경로도 됩니다). 학습된 모델은 패키지에 함께 들어 있습니다
(`solvers/hand/geort/assets/geort_prompt.npz`, 약 5MB). 설정의 `model:` 이나 `WHATSLAB_GEORT_MODEL` 로
다른 모델을 쓸 수 있습니다. 모델은 오른손용이고, 왼손은 좌우를 뒤집어 씁니다. rig 에서 쓰면 출력을 로봇
손 URDF 의 관절 범위로도 자릅니다.

| 손 | `retarget:` | `dexhand-description` 안의 URDF |
|---|---|---|
| ORCA Hand (오른손) | `geort:orca_right` | `orca_hand/urdf/right.urdf` |
| ROBOTIS HX5-D20 (오른손) | `geort:hx5_right` | `robotis_hx5_d20/urdf/hx5_d20_right.urdf` |

모델에는 Allegro·DG-5F-S 프롬프트도 들어 있지만, 아직 dexhand-description 에 없는 URDF 를 기준으로
학습했습니다. 학습(bake)은 robotPlatform(`tools/bake_geort_prompt.py`)에서 합니다.

## 설치

공개 소스(source-available) 라이선스로 배포합니다. PyPI 에는 없으므로 소스에서 설치합니다.

```bash
pip install '.[all]'      # receiver + hand + arm + viz + data
pip install '.[hand]'     # 필요한 것만: receiver / hand / arm / viz / data
pip install -e '.[all]'   # 개발용 editable
```

| extra | 설치되는 것 | 필요한 경우 |
|---|---|---|
| `receiver` | `airglove-client` (git 태그 v0.4.0) | AirGlove(Spine) 입력. WebXR 입력은 추가 설치가 필요 없습니다. |
| `hand` | `pin`, `dex-retargeting>=0.4.6`, `dexhand-description` | DexPilot 엔진 |
| `arm` | `pin`, `dexhand-description` | 팔 IK, rig, `whatslab.teleop` 프리셋 |
| `viz` | `viser`, `trimesh`, `pin` | 로봇 시각화, `tools/align_frames.py` |
| `data` | `pyarrow`, `imageio[ffmpeg]` | LeRobot v2.1 데이터셋 기록 |

numpy 는 상한이 없습니다. dex-retargeting 은 0.4.6 이 numpy 1 용 마지막 버전, 0.5.0 이 numpy 2 용 첫
버전이라 pip 가 환경에 맞는 쪽을 고릅니다. numpy 1 을 쓰는 환경(Isaac Sim 5.1)은 그쪽에서 `numpy<2` 와
`dex-retargeting<0.5` 를 고정합니다. dex-retargeting 은 모듈 최상단에서 `torch` 를 import 하고
`nlopt<2.8`·`pin>=2.7`·Python 3.13 미만이 필요합니다. GeoRT 추론(`whatslab.solvers.hand.geort`) 자체는
numpy 만 있으면 됩니다.

로봇·rig 설정은 패키지에 들어 있습니다. URDF 와 메쉬는 별도 패키지
[`dexhand-description`](https://github.com/whats-lab/dexterous-hand-urdf) 이 제공하며(`hand`/`arm`
extra 가 함께 설치), 다른 자산 경로를 쓰려면 `WHATSLAB_MODELS_ROOT` 를 지정합니다.

실물 하드웨어(Nero 팔 + ORCA 손)로 텔레옵하려면 패키지 의존성이 아닌 드라이버가 더 필요합니다.
`scripts/install_robot_deps.sh` 가 한 번에 설치합니다.

```bash
PY=$(which python) ./scripts/install_robot_deps.sh
python examples/quest_arm.py --rig rigs/nero_orca_right.yaml --viz --robot
```

`pyAgxArm`(Nero 팔, CAN)은 예제만 쓰므로 `examples/requirements-robot.txt` 에 있습니다. `orca_core`(ORCA 손)는
거기에도 넣을 수 없습니다. 모든 태그가 `numpy>=2.2.6` 을 선언해 numpy 1 환경과 충돌하고(실제 코드는 numpy
1.26 에서 동작합니다), wheel 에 손 설정(`models/<버전>/<모델>/config.yaml`)이 빠져 있기 때문입니다. 그래서
스크립트가 소스를 클론해 editable 로 설치합니다. 손 모델은 `orca_core` 기본값을 쓰고, 다른 모델은
`--hand-model` 로 지정합니다.

## 호환성

| 입력 | 필요한 것 |
|---|---|
| AirGlove | 같은 PC 에서 도는 **Spine 3.0.0**(OSC 프로토콜 3) |
| Quest 컨트롤러·핸드트래킹 | 헤드셋 브라우저(WebXR). 앱 설치도 Spine 도 필요 없습니다 |

**Spine 과 AirGlove.** `GloveModel`·`HandModel` 의 기본 글러브 소스인 `AirGloveReceiver` 는
[AirGlove-Client](https://github.com/whats-lab/AirGlove-Client)(`airglove-client` 0.4.0)를 감쌉니다. 이
클라이언트가 Spine 3.0.0 의 OSC 프로토콜 3(메시지 번호 헤더, `/pair/N` 주소)을 처리합니다. whatslab 0.5.0
부터 Spine 2.x 는 지원하지 않으며, Spine 2.x 에는 whatslab 0.4.x 를 쓰세요. Spine 은 PC 당 클라이언트 하나
(UDP 4040)에만 보내므로, Spine PC 에서 글러브를 받을 수 있는 프로세스는 하나뿐입니다. Spine 페어는
`AirGloveReceiver(pair=1)` 로 고릅니다. 손마다 다음을 줍니다.

- `HandPose.xr_joints` — `(26, 8)` 배열. OpenXR `XrHandJointEXT` 26관절(palm, wrist, 엄지 4, 검지~소지 각 5)의
  `[qx qy qz qw px py pz radius]` 를 받은 그대로 담습니다. 손목 기준(손목 = 항등)이고, OpenXR 관절 축(−Z 는
  뼈를 따라 손끝 방향, +Y 는 손등, 양손 같은 규칙)이며, 단위는 미터입니다.
- `HandPose.wrist` — 항상 `None` 입니다. Spine 3.0.0 은 world 손목 자세를 보내지 않고, 캘리브레이션되지 않은
  센서 프레임의 손등 IMU 원시 쿼터니언(Spine 이 `[y, x, z, -w]` 로 재배열)만 보냅니다. 이것을 팔 목표로 쓰면
  팔이 엉뚱하게 움직이므로 수신기는 손목을 내지 않고, `GloveModel` 은 팔 회전을 컨트롤러에서 가져옵니다(손마다
  경고 한 번). 원시값은 변환 없이 `AirGloveReceiver.raw_wrist(side)` 로 읽을 수 있습니다.

신선도(`stale_timeout`, 기본 0.2초)는 클라이언트가 프레임을 *받은* 시점부터 잽니다. Spine 3.0.0 은 시각과
순번을 보내지 않습니다(`seq`·`sender_time_us` 는 클라이언트가 세고 찍은 값입니다).
`AirGloveReceiver.frame(side)` 는 최신 프레임을 `seq`·수신 시각·`age_s` 와 함께 신선도 필터 없이 돌려주며,
새 프레임인지 가려야 하는 쪽에서 씁니다. `wrist_frame(side)`·`device_status()`·`alarms()`·`stats()` 는
클라이언트 값을 그대로 넘깁니다.

**손 리타게팅**은 `xr_joints` 만 읽습니다. `dex:<이름>` 이면 위치를 손목 기준으로 잡아 사람 손 21점(MANO
순서)으로 옮기고, 매 프레임 사람 손바닥 프레임(손목 + 검지·중지·소지 MCP)을 로봇의 q=0 손바닥 프레임에 맞춰
돌립니다. 그래서 손목 자세가 손가락으로 새지 않습니다. `geort:<이름>` 이면 같은 21점을 GeoRT 신경망으로
풉니다. 그 밖의 `retarget:` 값은 에러가 납니다.

## 빠른 시작

```python
from whatslab.teleop import GloveModel

m = GloveModel("rigs/nero_orca_right.yaml")   # 팔 = WebXR 컨트롤러 → IK, 손 = AirGlove → 리타게팅
m.start()

while True:
    q = m.get_q()             # {"right": {joint_name: rad, ...}}  — 팔 + 손을 합친 값
    publish_joint_states(q)   # 사용하는 쪽에서 sim/ROS 관절 순서로 재배열
```

프리셋은 `QuestModel`(Quest 맨손 핸드트래킹), `GloveModel`(컨트롤러 + 글러브), `HandModel`(손 단독) 세
가지입니다. 수신기 옵션은 dict 로 넘깁니다:
`GloveModel(rig, hand_kwargs={"glove_port": 4040}, arm_kwargs={"port": 8443})`,
`HandModel(hand_kwargs={...})`.

Quest 입력은 헤드셋 브라우저(WebXR + WebSocket)로만 받습니다. 설치할 앱은 없습니다. 컨트롤러
(`WebXRControllerReceiver`)와 맨손 핸드트래킹(`WebXRHandReceiver`, OpenXR 26관절이라 글러브와 같은
dex·geort 리타게팅을 씀)이 한 포트를 함께 씁니다. WebXR 경로는 무선이 기본입니다. WebXR 은 보안 컨텍스트가
필요하므로 이 PC 의 LAN IP 에 자체 서명 인증서로 HTTPS 를 엽니다. 유선 `adb reverse` 로 쓰려면 `tls=False` 를
넘깁니다. 자세한 내용은 `docs/API.md` 의 「WebXR 경로」에 있습니다.

다른 하드웨어 조합을 쓰려면 `TeleopModel` 을 상속해 추상 훅 `_get_raw_target()` 하나만 구현하면 됩니다. 이
훅은 어느 소스를 팔 엔드이펙터 목표로 쓸지 정합니다. 캘리브레이션, IK, 리타게팅, 안전 필터는 이미 연결되어
있습니다.

## 예제와 도구

```bash
python examples/quest_arm.py --rig rigs/nero_orca_right.yaml              # 컨트롤러 + 글러브
python examples/quest_arm.py --rig rigs/nero_orca_right.yaml --arm wrist  # Quest 맨손 핸드트래킹
python examples/airglove_hand.py --config geort:orca_right                # 글러브 → 로봇 손 단독
python examples/verify_rig.py --rig rigs/nero_orca_right.yaml             # rig 기구학 점검

python tools/align_frames.py robot --robot robots/nero.yaml               # 로봇을 정준 축에 정렬
```

팔 IK 의 정확도는 유효한 관절각에서 FK 로 만든 목표로만 판정하세요. 그러면 오차 하한이 정확히 0 이라 남는
오차는 모두 솔버 탓입니다. 직교 좌표로 합성한 궤적은 도달할 수 없는 자세를 지나므로 솔버 품질과 도달성이
섞입니다. (고정 기준선 벤치마크는 내부 도구라 이 저장소에 넣지 않았습니다.)

테스트는 `pip install -e '.[all,dev]' && pytest` 로 돌립니다.

## 문서

- [**실물 텔레옵 빠른 시작**](docs/QUICKSTART.md) — 클론부터 Nero 팔 + ORCA 손 텔레옵까지의 절차와 자주 걸리는 문제
- [**사용 가이드**](docs/GUIDE.md) — 새 로봇 올리기, 캘리브레이션, 팔 IK 튜닝과 변경 판정법, 진단, 실물 전송
- [**API 레퍼런스**](docs/API.md) — 서브패키지별 공개 심볼과 시그니처
- [**변경 이력**](CHANGELOG.md) — 버전별 변경 내역. **0.5.0 에 호환 없는 변경이 있습니다**
  (Spine 3.0.0 전용, `AirGloveReceiver`, `dex:`/`geort:` 리타게팅, Quest 는 WebXR 로만)

## 감사의 말

whatslab 은 다음 오픈소스 위에서 만들어졌습니다:
[Pinocchio](https://github.com/stack-of-tasks/pinocchio)(강체 기구학·IK),
[dex-retargeting](https://github.com/dexsuite/dex-retargeting)(DexPilot 손 리타게팅, MIT),
[GeoRT](https://zhaohengyin.github.io/geort)(기하 기반 손 리타게팅 — 소프트 프롬프트 모델 학습의 바탕),
[viser](https://github.com/nerfstudio-project/viser)(웹 3D 시각화),
[LeRobot](https://github.com/huggingface/lerobot)(데이터셋 형식).

## 라이선스

**주식회사 왓츠랩(WHATs LAB Corp) 소스 코드 라이선스**(CC BY-NC-ND 4.0 기반)를 따릅니다. source-available,
비상업, 2차 저작물 금지입니다. 자세한 내용은 [LICENSE](LICENSE) 를 참고하세요.

Copyright © 주식회사 왓츠랩(WHATs LAB Corp). All rights reserved.
