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
- 손: 글러브가 보낸 사람 손 URDF 관절각 → pinocchio FK → 리타게팅 IK.
  `backend="dex"`(dex-retargeting 2단계 vector+position) 또는 `backend="kp"`
  (키포인트 결합 목적함수 = 팜상대 지문 + 형상 + DexPilot 스타일 핀치 스냅, pin+numpy 만)
- 팔: pinocchio 해석 야코비안 + 감쇠 최소자승(DLS)
- 출력은 그대로 발행 가능한 `{side: {joint_name: rad}}` 형태

**캘리브레이션 내장**
- 손목 yaw 정렬(머리 기준 스냅샷)
- 사용자별 팔 도달 범위(reach) 보정 — rig config 에 저장되어 다음 세션에 재사용

## 설치

공개 소스(source-available) 라이선스로 배포하며, PyPI 에는 올리지 않고 소스에서
설치합니다.

```bash
pip install '.[all]'      # receiver + hand + arm + viz
pip install '.[hand]'     # 부분 설치: hand / arm / receiver / viz / data
pip install -e '.[all]'   # 개발용 editable
```

robot/rig config 는 패키지에 함께 들어 있습니다. URDF·메쉬는 별도의 단일 소스 패키지
[`dexhand-description`](https://github.com/whats-lab/dexterous-hand-urdf) 이 제공하며,
`hand`/`arm` extra 가 의존성으로 가져옵니다. 자산 경로는 `WHATSLAB_MODELS_ROOT` 로
바꿀 수 있습니다.

## 호환성

데이터 글러브 텔레옵은 왓츠랩의 글러브 미들웨어 **Spine** 을 거칩니다. 지원하는 Spine
버전은 **2.3.1 이하**입니다(그보다 새 버전은 아직 지원하지 않습니다). 컨트롤러·
핸드트래킹(Quest) 경로는 Spine 이 필요 없습니다.

글러브 경로는 Spine `docs/OSC_Protocol.md` 의 OSC 계약을 그대로 따릅니다 —
`GloveHumanAnglesReceiver` 와 `GloveRobotHandReceiver` 가
`/{side}/joint_angles/get`((이름, 각도) 쌍) + `/{side}/wrist/get` 을 받습니다.
모든 Spine 메시지는 `args[0]` 에 messageType 헤더를 싣습니다. `wrist` 는 변환 없이
Spine 프레임 그대로 전달됩니다.

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
직접 만든 하드웨어 조합은 `TeleopModel` 을 상속해 추상 훅 `_get_raw_target()`
**하나만** 구현하면 됩니다 — 어느 소스를 팔 EE 목표로 쓸지만 정하면 캘리브·IK·
리타게팅·안전필터 배선은 이미 되어 있습니다.

## 예제 & 도구

```bash
python examples/quest_arm.py --rig rigs/nero_orca_right.yaml              # 컨트롤러 + 글러브
python examples/quest_arm.py --rig rigs/nero_orca_right.yaml --arm wrist  # Quest 핸드트래킹
python examples/verify_rig.py --rig rigs/nero_orca_right.yaml             # rig 기구학 점검

python tools/align_frames.py robot --robot robots/nero.yaml              # 로봇을 정준 축에 정렬
python tools/bench_arm_ik.py --traj fk                                   # 팔 IK: 정확도/연속성/비용
python tools/bench_hand_retarget.py --dump … --profiles … --traj flex    # 손 리타게팅: 형상/GMC/LMC
python tools/train_hand_net.py --config orca_hand --side left …          # net 백엔드 학습
python tools/check_mirror.py                                             # 좌우 미러 게이트
python tools/export_hand_net_onnx.py --checkpoint … --out hand.onnx      # ONNX: 추론에 torch·pinocchio 불필요
```

`bench_hand_retarget.py` 는 손 리타게팅을 손볼 때의 고정 기준선이다. `pinch` 만 보지 말고
**`--traj flex`(굽힘 전용)로도 판정**한다 — 핀치 램프는 단조라서 순수 굽힘에서 드러나는
좌우 흔들림을 가린다. 주 지표는 `LMC`(손가락별 로컬 프레임)이고 `GMC`(공유 프레임)를 병기하며,
**두 값의 격차가 정렬 품질의 진단값**이다. 측정 전 `__pycache__` 를 비운다.

#### net 백엔드 기준선

손 리타게팅 `net` 백엔드의 기준선은 **`c2c7967`** 이다 — `prox` 를 proximal 센서
프레임에서 읽는 첫 커밋이고, 아래 숫자를 낸 모델들이 이 코드로 학습됐다. 손실은
`motion 1 / coverage 5 / bone 20 / pinch 1 / pos 20`(코드 상수), 1000 epoch, fp32,
`--random-mode mix` + 실측 q. 왼손 kp 대비:

| | 궤적 | 형상° | 말단° | 펴짐접촉 | GMC | LMC | \|dq\|p95 | ms |
|---|---|---|---|---|---|---|---|---|
| robotis kp | flex | 33.9 | 38.9 | **4.6** | **95.4** | 78.4 | **0.115** | 1.93 |
| robotis net | flex | **27.5** | **23.6** | 11.7 | 88.5 | **85.7** | 0.380 | **0.28** |
| robotis kp | abd | **7.5** | 8.3 | **7.2** | 99.7 | 99.2 | **0.032** | 1.93 |
| robotis net | abd | 11.0 | 8.6 | 11.0 | 99.7 | 99.1 | 0.037 | **0.27** |
| orca kp | flex | **46.7** | **41.8** | **1.6** | **96.6** | 77.1 | **0.111** | 2.20 |
| orca net | flex | 55.3 | 48.9 | 16.2 | 83.7 | **82.2** | 0.181 | **0.27** |
| orca kp | abd | **11.0** | 11.8 | **11.9** | 88.2 | 88.3 | **0.036** | 2.19 |
| orca net | abd | 12.9 | **11.7** | 11.1 | **91.6** | **92.2** | 0.150 | **0.27** |

net 이 이기는 것: **LMC 네 경우 모두**, robotis 굽힘 형상·말단(27.5/23.6 vs 33.9/38.9),
추론 7~8배. net 이 지는 것: **펴짐접촉 11~16mm vs kp 1.6~7.2**(손이 끝까지 안 펴진다),
**`|dq|p95` 최대 3배**(덜컹거린다), orca 굽힘 형상·말단. 이 표와 대조한다.

**LMC 정의가 바뀌었다.** 옛 정의는 로컬 프레임의 롤을 팜 y 축에 못박아, 로봇 손가락이
중립에서 벌어져 있으면 맞는 움직임인데 점수가 깎였다(orca 벌림 41.0 이 그것이고
고정 롤차 50도 였다). 지금은 뼈축을 최소회전으로 맞춘 뒤 방향을 비교한다 — 프레임
무관량이다. `c2c7967` 이전 기록의 LMC 는 이 표와 비교할 수 없다.

좌우 처리는 손마다 다르다 — 판정 기준은 그 손의 **좌우 URDF 미러 정확도**다.
orca 는 좌우가 정확한 미러(0.86mm, 관절 한계 16/16 동일)라 왼손 모델 하나로
`hand_solver.mirror_to` 로 양손을 쓴다(오른손 지문추종 24.49 vs 전용 24.02mm).
robotis 는 `finger_joint1~4` 가 좌우 규약 불일치라 미러가 63% 악화되므로
(49.65 vs 30.50mm) **side 별 체크포인트가 필수**다. 체크포인트에 학습 side 가
기록되고 로드 시 대조하므로 좌우를 뒤바꿔 넣으면 에러가 난다.

`bench_arm_ik.py` 는 팔 IK 를 손볼 때의 고정 기준선이다. **정확도 판정은 `--traj fk`**
(유효 관절각 → FK 로 목표를 만들어 오차 하한이 정확히 0)로 한다 — 좌표계로 합성한
궤적은 도달 불가 구간을 지나 솔버 품질과 도달성을 섞는다. `--floor` 는 최악 프레임의
도달 가능 하한을 구해 "솔버 실패 / 도달 불가"를 판정한다.

테스트: `pip install -e '.[all,dev]' && pytest`

## 문서

- [**사용 가이드**](docs/GUIDE.md) — 새 로봇 올리기, 캘리브레이션, 팔 IK 튜닝과
  변경 판정법, 진단, 실물 전송
- [**API 레퍼런스**](docs/API.md) — 서브패키지별 공개 심볼과 시그니처
- [**변경 이력**](CHANGELOG.md) — **0.2.0 에 호환 없는 변경이 있습니다**
  (`model.ik[s]` → `model.sides[s].ik`, `RobotModel.solve` 제거)

## 감사의 말

whatslab 은 다음 오픈소스 위에서 만들어졌습니다:
[Pinocchio](https://github.com/stack-of-tasks/pinocchio)(강체 기구학/IK),
[dex-retargeting](https://github.com/dexsuite/dex-retargeting)(손 리타게팅),
[viser](https://github.com/nerfstudio-project/viser)(웹 3D 시각화),
[LeRobot](https://github.com/huggingface/lerobot)(데이터셋 포맷).

## 라이선스

**주식회사 왓츠랩(WHATs LAB Corp) 소스 코드 라이선스**(CC BY-NC-ND 4.0 기반)를 따릅니다
— source-available, 비영리, 2차 저작물 금지. 자세한 내용은 [LICENSE](LICENSE) 를 참고하세요.

Copyright © 주식회사 왓츠랩(WHATs LAB Corp). All rights reserved.
