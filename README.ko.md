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
- 손: dex-retargeting 2단계(vector + position) IK, 결정적 종료
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
버전은 **2.3.1 미만**입니다(2.3.1 이상은 아직 지원하지 않습니다). 컨트롤러·
핸드트래킹(Quest) 경로는 Spine 이 필요 없습니다.

## 빠른 시작

```python
from whatslab.model import GloveModel

m = GloveModel("rigs/nero_orca_right.yaml")   # 팔 = 컨트롤러 IK, 손 = 글러브 리타게팅
m.start()

while True:
    q = m.get_q()             # {"right": {joint_name: rad, ...}}  — 팔 + 손 합침
    publish_joint_states(q)   # 사용하는 쪽 담당: sim/ROS 관절 순서로 재배열
```

프리셋: `QuestModel`(핸드트래킹) · `GloveModel`(컨트롤러 + 글러브) · `HandModel`(손 단독).
직접 만든 하드웨어 조합은 `TeleopModel` 을 상속해 `get_data()` 만 재정의하면 됩니다.

## 예제 & 도구

```bash
python examples/quest_arm.py --rig rigs/nero_orca_right.yaml              # 컨트롤러 + 글러브
python examples/quest_arm.py --rig rigs/nero_orca_right.yaml --arm wrist  # Quest 핸드트래킹
python examples/verify_rig.py --rig rigs/nero_orca_right.yaml             # rig 기구학 점검

python tools/align_frames.py robot --robot robots/nero.yaml              # 로봇을 정준 축에 정렬
```

테스트: `pip install -e '.[all,dev]' && pytest`

## 문서

- [**API 레퍼런스**](docs/API.md) — 서브패키지별 공개 심볼과 시그니처

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
