<p align="center"><img src="banner.jpg" alt="WHATs LAB" width="100%"></p>

<h1 align="center">whatslab</h1>

<p align="center"><a href="README.md">English</a> | <b>한국어</b></p>

<p align="center">
  <a href="LICENSE"><img src="https://img.shields.io/badge/license-CC%20BY--NC--ND%204.0%20based-lightgrey.svg" alt="License"/></a>
  <img src="https://img.shields.io/badge/python-3.9%2B-blue.svg" alt="Python"/>
  <img src="https://img.shields.io/badge/numpy-%3C2-blue.svg" alt="numpy"/>
  <img src="https://img.shields.io/badge/tests-88%20passing-brightgreen.svg" alt="Tests"/>
</p>

**whatslab** 은 주식회사 왓츠랩(WHATs LAB Corp)의 텔레오퍼레이션 코어 — 사람 동작(VR
컨트롤러·핸드트래킹·데이터 글러브)을 로봇 팔·손 관절각으로 바꾸는 순수 파이썬
SDK다. 왓츠랩의 시뮬레이터(MuJoCo, Isaac Sim)와 ROS2 스택 아래에 깔리는 공통
로직 계층으로 개발된다.

whatslab 은 ROS 에 의존하지 않으며 어디서든 in-process 로 동작한다. *부품*(입력 수신·
캘리브레이션·손/팔 리타게팅·시각화·데이터 기록)을 제공하고, 이 부품을 시뮬레이터나
로봇의 파이프라인으로 엮는 *조립*은 소비자가 맡는다. 입력은 단일 정준 좌표계
(x=앞, z=위, 오른손계)로 정규화되므로 다운스트림 코드는 축을 다시 매핑하지 않는다.

## 목차

- [주요 특징](#주요-특징)
- [설치](#설치)
- [빠른 시작](#빠른-시작)
- [예제 & 도구](#예제--도구)
- [문서](#문서)
- [감사의 말](#감사의-말)
- [라이선스](#라이선스)

## 주요 특징

**프레임워크 비의존:**
- 순수 파이썬, ROS 의존 0 — MuJoCo·Isaac Sim·ROS2 에서 in-process 로 사용
- 조립 가능한 부품 제공; 파이프라인은 소비자가 소유
- 엄격한 의존 방향 (`receiver → core`, `model → core·robot`)

**pip 한 방:**
- 손·팔·리시버·viz 가 pip `pinocchio`(double `pin`) 하나를 공유 — conda-forge 불필요
- 팔 IK 는 casadi/IPOPT 대신 해석 야코비안 + DLS 사용
- 스택 일관성을 위해 `numpy<2` 고정 (dex-retargeting, Isaac Sim 5.1)

**리타게팅 중심:**
- 손: dex-retargeting 2단계(vector + position) IK, 결정적 종료
- 팔: pinocchio 해석 야코비안 + 감쇠 최소자승(DLS)
- 출력은 발행 가능한 `{side: {joint_name: rad}}`

**캘리브레이션:**
- 손목 yaw 정렬(머리-상대 스냅샷)
- 사용자별 팔 도달반경(reach) 스케일 — rig config 에 저장

## 설치

공개 소스(source-available) 라이선스로 배포한다 — PyPI 엔 올리지 않고 소스에서 설치한다.

```bash
pip install '.[all]'      # receiver + hand + arm + viz
pip install '.[hand]'     # 부분 설치: hand / arm / receiver / viz / data
pip install -e '.[all]'   # 개발용 editable
```

robot/rig config 는 패키지에 동봉된다. URDF·메쉬는 별도 단일 소스 패키지
[`dexhand-description`](https://github.com/whats-lab/dexterous-hand-urdf) 가 제공하며
`hand`/`arm` extra 가 의존으로 끌어온다. 자산 트리는 `WHATSLAB_MODELS_ROOT` 로 override.

## 빠른 시작

```python
from whatslab.model import GloveModel

m = GloveModel("rigs/nero_orca_right.yaml")   # 팔 = 컨트롤러 IK, 손 = 글러브 리타게팅
m.start()

while True:
    q = m.get_q()             # {"right": {joint_name: rad, ...}}  — 팔 + 손 합침
    publish_joint_states(q)   # 소비자 몫: sim/ROS 관절 순서로 재배열
```

프리셋: `QuestModel`(핸드트래킹) · `GloveModel`(컨트롤러 + 글러브) · `HandModel`(손 단독).
커스텀 하드웨어 조합은 `TeleopModel` 을 상속해 `get_data()` 를 오버라이드한다.

## 예제 & 도구

```bash
python examples/quest_arm.py --rig rigs/nero_orca_right.yaml            # 컨트롤러 + 글러브
python examples/quest_arm.py --rig rigs/nero_orca_right.yaml --arm wrist  # Quest 핸드트래킹
python examples/verify_rig.py --rig rigs/nero_orca_right.yaml           # rig 기구학 점검

python tools/align_frames.py robot --robot robots/nero.yaml            # 로봇을 정준 축에 정렬
```

테스트: `pip install -e '.[all,dev]' && pytest`.

## 문서

- [**API 레퍼런스**](docs/API.md) — 서브패키지별 공개 심볼과 시그니처.

## 감사의 말

whatslab 은 훌륭한 오픈소스 위에 만들어졌다:
[Pinocchio](https://github.com/stack-of-tasks/pinocchio)(강체 기구학/IK),
[dex-retargeting](https://github.com/dexsuite/dex-retargeting)(손 리타게팅),
[viser](https://github.com/nerfstudio-project/viser)(웹 3D 시각화),
[LeRobot](https://github.com/huggingface/lerobot)(데이터셋 포맷).

## 라이선스

**주식회사 왓츠랩(WHATs LAB Corp) 소스 코드 라이선스**(CC BY-NC-ND 4.0 기반)를 따른다
— source-available, 비영리, 2차저작물 금지. [LICENSE](LICENSE) 참고.

Copyright © 주식회사 왓츠랩(WHATs LAB Corp). All rights reserved.
