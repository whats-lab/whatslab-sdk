# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

커밋 메시지는 이 저장소 관례대로 **한국어**로 쓴다.

**파이썬 코드에는 독스트링도 주석도 쓰지 않는다** (`src`·`tools`·`examples`·`tests`).
예외는 `# noqa`/`# type:`/`# pragma`/shebang, 그리고 `argparse(description=__doc__)` 를
쓰는 `tools/` 4개 파일의 모듈 독스트링뿐이다. 설명이 필요하면 코드 구조·이름으로
드러내거나 이 파일·`docs/API.md`·rig yaml 에 쓴다(yaml 주석은 유지한다).

## 개발 환경

`python`/`python3`(시스템)에는 whatslab 이 없다. 개발·테스트는 micromamba 환경에서 한다:

```bash
/home/whatslab09/micromamba/envs/dex_mj/bin/python   # editable 설치됨, all extras (numpy 1.26)
/home/whatslab09/micromamba/envs/dex_vla/bin/python  # Isaac Sim 스택 (USD export 용)
```

새 환경이면 `pip install -e '.[all,dev]'`. extras: `receiver`(python-osc) / `hand`
(dex-retargeting·pin·nlopt·torch) / `arm`(pin) / `viz`(viser·trimesh) / `data`
(pyarrow·imageio) / `all` / `dev`(pytest). URDF·메쉬는 별도 패키지
[`dexhand-description`](https://github.com/whats-lab/dexterous-hand-urdf)가 제공한다.

## 명령어

```bash
$PY -m pytest -q -rs                            # 전체 (기준: 100 passed, skip 0)
$PY -m pytest tests/test_arm.py -q -rs          # 파일 단위
$PY -m pytest tests/test_arm.py::test_x -x -q   # 단일 테스트
```

`-rs` 를 항상 붙인다. 무거운 deps 는 `pytest.importorskip`(pinocchio, dex_retargeting,
viser, trimesh, lerobot)으로 게이팅되어 있어, extras 가 빠진 env 에서는 "통과"가 실제로는
skip 이다. 린터 설정은 pyproject 에 없다(강제 린트 없음).

```bash
$PY examples/quest_arm.py --rig rigs/nero_orca_right.yaml [--arm controller|wrist] [--viz]
$PY examples/verify_rig.py --rig rigs/nero_orca_right.yaml [--write]   # reach_max 샘플링/기록
$PY tools/align_frames.py robot|attach|ee ...   # viser 정렬 튜너 (아래 3단계 워크플로우)
$PY tools/bench_arm_ik.py --traj fk|wave|reach|slow [--floor] [--set solver.backend=dls]
                                                # 팔 IK 고정 기준선 (정확도/연속성/비용)
$PY tools/export_combined_urdf.py --rig … --out …     # rig → 단일 URDF (sim 에셋)
$PY tools/export_combined_usd.py  --rig … --out …     # dex_vla(Isaac) env 에서만
$PY tools/export_targets.py …                         # v2.1 데이터셋 → GR00T / v3.0
scripts/install_quest_app.sh [PoseDataTracker*.apk]   # adb 로 Quest 앱 설치
```

## 아키텍처

**파이프라인** (`src/whatslab/model/base.py`) — `TeleopModel.get_q()` 가 매 호출마다
`get_data() → _apply_calib() → solve()` 를 엮어 `{side: {joint_name: rad}}` 를 낸다.
캐시 없음, 항상 양손(`SIDES = ("left","right")`) 처리. 서브클래스가 구현할 추상 훅은
`_get_raw_target()` **하나**뿐 — 어느 소스를 팔 EE 목표로 쓸지와 그 프레임을 정한다
(`model/quest.py` = 손목, `model/glove.py` = 컨트롤러+글러브 햅틱). side 를 `None` 으로
주면 그 side 는 IK 를 건너뛰고, 목표/손가락 입력이 없는 컴포넌트는 q 에서 **생략**된다
(0 을 채우지 않는다).

**레이어 규칙** — `receiver → core`, `model → core·robot`. receiver 는 model 을 import
하지 않는다. 컴포넌트를 엮는 조립은 소비자(sim/ROS2 노드) 몫이다. 계약은
`core/interfaces.py` 의 Protocol(`Receiver`/`HandController`/`ArmSolver`) — 구조적 타이핑이라
시그니처만 맞으면 커스텀 구현이 그대로 꽂힌다. 이 방향을 깨는 import 를 추가하지 말 것.

**정준 프레임**(x=앞, z=위, 오른손계)이 전 경계의 불변식이다. 리시버 출력, `TeleopModel`
입출력, `RobotModel` 의 데카르트 API 모두 정준. `RobotModel`(`robot/model.py`)은
"정준 샌드위치": 입력 4x4 → `to_base()` → IK, **q 출력은 무변환**(관절공간은 프레임 무관),
데카르트 출력은 요청 시에만 `to_canonical()`. `RobotModel` 은 무상태(정의 + 기하 함수만);
현재 q·목표·평활 상태는 호출자/솔버가 소유한다.

**팔 IK 책임 분리** (이 층을 건드릴 때 반드시 구분):
- `model/calibration.py` `ArmCalibration` — yaw 정렬 스냅샷 + 사람→로봇 reach 스케일.
  스케일은 **여기서만** 한다. `rig calibration.enabled` 는 **reach 스케일만** 게이트한다
  (`RobotModel.solve` 의 기존 의미와 같다) — off 여도 yaw 캘리브(`W`)는 그대로 동작한다.
  `calibrate_reach(persist=True)` 는 `save_calibration` 으로 `input_reach` 를 쓰면서
  `enabled` 를 **true 로 덮어쓴다**(`robot/config.py`).
- `model/ik.py` `RobotArmIK` — 정준→베이스 변환, `reach_max` 클램프(안전망), 첫 타깃
  `solve_robust` 시드, 스톨 시 전역 재탐색(`_recover_if_stalled`), 캘리 시 `reseed()`.
  계약은 `solve(T_canonical) -> q_arm` + `joint_names` 둘뿐 — 커스텀 IK 교체 가능.
- `teleop/arm/arm_ik.py` — 수치 해법만. `ArmIK`(dls: 매 프레임 수렴, 정밀) /
  `DiffArmIK`(diff: 틱당 소수 스텝 + rate-limit + null-space, 텔레옵 권장).
  `teleop/arm/builders.py:backend_cls(rig.solver.backend)` 로 선택.

**팔 IK 를 만질 때의 규칙** (전부 실측으로 확인된 것 — 어기면 같은 회귀가 반복된다):
- **추종과 전역 탐색을 섞지 말 것.** 프레임 추종은 백엔드 `solve()`. `solve_robust`
  는 후보 하나가 full-convergence DLS(수 ms)라 후보 10개면 60Hz 예산(16.7ms)을
  넘는다 → **첫 타깃 / `reseed()` / 확실한 스톨** 에서만 부른다. 매 프레임 부르면
  정확도는 그대로인데(측정: 15.4 vs 15.7mm) 비용 6배 + 프레임마다 다른 분기로 튄다.
- **전역 해를 무조건 채택하지 말 것.** 후보가 관절범위 균등 랜덤이라 현재 자세와
  무관하다 → 거리 가중 선택(`reseed_w_dist`) + 개선분 확인(`reseed_min_gain`) +
  틱당 이동 상한(`reseed_dq_max`) 세 겹이 있어야 EE 순간이동이 안 난다.
- **침묵 절단(시간 예산 등) 금지.** 후보를 조용히 버리면 전역 탐색이 사실상 꺼진
  채로 "동작하는 것처럼" 보인다(실제로 그렇게 8ms 예산이 탐색을 껐던 적이 있다).
- **정확도는 `tools/bench_arm_ik.py --traj fk` 로만 판정한다.** 좌표계로 합성한
  궤적은 도달 불가 구간을 지나 솔버 품질과 도달성을 섞는다. `fk` 궤적(유효 q → FK)
  은 하한이 0 이라 남는 오차가 전부 솔버 탓이다. 의심되면 `--floor`.
- **수치 반복값은 rig `solver:` 에서만 튜닝한다**(`max_iter`/`iters_per_call`/`tol`).
  코드 기본값을 고치면 diff 에 남지 않는다 — `max_iter` 가 코드에서 5 로 내려가
  전역 탐색 후보가 전부 미수렴했던 사례가 있다.

**config 2계층** (`src/whatslab/configs/`):
- `robots/*.yaml` = "이 로봇이 무엇인가" — 설치 무관 스펙(`urdf`, `axis_align`, `ee`).
- `rigs/*.yaml` = 조립·설치 — robots 참조 + `mount`/`attach`/`lock_joints`/`solver`/
  `calibration`. arm·hand 둘 다 optional(부분 조립).
- 회전값은 전부 config(URDF origin 관례)에 있다 — 코드에 하드코딩 회전을 넣지 않는다.
  값은 `tools/align_frames.py` 3단계(robot→attach→ee)로 튜닝해 확정한다.
- 캘리브 결과는 rig yaml 에 역기록된다(`robot/config.py` 의 `save_calibration`,
  `save_reach_max`) → 커밋 diff 에 `input_reach`/`reach_max` 만 바뀐 게 정상.

**손 리타게팅** — `HandPose`(사람 골격, 관절명→회전)가 정본이고 `to_sensor_array()` 로의
배열화는 `teleop/hand/controller.py` 경계에서만 일어난다. 엔진은
`teleop/hand/retargeter.py`(dex-retargeting 2단계 vector+position IK, `maxeval` 종료라
결정적). 로봇 손 추가는 `teleop/hand/hand_configs/` 에 config 를 등록하는 방식.
추적이 끊기면 직전 명령을 유지한다(급변 방지).

**자산 경로** (`paths.py`) — `models_root()`: `WHATSLAB_MODELS_ROOT` > `dexhand_description`
패키지 share. `configs_root()`: `WHATSLAB_CONFIGS_ROOT` > 동봉 `whatslab/configs`.

`whatslab` 은 PEP 420 네임스페이스 패키지다 — `src/whatslab/__init__.py` 를 만들지 말 것.

## 이 저장소의 제약

- **`numpy>=1.24,<2` 상한 유지.** 공유 SDK 라 소비자 스택(dex-retargeting, Isaac Sim 5.1)에
  맞춘 의도적 핀이다. 상한을 풀면 소비자 env 가 오염된다.
- **pip 전용 스택 유지.** casadi/IPOPT·conda-forge pinocchio 를 도입하지 않는다(팔 IK 가
  해석 야코비안 + DLS 인 이유). 손·팔·viz 가 pip `pin` 하나를 공유한다.
- 무거운 의존(python-osc 등)은 **`start()` 안에서 lazy import** — 모듈 import 만으로
  extra 를 강제하지 않는다.
- 기본 OSC 포트: Quest 9000(`receiver/quest_base.py`), 글러브 수신 4040 / 송신 4042
  (`receiver/glove_base.py`). 포트별 서버는 `osc_transport._registry` 싱글턴을 공유하므로,
  테스트는 `tests/conftest.py` 의 autouse 픽스처가 매번 레지스트리를 비운다.
- 데이터 글러브 경로는 Spine **2.3.1 이하**만 지원(Quest 경로는 Spine 무관).
- 글러브 OSC 주소·페이로드의 정본은 **Spine `docs/OSC_Protocol.md`** 다. 여기서 임의로
  주소를 바꾸지 말 것 — `GloveHumanHandReceiver` 는 `/{side}/quat/get`,
  `GloveRobotHandReceiver` 는 `/{side}/joint_angles/get` + `/{side}/wrist/get` 을 받는다.
  모든 Spine 메시지의 `args[0]` 은 messageType 헤더(10진 문자열)이고 실데이터는
  `args[1]` 부터다. `joint_angles` 는 **(이름, rad) 쌍**이라 배열 순서를 가정하지
  않는다(순서 = pinocchio 내부 순서, 프로파일마다 다름). `wrist` 만
  `HandCoordinateConvention` 을 타지 않고 raw `(w,x,y,z)` → `[y,x,z,-w]` 로 나가므로
  `unpack_wrist` → `spine_lh_xyzw` → `wrist_to_canonical` 3단계로 quat 계열과 같은
  프레임에 올린다 — 이 조합은 프로토콜 문서에서 유도했고 실기 검증은 아직 없다.
- `whatslab.data` 는 lerobot 라이브러리 없이 v2.1 을 쓰는 경량 sink 다 — lerobot 을
  런타임 의존으로 추가하지 않는다(numpy1 env 비오염 목적).
- 새 런타임 자산은 `[tool.setuptools.package-data]` 에 명시해야 wheel 에 들어간다
  (`include-package-data = false`).
- 커밋 메시지: `type(scope): 한국어 요약` (예: `feat(arm-ik): …`, `fix(ik): …`).
- 공개 API 를 바꾸면 `README.md`·`README.ko.md`·`docs/API.md` 3종을 함께 맞춘다
  (`docs/API.md` 가 공개 심볼 표의 정본).
- 라이선스: source-available(CC BY-NC-ND 4.0 기반), PyPI 미업로드 —
  `"Private :: Do Not Upload"` 분류자를 제거하지 않는다.
