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
$PY -m pytest -q -rs                            # 전체 (기준: 117 passed, skip 0)
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

**파이프라인** (`src/whatslab/teleop/base.py`) — `TeleopModel.get_q()` 가 매 호출마다
`get_data() → _apply_calib() → solve()` 를 엮어 `{side: {joint_name: rad}}` 를 낸다.
캐시 없음, 항상 양손(`SIDES = ("left","right")`) 처리. 서브클래스가 구현할 추상 훅은
`_get_raw_target()` **하나**뿐 — 어느 소스를 팔 EE 목표로 쓸지와 그 프레임을 정한다
(`teleop/models/quest.py` = 손목, `teleop/models/glove.py` = 컨트롤러+글러브 햅틱). side 를 `None` 으로
주면 그 side 는 IK 를 건너뛰고, 목표/손가락 입력이 없는 컴포넌트는 q 에서 **생략**된다
(0 을 채우지 않는다).

**레이어 규칙** — `core ← receiver`, `core·paths ← solvers`, `core·robot·solvers·receiver
← teleop`. 즉 `teleop` 만 위쪽이고 나머지는 서로를 모른다:
- `solvers/` = **수치 해법만**. 팔 IK(`arm/arm_ik.py` DLS)·손 리타게팅
  (`hand/retargeter.py` nlopt). 리시버도 rig 도 모른다.
- `teleop/` = 파이프라인(`base.py`) + 전처리(`calibration.py`) + 장치별 조립
  (`models/quest.py`·`glove.py`·`hand.py`). **리시버를 하드와이어하는 곳은 여기뿐**이다.
- `robot/` = rig config 또는 yaml 경로 → `RobotModel`(정의 + 기하 + 솔버 소유) +
  `RobotArmIK`(런타임 정책, 유상태: warm start·콜드스타트 카운터). `solvers` 를
  백엔드로 고른다. `RobotModel` 은 side 마다 하나씩 만든다 — 솔버가 유상태다.
커스텀 조립을 원하는 소비자(sim/ROS2 노드)는 `teleop/models/quest.py` 를 본떠
`TeleopModel` 을 상속하고 `_get_raw_target()` 만 구현하면 된다. 계약은
`core/interfaces.py` 의 Protocol(`Receiver`/`HandController`/`ArmSolver`) — 구조적 타이핑이라
시그니처만 맞으면 커스텀 구현이 그대로 꽂힌다. 이 방향을 깨는 import 를 추가하지 말 것.

**side 별 상태는 `SideModel`(`teleop/side.py`) 안에만 둔다.** `TeleopModel.sides`
(`{side: SideModel}`)가 유일한 소유자이고 항상 `SIDES` 전부를 담는다(로봇 없는
side 는 `robot=None`). `SideModel` 은 그 side 의 `robot`·`ik`·`retarget`·`calib`·
`safety` + 이 틱의 `raw_target`·`target`·`q` 를 들고 있다.

이 구조는 실측 버그 3개의 결과다. 전에는 side 로 키를 잡은 딕셔너리가 8개
병렬로 있었고, 그중 하나가 side 별이 아닐 때마다 터졌다 — `SIDES` 는 항상 둘이고
같은 rig 를 쓰면 **관절 이름도 같아서** 유상태 컴포넌트를 공유하면 두 side 가
서로를 밀어낸다: 솔버 `history_data`(오른쪽만 2.6mm → 양쪽 347mm),
`SafetyFilter._last`(8.9 → 72.2mm, 실기 221mm), `_cold_start` 가 공유 솔버의
`history_data` 를 시드로 씀(26.1 → 13.6mm). **side 를 도는 루프에 유상태 객체를
하나만 끼우고 있으면 그건 버그다** — 새 side 별 상태는 `SideModel` 필드로 넣는다.
특히 한쪽 컨트롤러가 반대쪽 팔 rig 에 도달 불가일 때(왼손 → 오른팔 rig) 그쪽 IK 가
계속 쓰레기 q 를 내므로 오염이 크게 증폭된다. 한 팔만 쓸 때는
`TeleopModel([None, rig])` 로 반대쪽을 끈다(`examples/quest_arm.py --sides`).

**정준 프레임**(x=앞, z=위, 오른손계)이 전 경계의 불변식이다. 리시버 출력, `TeleopModel`
입출력, `RobotModel` 의 데카르트 API 모두 정준. `RobotModel`(`robot/model.py`)은
"정준 샌드위치": 입력 4x4 → `to_base()` → IK, **q 출력은 무변환**(관절공간은 프레임 무관),
데카르트 출력은 요청 시에만 `to_canonical()`. `RobotModel` 은 무상태(정의 + 기하 함수만; 같은 패키지의 `RobotArmIK` 는 유상태);
현재 q·목표·평활 상태는 호출자/솔버가 소유한다.

**팔 IK 책임 분리** (이 층을 건드릴 때 반드시 구분):
- `teleop/calibration.py` `ArmCalibration` — yaw 정렬 스냅샷 + 사람→로봇 reach 스케일.
  스케일은 **여기서만** 한다(전에 `RobotModel.solve` 에 두 번째 구현이 있었고 지웠다).
  `rig calibration.enabled` 는 **reach 스케일만** 게이트한다 — off 여도 yaw
  캘리브(`W`)는 그대로 동작한다.
  `calibrate_reach(persist=True)` 는 `save_calibration` 으로 `input_reach` 만 쓴다 —
  `enabled` 는 이미 있으면 건드리지 않는다(없을 때만 true).
- `robot/arm_ik.py` `RobotArmIK` — 정준→베이스 변환, `reach_max` 클램프(안전망),
  **첫 타깃 콜드 스타트**(`_cold_start`), 캘리 시 `reseed()`. 그 뒤로는 백엔드
  `solve()` 만 부른다 — 스톨 탈출은 없다(전에 있었고 `|Δq|` 0.29 의 EE
  순간이동 원인이었다). **콜드 스타트에는 틱/이동 상한을 걸지 않는다** — 상한을
  걸면 전역 해가 수 rad 떨어져 있을 때 두 분기 사이에 갇힌다(실측: 캘리 후
  69.4 → 143.7mm 로 악화). 물리적 속도 제한은 하류의
  `SafetyFilter`(`max_joint_velocity`) 몫이다. 위치·방위를 **둘 다** 맞출 때까지
  프레임을 넘겨 재시도한다(`cold_max_tries`); 못 맞추면 경고하고 그 분기로 시작한다.
  `_cold_start` 는 `solve_robust(q_ref=...)` 에 시드를 **명시적으로** 넘긴다 —
  기본값(공유 솔버의 `history_data`)을 쓰면 다른 인스턴스가 남긴 자세를 상속한다.
  계약은 `solve(T_canonical) -> q_arm` + `joint_names` 둘뿐 — 커스텀 IK 교체 가능.
- `solvers/arm/arm_ik.py` — 수치 해법만. `ArmIK`(dls: 매 프레임 수렴, 정밀) /
  `DiffArmIK`(diff: 틱당 소수 스텝 + rate-limit + null-space, 텔레옵 권장).
  `solvers/arm/builders.py:backend_cls(rig.solver.backend)` 로 선택. 새 공개 심볼은
  `solvers/arm/__init__.py` 와 `solvers/__init__.py` 의 `__all__` 양쪽에 등록한다.

**팔 IK 를 만질 때의 규칙** (전부 실측으로 확인된 것 — 어기면 같은 회귀가 반복된다):
- **추종과 전역 탐색을 섞지 말 것.** 프레임 추종은 백엔드 `solve()`. `solve_robust`
  는 후보 하나가 full-convergence DLS(수 ms)라 후보 10개면 60Hz 예산(16.7ms)을
  넘는다 → **첫 타깃 / `reseed()` / 확실한 스톨** 에서만 부른다. 매 프레임 부르면
  정확도는 그대로인데(측정: 15.4 vs 15.7mm) 비용 6배 + 프레임마다 다른 분기로 튄다.
- **전역 해를 무조건 채택하지 말 것.** 후보가 관절범위 균등 랜덤이라 현재 자세와
  무관하다 → 채택 시 EE 가 순간이동한다. 그래서 지금은 첫 타깃과 `reseed()` 에서만
  쓴다. 추종 중 회복이 다시 필요해지면 `solve_robust(q_ref=, w_dist=)` 의 거리
  가중을 쓰고, 채택분은 `SafetyFilter` 예산으로 램프해 들어가야 한다.
- **침묵 절단(시간 예산 등) 금지.** 후보를 조용히 버리면 전역 탐색이 사실상 꺼진
  채로 "동작하는 것처럼" 보인다(실제로 그렇게 8ms 예산이 탐색을 껐던 적이 있다).
- **도달 가능한 목표로만 재면 여분 자유도의 가치가 안 보인다.** `fk` 궤적은 도달
  가능이 보장돼서 여분 관절을 잠가도 오차가 같다 — 그걸로 `joint7` 잠금을 "최적"
  이라 판정했다가, 실기에서 카펄이 한계에 박혀 방위를 못 맞추는 원인이 됐다.
  잠금 구성은 **도달 경계 목표**(임의 방위 도달률)로 판정한다.
- **정확도는 `tools/bench_arm_ik.py --traj fk` 로만 판정한다.** 좌표계로 합성한
  궤적은 도달 불가 구간을 지나 솔버 품질과 도달성을 섞는다. `fk` 궤적(유효 q → FK)
  은 하한이 0 이라 남는 오차가 전부 솔버 탓이다. 의심되면 `--floor`.
- **여분 자유도는 중앙으로 되돌리는 힘이 없으면 코너에 박힌다.** 널스페이스에서
  랜덤워크하다 한계에 도달하면 추종만으로는 못 나온다(실측: joint5 가 가동범위
  ±157.6° 로 제일 넓은데도 하한에 45% 프레임 붙어 있었다. 같은 프레임 전역탐색은
  포화 0% / 3.1mm, 추종은 54.0mm). 그래서 `k_posture > 0` 이 **필수**이고,
  `q_neutral` 은 `pin.neutral`(전부 0)이 아니라 **관절범위 중앙**이어야 한다 —
  joint4 `[-57.9, 122.6]`, carpal `[-65, 35]` 처럼 비대칭 범위에서 0 은 중앙이
  아니다. `_limit_gradient` 는 한계 0.10rad 안에서만 작동해 이미 늦다.
- **여분 자유도 배분은 `joint_weights` 로 한다. 위치·방위를 관절 블록으로 엄격
  분리하지 말 것.** 구형 손목이라고 분리가 유리한 게 아니다 — nero 는 joint5·6·7
  축이 0.0mm 로 정확히 교차하는데도, joint6 가동범위가 `[-41.8°, 54.4°]` 뿐이라
  분리하면 팔의 방위 기여 경로가 끊겨 66~72% 프레임에서 joint6 가 포화한다(실측:
  pos 89.3→51.4mm 로 좋아지지만 ori 18.3→38.1° 로 무너짐 — 그래서 `DecoupledArmIK`
  백엔드를 만들었다가 지웠다). 가중 DLS 는 커플링을 유지해 손목 포화 시 팔이
  이어받는다(joint1~4=2.5: run6 21.9→12.5mm, 방위도 동시 개선). 가중치는 **비율과
  절대 스케일이 둘 다 의미가 있다** — 감쇠항 `λ²I` 는 `W` 와 함께 스케일되지
  않으므로 `arm=2.0` 과 `wrist=0.5` 는 다르게 동작한다.
- **가중치는 시작점을 여러 개 잡고 판정한다.** 단일 시작점 리플레이는 초기 basin
  탈출 여부가 지배해서 후보 순위가 뒤집힌다(실측: 같은 설정이 시작점 0 에서 89mm,
  15% 지점에서 5mm). 데이터셋 2개 × 시작점 6개가 최소선이다.
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
  쓰기는 임시파일 + `os.replace` 로 원자적이다 — 텔레옵이 도는 중에 다른
  프로세스가 같은 yaml 을 읽어도 잘린 파일을 보지 않는다(실제로 pytest 가
  `backend: 'dl'` 로 읽어 깨진 적이 있다). 다만 `yaml.safe_dump` 로 **재직렬화**
  하므로 **rig yaml 의 주석은 캘리브 저장 시 사라진다** — rig 값의 근거는 주석이
  아니라 이 파일이나 `docs/API.md` 에 쓴다(robots yaml 은 안 쓰이므로 무관).

**손 리타게팅** — `HandPose`(사람 골격, 관절명→회전)가 정본이고 `to_sensor_array()` 로의
배열화는 `solvers/hand/controller.py` 경계에서만 일어난다. 엔진은
`solvers/hand/retargeter.py`(dex-retargeting 2단계 vector+position IK, `maxeval` 종료라
결정적). 로봇 손 추가는 `solvers/hand/hand_configs/` 에 config 를 등록하는 방식.
추적이 끊기면 직전 명령을 유지한다(급변 방지).

**자산 경로** (`paths.py`) — `models_root()`: `WHATSLAB_MODELS_ROOT` > `dexhand_description`
패키지 share. `configs_root()`: `WHATSLAB_CONFIGS_ROOT` > 동봉 `whatslab/configs`.

`whatslab` 은 PEP 420 네임스페이스 패키지다 — `src/whatslab/__init__.py` 를 만들지 말 것.

## 이 저장소의 제약

- **`numpy>=1.24,<2` 상한 유지.** 공유 SDK 라 소비자 스택(dex-retargeting, Isaac Sim 5.1)에
  맞춘 의도적 핀이다. 상한을 풀면 소비자 env 가 오염된다.
- **pip 전용 스택 유지.** casadi/IPOPT·conda-forge pinocchio 를 도입하지 않는다(팔 IK 가
  해석 야코비안 + DLS 인 이유). 손·팔·viz 가 pip `pin` 하나를 공유한다.
- **lazy import 금지.** 함수 안에서 import 하지 않는다 — 전부 모듈 최상단이다.
  결과로 `import whatslab.teleop` 이 pinocchio·dex_retargeting·torch·nlopt·python-osc
  를 전부 끌어온다(약 0.9초). extra 를 나눠 설치하는 소비자는 `[all]` 을 써야 한다.
  예외는 둘뿐이고 각각 하드한 이유가 있다:
  `paths.models_root()` 의 `dexhand_description`(= `WHATSLAB_MODELS_ROOT` 로
  덮어쓰면 패키지 없이 동작해야 한다), `solvers/hand/spherical_fk.py` 의 `rerun`
  (선언된 의존이 아니다 — 최상단으로 올리면 모듈 자체가 못 뜬다).
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
