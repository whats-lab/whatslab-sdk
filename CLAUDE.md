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
(onnxruntime·pin) / `arm`(pin) / `viz`(viser·trimesh) / `data`
(pyarrow·imageio) / `all` / `dev`(pytest). URDF·메쉬는 별도 패키지
[`dexhand-description`](https://github.com/whats-lab/dexterous-hand-urdf)가 제공한다.

## 명령어

```bash
$PY -m pytest -q -rs                            # 전체 (센서 URDF: 117 passed/6 skip)
$PY -m pytest tests/test_arm.py -q -rs          # 파일 단위
$PY -m pytest tests/test_arm.py::test_x -x -q   # 단일 테스트
```

`-rs` 를 항상 붙인다. **skip 6개는 URDF 자체가 없는 손**(allegro/schunk/tesollo/
ability)이다. 동봉 `dexhand_description` 은 센서 프레임이 없어 손 config 유도가 막히고
skip 이 14개로 는다 — 손 쪽을 만질 때는 `WHATSLAB_MODELS_ROOT` 로 센서 프레임이 있는
models root 를 가리켜야 실제로 검증된다. 무거운 deps 는 `pytest.importorskip`(pinocchio, onnxruntime,
viser, trimesh, lerobot)으로 게이팅되어 있어, extras 가 빠진 env 에서는 "통과"가 실제로는
skip 이다. 린터 설정은 pyproject 에 없다(강제 린트 없음).

```bash
$PY examples/quest_arm.py --rig rigs/nero_orca_right.yaml [--arm controller|wrist] [--viz]
$PY examples/verify_rig.py --rig rigs/nero_orca_right.yaml [--write]   # reach_max 샘플링/기록
$PY tools/align_frames.py robot|attach|ee ...   # viser 정렬 튜너 (아래 3단계 워크플로우)
$PY tools/bench_arm_ik.py --traj fk|wave|reach|slow|walk|overshoot [--floor]
                          [--seeds 40] [--dump out.json] [--set solver.backend=dls]
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
  (`hand/uni_retargeter.py` ONNX 순전파). 리시버도 rig 도 모른다.
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
  `rig calibration.enabled` 로 통째로 껐다 켠다 — off 면 리시버 좌표를 그대로
  목표로 쓴다(스케일도 `W` 도 없음). 실행 중 A/B 는 `examples/quest_arm.py --no-calib`.
  on 이면 `target = scale·(p0 + W(p − p0))`, `rot = W·G` — `p0`/`W` 는 yaw 캘리브
  시점 스냅샷이고 `W` 는 **변위에만** 걸린다(위치는 리시버에서 이미 상대로 들어온다).
  캡처 전에는 `p0` 가 없어 `scale·p` 로 동작한다.
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
  가능이 보장돼서 여분 관절을 잠가도 오차가 같다. 잠금 구성은 **도달 경계 목표**
  (실제 작업 위치 x 임의 방위의 도달률)로 판정한다.
- **`nero_orca_right` 의 `lock_joints: ["joint7"]` 은 비용을 알고 택한 것이다 —
  측정이 나빠 보여도 되돌리지 말 것.** 실측(run6/run5 + 방위 도달률):

  | | run6 추종 | run5 추종 | 방위 도달률 |
  |---|---|---|---|
  | joint7 해제 | 14.5mm / 5.3° | 1.4mm / 0.5° | 61% |
  | joint7 잠금 | 22.3mm / 13.0° | 2.4mm / 0.8° | 32% |

  즉 잠금은 방위 도달률을 반토막 내고 추종·포화도 악화시킨다. 그래도 잠근 이유는
  실기 조작감이다(관절 하나가 덜 움직여 동작이 차분하다). 숫자만 보고 해제하지
  말고 사용자에게 확인할 것. **비싸게 만드는 것(`joint_weights: {joint7: N}`)은
  대안이 아니다** — 순간 사용량은 줄지만 경로가 비효율적이 되어 총회전이 오히려
  늘고(4637° → 6519°) 추종도 나빠진다(14.5 → 18.1mm).
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
- **추종 품질은 단일 궤적으로 판정하지 않는다.** `--traj walk` 는 시드별 평균 오차가
  3~150mm 로 흩어진다 → `--traj walk --seeds 40 --dump` 로 시드별 값을 받아
  **대응차(paired)** 로 비교한다. 단일 시드로 판정하면 없는 원인을 만들어낸다.
- **`dtheta_max` 는 목표의 변화량이 아니라 현재 EE 자세에서 목표까지의 각도를 자른다.**
  크게 두면(≥1.0 은 사실상 무제한) 도달 불가 방위를 끝까지 쫓다 위치를 내준다.
  위치/방위 트레이드이므로 `score = pos + 0.1·ori` 로 판정한다.
- **널스페이스 투영자는 감쇠 유사역으로 만들지 말 것.** `N = I − J⁺J` 에서 `J⁺` 가
  감쇠를 쓰면 멱등이 아니다(태스크 감쇠 공유 시 `‖N²−N‖` 최대 0.38, 별도 1e-6 으로
  분리해도 0.17) → `_null_projector` 는 SVD rank 절단(`solver.proj_rcond`)으로
  정확한 직교 투영자를 만든다. 감쇠판의 누설이 우연히 도움이 되고 있었으므로
  투영자를 고칠 때 `solver.k_limit` 을 함께 재튜닝해야 한다(0.15 → 0.30).
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

**손 리타게팅** — 입력은 **사람 손 URDF 관절각**(`HandPose.joint_angles`, 이름→rad)
하나뿐이다. 글러브(Spine)가 `/{side}/joint_angles/get` 으로 (이름, rad) 쌍을 보내고
`GloveHumanAnglesReceiver` 가 받는다. 엔진은 **학습된 통합 ONNX 모델 하나**
(`solvers/hand/uni_retargeter.py` + `assets/uni_all.onnx`)다 — 사람 FK·인코더·
통합 헤드·관절범위 역정규화가 전부 그래프 안이고, 로봇 의존부(관절 기술자 19D·
관절범위·사이드 부호)는 `assets/uni_tables.npz` 에서 **입력**으로 들어간다.
로봇이 늘어도 그래프는 안 바뀐다. 추적이 끊기면 직전 명령을 유지한다(급변 방지).
- **dex(dex_retargeting 2단계 IK)·kp(키포인트 IK) 백엔드는 제거했다.** 프레임별
  최적화 없이 순전파 한 번(CPU 1스레드 약 1.1ms, 900Hz+)으로 대체됐고 좌우·5개
  손(orca/allegro/tesollo/robotis/human)이 그래프 하나를 쓴다. `backend=` 인자는
  받지 않는다(rig 에 `backend:` 가 남아 있으면 에러). 모델·표·학습 설정의 정본은
  retarget_net 저장소다.
- **새 로봇 손 온보딩**: retarget_net `tools/onboard_urdf.py` 가 URDF 하나에서 표를
  뽑는다(센서 프레임 계약: `{side}_sensor_dorsum` + 손가락마다 `_proximal`/`_distal`.
  손가락 체인 밖 활성 관절은 손목으로 보고 자동 고정 — orca 손목이 이 경우).
  표는 모델을 돌게 할 뿐이라, 학습에 없던 손은 몇백 스텝 미세조정 후 그래프를
  재수출해야 성능이 나온다.
- **사람 관절 이름은 반드시 그 side 의 이름으로 인덱싱한다.** 표에는
  `human:left:joints`/`human:right:joints` 가 따로 있다. 왼손 목록(`human:joints`)
  으로만 `_hidx` 를 만들면 오른손 입력이 전부 미스 → `q_human` 이 0 벡터로 남고
  ONNX 출력이 상수다(실측: 오른손 이름적중 0/12, `|Δq|` 0.00rad — 손이 아예
  움직이지 않는다). 별칭은 `left_`/`right_` 양쪽을 벗겨 넣는다.
- `hand_configs/`(URDF 경로 해석 + 센서 프레임 사슬 유도)와 `human_fk.py`
  (pinocchio FK)는 리타게팅 엔진이 아니라 **실물 관절 매핑(`examples/robot_io.py`)
  과 viz 전용**이다. `UniRetargeter.urdf_path` 는 pinocchio 없이 경로만 계산한다.
  dex/kp 시절의 잔재(`get_two_stage_config`·`_SCALE_FACTOR`·`_KP_*`·
  `_TARGET_JOINT_NAMES`·`_FIXED_JOINTS`·`_HUMAN_CHAIN` 의 사람관절 짝짓기·
  `FingerChain`·`rot_between`·`palm_frame_from_fingers`·`HumanHandFK.positions`)는
  전부 지웠다 — 손별 선언은 `_CHAIN_LEN`(손가락 → 사슬 길이) 하나다.
- **viz 의 손 클래스는 하나다**(`viz.HandViz`). 전에는 `_UrdfHandViz` 를
  `RobotHandViz`/`HumanHandViz` 가 상속했는데, 두 서브클래스의 차이는 루트 자세를
  어디서 얻느냐뿐이었고 `RobotHandViz` 쪽 경로(`_r_origin`/`_r_frame`)는
  `UniRetargeter` 에 그 속성이 없어 죽은 코드였다. 지금은 `root_pose` 를 호출자가
  `upright_root`/`human_upright_root` 로 만들어 넘긴다. `HandSkeletonViz` 는
  없는 `_bone_pairs()` 를 부르던 깨진 클래스라 지웠다.
- **구면관절 FK(`spherical_fk.py`)는 제거했다.** 글러브가 quat 을 보낼 때의
  경로였고 q 가 오는 지금은 필요 없다. 대가로 **Quest 핸드트래킹 리타게팅은
  지원하지 않는다** (Quest 컨트롤러 → 팔 IK 경로는 그대로다).

**자산 경로** (`paths.py`) — `models_root()`: `WHATSLAB_MODELS_ROOT` > `dexhand_description`
패키지 share. `configs_root()`: `WHATSLAB_CONFIGS_ROOT` > 동봉 `whatslab/configs`.

`whatslab` 은 PEP 420 네임스페이스 패키지다 — `src/whatslab/__init__.py` 를 만들지 말 것.

## 이 저장소의 제약

- **`numpy>=1.24,<2` 상한 유지.** 공유 SDK 라 소비자 스택(dex-retargeting, Isaac Sim 5.1)에
  맞춘 의도적 핀이다. 상한을 풀면 소비자 env 가 오염된다.
- **pip 전용 스택 유지.** casadi/IPOPT·conda-forge pinocchio 를 도입하지 않는다(팔 IK 가
  해석 야코비안 + DLS 인 이유). 손·팔·viz 가 pip `pin` 하나를 공유한다.
- **lazy import 금지.** 함수 안에서 import 하지 않는다 — 전부 모듈 최상단이다.
  결과로 `import whatslab.teleop` 이 pinocchio·onnxruntime·python-osc
  를 전부 끌어온다(약 0.9초). extra 를 나눠 설치하는 소비자는 `[all]` 을 써야 한다.
  예외는 둘이다. `paths.models_root()` 의 `dexhand_description`(= `WHATSLAB_MODELS_ROOT`
  로 덮어쓰면 패키지 없이 동작해야 한다), `hand_configs/_base.py` 의
  `xml.etree`(URDF 링크 존재 확인용, 표준 라이브러리라 무게가 없다).
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
