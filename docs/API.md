# whatslab API

공개 API 목록. import 루트는 `whatslab` (PEP 420 네임스페이스). 시그니처는 소스에서
추출한 것이며 `self` 는 생략한다. 개념·사용 흐름은 [README](../README.md) 참고.

의존 규칙: `core ← receiver`, `core·paths ← solvers`,
`core·robot·solvers·receiver ← teleop`. `teleop` 만 위쪽이고 나머지는 서로를 모른다.
컴포넌트를 엮는 조립은 소비자 몫이다.

---

## whatslab.teleop — 사용자 대면 최상위 텔레옵 API

```python
from whatslab.teleop import QuestModel, GloveModel
```

| 심볼 | 설명 |
|---|---|
| `TeleopModel(robot)` | 베이스 클래스. 소스 리시버 + IK + 리타게팅 + 캘리브를 조립해 `get_q()` 를 낸다. `robot` = rig yaml 경로(또는 `[left, right]`, `{side: rig}`). 유저는 서브클래싱해 자기 하드웨어 조합을 정의. |
| `TeleopModel.sides` | `{side: SideModel}` — **side별 상태의 유일한 소유자.** 항상 `SIDES` 전부를 담는다(로봇 없는 side 는 `robot=None`). |
| `SideModel` | 한 side 의 전부: `side`·`robot`·`ik`·`retarget`·`calib`·`safety` + 이 틱의 `raw_target`·`target`·`q`. 유상태 컴포넌트가 이 안에만 있으므로 side 간 공유가 구조적으로 불가능하다. `solve`, `apply_calib`, `filter`, `sync_ik`, `reseed`. |
| `QuestModel(robot)` | 프리셋: Quest 핸드트래킹(손목→팔, 손가락→손). |
| `GloveModel(robot)` | 프리셋: 팔=Quest 컨트롤러 IK, 손=글러브 리타게팅. 햅틱 지원. |
| `HandModel(robot)` | 프리셋: 손 리타게팅 단독(팔 IK 없음). |
| `RobotArmIK(...)` | 팔 IK 컴포넌트. 정준 목표 4x4 → 팔 관절각. 프레임 추종은 rig 백엔드의 `solve()`, 전역 재탐색(`solve_robust`)은 **첫 타깃(`_cold_start`)과 `reseed()`** 에서만(후보 하나가 수 ms라 매 프레임 불가). 튜닝은 `cold_*` 속성. |
| `ArmCalibration(reach_max, input_reach, enabled=True)` | yaw 정렬 + reach 스케일 소유. `enabled=False` 면 **reach 스케일만** 건너뛴다 — yaw 캘리브는 그대로 동작한다(rig `calibration.enabled`). |

### `TeleopModel` 메서드

| 메서드 | 반환 | 설명 |
|---|---|---|
| `start()` / `stop()` | — | 소스 리시버 수신 시작/정지(대상은 arm/hand source 에서 자동 도출). |
| `_get_raw_target()` | `Dict[str, Optional[Pose]]` | **유일한 추상 훅.** 어느 소스를 팔 EE 목표로 쓸지와 그 프레임을 정한다. 서브클래스가 구현할 것은 이것뿐이다. |
| `get_data()` | `Dict[str, dict]` | `_get_raw_target()` 결과 + 리시버 값을 역할별로 모은다(arm_pose/fingers/q/tracked). 보통 오버라이드하지 않는다. |
| `solve(data)` | `Dict[str, Dict[str, float]]` | data → IK/리타게팅 → side별 `{joint: rad}`. |
| `get_q()` | `Dict[str, Dict[str, float]]` | 매 호출 get_data→calib→solve→safety 를 엮어 `{side: {joint: rad}}` 반환. `sides[s].q` 에도 남는다. |
| `calibrate_yaw()` | `Dict[str, bool]` | 손목 yaw 정렬 스냅샷(즉시). side별 성공 여부. |
| `calibrate_reach(duration=8.0, rate_hz=60.0)` | `Dict[str, bool]` | duration 초 폴링해 최대 도달반경 측정→calib 등록(블로킹). |
| `set_reach(input_reach)` | `Dict[str, bool]` | reach 스케일 스칼라를 직접 주입. |
| `send_feedback(data)` | — | 역방향 피드백(기본 no-op; `GloveModel` 이 햅틱으로 오버라이드). |

## whatslab.receiver — 입력 소스 (텔레옵 무관, 단독 사용 가능)

```python
from whatslab.receiver.quest.controller import QuestControllerReceiver
```

`side` = 물리적 기기의 좌/우(채널 재해석 금지). 출력은 항상 정준 프레임
(x=앞, z=위, 오른손계). `python-osc` 는 `start()` 에서 lazy import.

| 클래스 | `get(side)` → | 설명 |
|---|---|---|
| `QuestControllerReceiver` | `InputSample(controller=Pose)` | Quest 컨트롤러 6D 위치/자세. `connected(side)`. |
| `QuestHandReceiver` | `InputSample(hand=HandPose)` | Quest 핸드트래킹(손목 6D + 손가락). `connected(side)`. |
| `GloveHumanAnglesReceiver` | `InputSample(hand=HandPose(joint_angles=…))` | **사람 손** URDF 관절각(`/{side}/joint_angles/get`) + 손목. 손 리타게팅의 입력. |
| `GloveHumanHandReceiver` | `InputSample(hand=HandPose(joint_rot=…))` | AirGlove 손가락 회전(`/{side}/quat/get`). 전송 계층만 — 손 리타게팅은 관절각을 쓴다. |
| `GloveRobotHandReceiver` | `InputSample(joint_q=…, hand=wrist만)` | Spine 이 IK 를 끝낸 URDF 관절각을 직접 받는다(손 리타게팅 바이패스). `joint_map` = Spine 이름→로봇 관절명. |

공통: `start()`, `stop()`, `get(side) -> InputSample`.

## whatslab.robot — 팔 기구학 모델 + config 로더

| 심볼 | 설명 |
|---|---|
| `RobotModel(rig)` | `rig` 는 `RigConfig` 또는 rig yaml 경로(str/PathLike). 경로면 `load_rig` 로 읽는다. |
| `RobotModel.ee_pose(q_arm)` | FK: 관절각 → EE 4x4. |
| `RobotModel.to_base(T)` / `to_canonical(T)` | 정준↔베이스 프레임 변환. |
| `RobotModel.clamp_reach(T_base)` | 베이스 목표를 `reach_max` 구로 클램프(안전망). 구현은 `robot.model.clamp_reach(T_base, reach_max)` 모듈 함수 한 곳. |
| `RobotModel.sync_state(q_arm)` | IK 웜스타트용 현재 상태 갱신. |
| `RobotModel.make_hand_controller(config_name, side)` | 손 리타게팅 컨트롤러 생성. |
| `load_robot(path)` / `load_rig(path)` | yaml → `RobotSpec` / `RigConfig`. |
| `save_calibration(rig, input_reach)` / `save_reach_max(rig, reach_max)` | 캘리브 값을 rig yaml 에 기록. **yaml 을 재직렬화하므로 rig yaml 의 주석은 지워진다.** |

### rig `solver:` — 여분 자유도 배분

어느 관절을 먼저 쓸지, 자세를 어디로 되돌릴지. 둘 다 rig yaml 전용이고 코드 변경이
필요 없다.

| 키 | 설명 |
|---|---|
| `joint_weights: {관절명: 비용}` | 가중 DLS 의 관절 비용 `W`(기본 1.0, 양수). `dq = W⁻¹Jᵀ(JW⁻¹Jᵀ+λ²I)⁻¹e` — 싼 관절이 먼저 쓰인다. 커플링을 유지하므로 싼 관절이 한계에 걸리면 비싼 관절이 이어받는다. **비율과 절대 스케일이 둘 다 의미가 있다**(감쇠항 `λ²I` 는 `W` 와 함께 스케일되지 않는다). |
| `k_posture` / `k_limit` | 널스페이스에서 자세를 `q_neutral`(= 관절범위 중앙)로 되돌리는 힘 / 한계 근처에서 밀어내는 힘. 기본 0.05 / 1.0. **0 으로 두면 여분 자유도가 코너에 박혀 안 나온다.** |

`nero_orca_right` 실측(실기 녹화 run6 8747프레임 + run5 4058프레임, 시작점 3개 평균):

| 설정 | run6 pos/ori/포화 | run5 pos/ori/포화 |
|---|---|---|
| `k_posture=0`, 중립=0 | 21.9mm / 13.3° / 13.6% | 16.9mm / 5.6° / 7.4% |
| 현재 기본값 | **12.5mm / 4.8° / 4.9%** | **10.1mm / 1.0° / 1.4%** |
| 전역탐색 하한 | 2.4mm / 4.4° | — |

## whatslab.solvers.hand — 손 리타게팅

```python
from whatslab.solvers.hand import HandRetargetController
```

입력은 **사람 손 URDF 관절각** 하나다 — `HandPose.joint_angles`(이름→rad). 글러브가
`/{side}/joint_angles/get` 으로 보내는 값이고, 사람 FK 는 두 엔진이 `HumanHandFK` 를
공유한다(평범한 pinocchio FK).

| 심볼 | 설명 |
|---|---|
| `HandRetargetController(hand_type, config_name, backend="dex")` | 손 리타게팅 컨트롤러. `compute(InputSample) -> HandCommand`. `sample.hand.joint_angles` 가 비면 직전 명령 유지. `backend`: `"dex"` / `"kp"`. |
| `HumanHandFK(side, urdf_path=None)` | 사람 손 URDF FK. `points(angles)` = 손가락별 키포인트 4개 + `palm`, `positions(angles)` = `JOINT_INDEX` 배치의 (23,3). `joint_names` = URDF revolute 관절(base 프로파일 21개). 링크명으로 참조하고 손끝은 `{side}_sensor_{finger}_distal` → `{side}_{finger}_tip` 순으로 찾는다. |
| `HandRetargeter` | `dex` 백엔드 엔진 — dex-retargeting 2단계(vector + position) IK, nlopt/torch 필요. |
| `KPHandRetargeter(hand_type, config_name, keypoints=None, ...)` | `kp` 백엔드 엔진 — 팜상대 키포인트 결합 목적함수(가중 DLS + IRLS Huber), pin+numpy 만 사용. 팜 프레임 정렬 + 손길이 비율 스케일 + 중립 1회 구간별 방향 보정은 URDF 에서 자동 유도(키포인트는 `{side}_sensor_*` 우선, 없으면 사슬 추출, 아니면 `keypoints` 명시). 목적함수 = 팜상대 지문 위치 + 미터벡터 형상(`w_shape`, config `_KP_SHAPE_WEIGHT`) + 엄지쌍 상대벡터 램프 스냅(30mm 이하에서 목표→0, 가중 `w_pair`→`w_snap`) + 손가락간 최소분리 30mm. warm start 유상태 — side 마다 인스턴스 하나. `reset()` 으로 콜드 스타트 재개(orca 는 형상 전용 콜드 solve, `_KP_COLD_SHAPE`). |
| `HandKeyvector(model, data, chains, dorsum_frame, frac=0.5)` | 전처리 — 원점 `dorsum`, 축은 손가락방향 유도, `prox` = 사슬 `frac` 지점(기본 50% 중앙), `L_ref = ‖dorsum−middle_distal‖` 를 q=0 에서 상수로 고정. `encode(q) -> (5,6)` = `v_dorsum→distal` + `v_dorsum→prox`(`v_prox→distal` 은 선형종속이라 제외). `jacobian(q, idx_v) -> (5,6,n)`. **강체 사슬은 뼈 길이가 q 에 무관하므로 `prox` 의 구간·보간비가 상수**여서 고정 아핀 조합이고 미분 가능하다. 데이터셋 통계 정규화를 쓰지 않으므로 체크포인트에 통계를 딸려보낼 필요가 없다. |
| `chain_weights(seg_lengths, frac=0.5)` | 호길이 `frac` 지점의 `(구간, 보간비)`. |
| `sensor_chains(model, side, n_links=3)` / `human_chains(fk)` / `finger_columns(model, tip_fids)` | 사슬·관절열 유도. `sensor_chains` 는 `KPHandRetargeter` 와 같은 규약(마지막 3링크 + 센서 팁)이라 팜 프레임이 일치한다. |
| `KeyvectorFK(kv, idx_q, idx_v, q_template)` / `keyvector_fk(q_act, fk)` | pinocchio 정확 FK 를 `torch.autograd.Function` 으로. `∂L/∂q = (∂L/∂x)^T J`. 학습 전용(추론 경로에는 FK 가 없다). `idx_q`(형상)와 `idx_v`(속도)를 분리해서 받는다. **정본 참조 구현** — 속도가 필요하면 `TorchKeyvectorFK` 를 쓰고 이것과 대조 검증한다. |
| `TorchKeyvectorFK(kv, idx_q, joint_names, dtype=float64)` | **배치 FK 를 순수 torch 로** — 사슬을 URDF 에서 뽑아 회전축을 수치로 유도한 뒤 Rodrigues 로 배치 전개한다. autograd 가 그대로 붙어 수동 야코비안이 필요 없고 CUDA 로 올라간다. pinocchio 와 값·기울기가 기계 정밀도(2e-16)로 일치한다. 처리량은 `KeyvectorFK` 대비 배치 1024 에서 12배, 16384 에서 39배(CPU f64) / 65배(CUDA f32) — **이득의 대부분은 GPU 가 아니라 배치 벡터화다.** |
| `joint_axis(model, data, jid)` | 회전관절의 축을 수치로 유도(단순 회전이 아니면 에러). 바인딩별 축 표현 차이를 피한다. |
| `NetHandRetargeter(hand_type, config_name, checkpoint=None, ..., mirror_to=None)` | `net` 백엔드 엔진 — 사람 keyvector → 로봇 관절각. `compute(joint_angles) -> q` 는 `KPHandRetargeter` 와 같은 계약. forward 1회라 FK 가 없다. `mirror_to` 를 주면 입력 keyvector 의 z 성분을 반전해 좌우 통합 모델을 쓴다. |
| `HandNet(in_dim, joint_counts, hidden=128)` | 손가락별 MLP `(in_dim, hidden, hidden, n_joints_i)` + `Tanh`. 출력 `[-1,1]` 을 `joint_roms` 로 선형 매핑한다. |
| `AffineHandNet(net, n_finger, dim=6)` / `ResidualAffine(n_finger, dim=6)` | 손가락별 residual affine(`A=0`,`b=0` 초기화 → 항등). few-shot 앵커가 잡아야 하는 전역 정렬을 소수 파라미터로 흡수한다. |
| `coverage_loss` / `distance_loss` / `extension_loss` / `motion_loss_global` / `motion_loss_local` / `flatness_loss` / `pinch_loss` / `soft_pinch_loss` / `chamfer_both` / `chamfer_partial` | 학습 손실. `motion_loss_local` 은 두 섭동의 사잇각을 맞춰 회전에 불변하고, `motion_loss_global` 은 변위 방향을 직접 맞춘다. `coverage_loss(partial=True)` 는 단방향 Chamfer 로 로봇 여분 영역 왜곡을 피한다. `distance_loss` 는 손끝 쌍거리(접선), `extension_loss` 는 `‖dorsum→tip‖`(반경)을 맞춘다 — 굽힘 진폭은 반경량이라 쌍거리로는 안 잡힌다. |
| `CONFIG_REGISTRY` | `{config_name: HandConfig}` — 로봇 손 등록부. |

rig `hand_solver:` 로 코드 수정 없이 고른다 — `backend`(dex|kp) + kp 가중치
(`w_tip`/`w_shape`/`w_pair`/`w_snap`/`iters_per_call`).

`net` 백엔드는 학습이 필요하다 — `tools/train_hand_net.py`(Phase 1: GeoRT 5원칙 /
Phase 2: 단방향 Chamfer + 거리보존 + 로컬모션 + few-shot 앵커 + residual affine),
평가는 `tools/bench_hand_retarget.py --backends net --net-checkpoint …`.
좌우 통합 모델의 수용 기준은 `tools/check_mirror.py`.

**ONNX 내보내기** — `tools/export_hand_net_onnx.py --config … --side … --checkpoint …
--out x.onnx`. `q_human (batch, n) → q_robot (batch, m)` 그래프 하나로, **FK 까지
그래프 안에 들어가므로 추론에 torch·pinocchio 가 필요 없다.** 소비자는 `numpy` +
`onnxruntime` 만 있으면 된다(6735MB → 98MB). 사슬이 1-DoF 회전관절뿐이라 Rodrigues 로
전개되고 `joint_roms` 선형 매핑도 상수로 박힌다. 정본(pinocchio+torch) 경로와 최대
오차 2e-6 rad. 단일 프레임 0.20ms(정본 0.51ms), 배치 256 에서 프레임당 0.0045ms.
`--mirror` 는 좌우 통합 모델을 반대 side 에 쓸 때 z 성분을 반전한다.

## whatslab.core — 계약(타입 + Protocol), 의존성 0

| 심볼 | 설명 |
|---|---|
| `types.Pose` | 위치 + quaternion(xyzw). |
| `types.HandPose` | 손목 6D + `joint_angles`(관절명→rad, 손 리타게팅 입력) + `joint_rot`(관절명→회전, quat 전송용). |
| `types.InputSample` | 리시버 출력 컨테이너(controller/hand/q/hmd). |
| `types.HandCommand` | 리타게팅 출력(로봇 손 관절각). |
| `types.JointSpec` | 관절 이름/한계 스펙. |
| `interfaces.Receiver` / `HandController` / `ArmSolver` | 컴포넌트 Protocol(구조적 타이핑). |
| `interfaces.GlobalArmSolver` | `ArmSolver` + `solve_robust`·`pose_error`·`sync_state`·`q_neutral`. `RobotArmIK` 가 콜드 스타트·`reseed()` 에서 요구하는 계약이다(런타임에는 `hasattr` 로 확인). 커스텀 백엔드를 끼울 때 이 표면을 맞추면 콜드 스타트도 동작한다. |

## whatslab.data — LeRobot 데이터셋 sink (경량, lerobot 라이브러리 불요)

```python
from whatslab.data import LeRobotRecorder
```

| 메서드 | 설명 |
|---|---|
| `add_frame(state, action, images, replay, task)` | 한 프레임 누적. |
| `save_episode()` | 현재 에피소드를 v2.1 parquet 로 저장. |
| `finalize()` | 데이터셋 메타 마감. |

## whatslab.viz — viser 웹 3D 시각화 (`whatslab-sdk[viz]`)

`get_server(port=8080)` 로 포트당 서버 공유(여러 viz 공존). `http://localhost:8080`.

| 클래스 | 내용 |
|---|---|
| `URDFScene` | URDF 하나를 메쉬(STL)/스켈레톤 자동 판별 렌더 + 관절 구동. `set_root`, `q_from_named`, `fk`, `frame_pose`. |
| `RobotArmViz` | 팔+손 URDF 메쉬를 solver q 로 구동 + 목표 EE 프레임. `start`, `update`. |
| `RobotHandViz` | 로봇 손 링크 스켈레톤(q FK). |
| `HandSkeletonViz` | 사람 손 23관절 스켈레톤. |

## whatslab.safety — 운동학 안전 유틸 (dep-light)

로직만 제공. 강제(watchdog/e-stop 배선)는 소비자(ROS safety_gate 등)가, 최종 권위는 하드웨어가 갖는다.

| 심볼 | 설명 |
|---|---|
| `JointLimit` | 관절 pos/vel 한계. |
| `load_limits_from_urdf(urdf_xml)` | URDF → `{joint: JointLimit}`. |
| `tighten(base, ...)` | 한계를 보수적으로 조임. |
| `SafetyFilter` | clamp + rate-limit + hold/estop 상태기. `step(desired, dt=None)`, `trip`, `reset`, `estopped`, `enabled`, `set_enabled`, `seed`, `holding`, `clone`. **상태(`_last`)를 들고 있으므로 side 마다 하나씩 필요하다** — `clone()` 으로 복제한다. |

## 알려진 결합·제약

- **`receiver.quest.CONTROLLER_POS_OFFSET`** (`[0.02, -0.04, 0.1]`) 는 컨트롤러 마운트
  편차 보정으로 리시버가 `controller.pos` 에 가산한다(외부 설정 불가). **Quest 앱
  (PoseDataTracker)의 같은 이름 상수와 값이 일치해야 한다** — 저장소가 달라 자동
  검증이 안 되므로 앱을 올릴 때 함께 확인한다.
- **reach 스케일은 정준 원점 기준, `reach_max` 는 로봇 베이스 기준**이다. 둘의 중심이
  `mount.xyz` 만큼 떨어져 있으면 최대로 뻗은 목표가 `clamp_reach` 에 잘린다
  (`mount.xyz = [0, -0.15, -0.3]`, 오프셋 0.335m 에서 최악 방향 초과 335mm — 실기
  run6 에서는 초과 프레임 0.3%, 초과분 최대 30mm). `Diag` 의 `clamp` 비율이 높으면
  이것부터 본다.
- **`whatslab.robot` import 가 `hand` extra 를 요구한다.** 함수 안 import 금지
  방침(모든 import 는 모듈 최상단)의 결과다 — `robot/model.py` 가
  `solvers.hand.HandRetargetController` 를 최상단에서 끌어온다. extra 를 나눠
  설치하는 소비자는 `[all]` 을 써야 한다. `[arm]` 만으로는 `RobotModel` 을 못 쓴다.

## whatslab.paths — 자산 경로 해석

- `models_root()` — URDF/메쉬 루트. `WHATSLAB_MODELS_ROOT` > `dexhand_description` 패키지 share(lazy import).
- `configs_root()` — rig/robot config 루트. `WHATSLAB_CONFIGS_ROOT` > 동봉 `whatslab/configs`.
