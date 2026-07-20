# whatslab API

공개 API 목록. import 루트는 `whatslab` (PEP 420 네임스페이스). 시그니처는 소스에서
추출한 것이며 `self` 는 생략한다. 개념·사용 흐름은 [README](../README.md) 참고.

의존 규칙: `receiver → core`, `model → core·robot`. `receiver` 는 `model` 을 import
하지 않는다. 컴포넌트를 엮는 조립은 소비자 몫이다.

---

## whatslab.model — 사용자 대면 최상위 텔레옵 API

```python
from whatslab.model import QuestModel, GloveModel
```

| 심볼 | 설명 |
|---|---|
| `TeleopModel(robot)` | 베이스 클래스. 소스 리시버 + IK + 리타게팅 + 캘리브를 조립해 `get_q()` 를 낸다. `robot` = rig yaml 경로(또는 `[left, right]`). 유저는 서브클래싱해 자기 하드웨어 조합을 정의. |
| `QuestModel(robot)` | 프리셋: Quest 핸드트래킹(손목→팔, 손가락→손). |
| `GloveModel(robot)` | 프리셋: 팔=Quest 컨트롤러 IK, 손=글러브 리타게팅. 햅틱 지원. |
| `HandModel(robot)` | 프리셋: 손 리타게팅 단독(팔 IK 없음). |
| `RobotArmIK(...)` | 팔 IK 컴포넌트. 정준 목표 4x4 → 팔 관절각. |
| `ArmCalibration(...)` | yaw 정렬 + reach 스케일 소유. |

### `TeleopModel` 메서드

| 메서드 | 반환 | 설명 |
|---|---|---|
| `start()` / `stop()` | — | 소스 리시버 수신 시작/정지(대상은 arm/hand source 에서 자동 도출). |
| `get_data()` | `Dict[str, dict]` | (오버라이드 지점) 소스에서 side별 값을 모아 역할 결정(arm_pose/fingers/q/hmd). |
| `solve(data)` | `Dict[str, Dict[str, float]]` | data → IK/리타게팅 → side별 `{joint: rad}`. |
| `get_q()` | `Dict[str, Dict[str, float]]` | 매 호출 get_data→calib→solve 를 엮어 `{side: {joint: rad}}` 반환. |
| `calibrate_yaw()` | `Dict[str, bool]` | 손목 yaw 정렬 스냅샷(즉시). side별 성공 여부. |
| `calibrate_reach(duration=8.0, rate_hz=60.0)` | `Dict[str, bool]` | duration 초 폴링해 최대 도달반경 측정→calib 등록(블로킹). |
| `set_reach(input_reach)` | `Dict[str, bool]` | reach 스케일 스칼라를 직접 주입. |
| `send_feedback(data)` | — | 역방향 피드백(기본 no-op; `GloveModel` 이 햅틱으로 오버라이드). |

## whatslab.receiver — 입력 소스 (텔레옵 무관, 단독 사용 가능)

```python
from whatslab.receiver.quest_controller import QuestControllerReceiver
```

`side` = 물리적 기기의 좌/우(채널 재해석 금지). 출력은 항상 정준 프레임
(x=앞, z=위, 오른손계). `python-osc` 는 `start()` 에서 lazy import.

| 클래스 | `get(side)` → | 설명 |
|---|---|---|
| `QuestControllerReceiver` | `InputSample(controller=Pose)` | Quest 컨트롤러 6D 위치/자세. `connected(side)`. |
| `QuestHandReceiver` | `InputSample(hand=HandPose)` | Quest 핸드트래킹(손목 6D + 손가락). `connected(side)`. |
| `GloveHumanHandReceiver` | `InputSample(hand=HandPose)` | AirGlove 손가락 회전. `send_haptic(side, values)`. |
| `GloveRobotHandReceiver` | `InputSample(hand=…, q=…)` | 로봇 관절각을 직접 주는 글러브(IK 바이패스). |

공통: `start()`, `stop()`, `get(side) -> InputSample`.

## whatslab.robot — 팔 기구학 모델 + config 로더

| 심볼 | 설명 |
|---|---|
| `RobotModel.from_yaml(path)` | rig/robot yaml → 모델. |
| `RobotModel.solve(T_canonical)` | 정준 목표 4x4 → 팔 관절각. |
| `RobotModel.ee_pose(q_arm)` | FK: 관절각 → EE 4x4. |
| `RobotModel.to_base(T)` / `to_canonical(T)` | 정준↔베이스 프레임 변환. |
| `RobotModel.sync_state(q_arm)` | IK 웜스타트용 현재 상태 갱신. |
| `RobotModel.make_hand_controller(config_name, side)` | 손 리타게팅 컨트롤러 생성. |
| `load_robot(path)` / `load_rig(path)` | yaml → `RobotSpec` / `RigConfig`. |
| `save_calibration(rig, input_reach)` / `save_reach_max(rig, reach_max)` | 캘리브 값을 rig yaml 에 기록. |

## whatslab.core — 계약(타입 + Protocol), 의존성 0

| 심볼 | 설명 |
|---|---|
| `types.Pose` | 위치 + quaternion(xyzw). |
| `types.HandPose` | 손목 6D + 관절명→회전(사람 손). `to_sensor_array()` 경계에서만 배열화. |
| `types.InputSample` | 리시버 출력 컨테이너(controller/hand/q/hmd). |
| `types.HandCommand` | 리타게팅 출력(로봇 손 관절각). |
| `types.JointSpec` | 관절 이름/한계 스펙. |
| `interfaces.Receiver` / `HandController` / `ArmSolver` | 컴포넌트 Protocol(구조적 타이핑). |

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
| `SafetyFilter` | clamp + rate-limit + hold/estop 상태기. `step(desired)`, `trip`, `reset`, `estopped`, `set_enabled`, `seed`, `holding`. |

## whatslab.paths — 자산 경로 해석

- `models_root()` — URDF/메쉬 루트. `WHATSLAB_MODELS_ROOT` > `dexhand_description` 패키지 share(lazy import).
- `configs_root()` — rig/robot config 루트. `WHATSLAB_CONFIGS_ROOT` > 동봉 `whatslab/configs`.
