# Changelog

이 저장소는 [Semantic Versioning](https://semver.org/lang/ko/) 을 따른다.
1.0 이전이므로 minor 버전에서 호환 없는 변경이 있을 수 있다.

## [Unreleased]

### 호환 없는 변경 — 사람 손 FK 를 pinocchio FK 하나로 통일

글러브(Spine)가 사람 손도 로봇 손과 똑같이 `/{side}/joint_angles/get` 으로 URDF
관절각을 보낸다. 차이는 **로봇은 받은 값을 그대로 쓰고**(`InputSample.joint_q`
패스스루), **사람은 그 q 를 URDF 로 FK 해서 IK 를 돈다**는 것뿐이다.

- **`solvers/hand/spherical_fk.py` 제거** (`HandSphericalFK`·`build_model`·
  `HandRerunViz`). quat 17슬롯을 볼조인트 모델에 밀어넣던 경로였고, q 가 오는
  지금은 URDF 를 그대로 FK 하면 된다. 딸려 있던 `_correct_quat`·`sensor_to_jid`·
  `PINKY_CMC_OFFSET`·`AGA_SKIP_JOINT` 보정도 사라졌다.
- **`HumanHandFK`**(`solvers/hand/human_fk.py`)가 사람 FK 의 단일 구현이고
  **dex·kp 두 엔진이 공유**한다. `points(angles)` / `positions(angles)`.
- **두 엔진의 `compute()` 입력이 관절각 dict 로 바뀌었다**(전에는 (17,4) quat 배열).
  `HandRetargetController.compute()` 는 `sample.hand.joint_angles` 를 쓴다.
- **`HandPose.joint_angles`** 추가(이름→rad). `joint_rot`(quat)은 전송 계층에 남는다.
- **`GloveHumanAnglesReceiver`** 추가 — `/{side}/joint_angles/get` + `/{side}/wrist/get`.
  `HandModel`·`GloveModel` 의 기본 손 소스다. `GloveModel(robot, hand_source=…)` =
  `angles`(사람) / `robot`(패스스루).
- **`send_haptic` 이 `GloveReceiverBase` 로 올라갔다** — 어느 리시버를 쓰든 햅틱이 된다.
- **Quest 핸드트래킹 리타게팅은 지원하지 않는다.** 컨트롤러 → 팔 IK 경로는 그대로다.
- **URDF 참조를 관절명에서 링크명으로 바꿨다.** Visualizer 규칙이 "링크 = 뼈 이름,
  관절 = 운동 이름"이고 관절명은 개명된 이력이 있다(`thumb_cmc0`→`thumb_cmc_flex`).
  사람 손 손끝은 `{side}_sensor_{finger}_distal`(실제 센서 장착점) →
  `{side}_{finger}_tip` 순으로 찾는다.

### 수정 — dex 가 스켈레톤을 못 따라가던 문제 (프레임 불일치)

`_COORD_TRANSFORM`(config 의 손으로 튜닝한 3x3)은 **옛 구면 FK 프레임**(뼈가 +x)에
맞춰져 있었다. 새 `HumanHandFK` 는 URDF 프레임(손가락 +z, 너클 y)이라 회전이 어긋나
사람 키포인트가 로봇 공간에서 돌아간 채 놓였다.

- 이제 **팜 프레임에서 유도한다** — `_coord_transform = r_frame @ h_frame.T`,
  센터링도 손목이 아니라 팜 원점 기준. kp 백엔드와 같은 정렬이다.
- `_COORD_TRANSFORM` / `get_coord_transform` / `get_tf_coord_transform` 제거.
- 실측 핀치(글러브 캘리브 덤프) 지문 추종:

  | | 하드코딩 상수 | 유도(팜 프레임) |
  |---|---|---|
  | orca | 45.8~71.4mm | **14.5~23.3mm** |
  | robotis | 106~131mm | **35.5~47.4mm** |

  엄지-손가락 접촉도 orca 47~83 → 32~50mm, robotis 72~89 → 45~55mm.

### 수정 — 팜 프레임 y 축이 특이했다 (사람↔로봇 매핑 90° 회전)

`_palm_frame` 의 y 축을 `중지너클 − 너클평균`(너클 아치 볼록량)으로 잡았는데 그게
4~7mm 뿐이고 대부분 너클선 성분이라, 직교화하면 거의 남지 않았다. 사람 손에서는
방향까지 뒤집혀 y 가 손가락 방향이 아니라 **손바닥 법선**이 됐다.

- y 축을 `너클평균 − 팜기준점`으로 바꿨다. 팜기준점은 `{side}_sensor_dorsum` →
  손가락 공통 조상 → 베이스 링크 순. 조건수 3.9 → 46~130mm 이고, 세 손 모두 팜
  프레임에서 중지가 +y 를 향한다(사람 0.93 / orca 0.99 / robotis 0.93).
- 실측 핀치(글러브 캘리브 덤프) 기준 orca 검지 지문오차 **51.1 → 22.4mm**,
  robotis 약지 **109.2mm(발산) → 35.8mm**.
- `robotis_hx5_d20` 의 `_WRIST_LINK["left"]` 가 존재하지 않는 링크명
  (`robotis_hx5_d20_left`)이어서 왼손 dex 리타게팅이 아예 안 되던 버그도 고쳤다
  (`hx5_d20_left_base`).

남은 오차는 두 손의 원인이 다르다(지문전용 다중시드 하한과 비교): **orca 는 솔버
문제**(하한 4.1mm 인데 22.4mm — 목적함수 가중치가 지렛대), **robotis 는 rig 문제**
(하한 21.2mm 로 현행이 이미 하한 — 목표가 도달 불가). 각 손가락을 자기 너클에
고정해 전방 체이닝하는 안(`anchor_base=True`)은 공통 기준으로 재보면 orca 38.5 →
43.3mm 로 악화라 기본값을 껐다.

### 호환 없는 변경 — 손 config 를 URDF 에서 유도

`_FINGERS` 하드코딩 링크 테이블(7개 손, 521줄)을 지웠다. URDF 의
`{side}_sensor_{finger}_distal` 에서 손가락 사슬·팁·팜 링크를 유도하고, config 에는
URDF 로 알 수 없는 **사람-관절 짝짓기(`_HUMAN_CHAIN`)** 만 남는다 — 로봇 관절 수가
사람과 다를 때 어느 사람 관절을 공유할지는 손별 판단이라 유도할 수 없다.

- 제거: `_FINGERS`, `_COORD_TRANSFORM`, `_WRIST_LINK`, `_SIDE_MAP`, `_RVIZ_FILENAME`,
  `_LINK_FALLBACK`. 팜 링크는 손가락 공통 조상에서 유도한다.
- 짝짓기 길이가 URDF 사슬과 안 맞으면 **유도된 사슬을 그대로 보여주는** 에러를 낸다.
- `robotis` 검지 사슬이 `link5 → link7` 로 `link6` 을 건너뛰던 것을 유도가 바로잡고,
  `_WRIST_LINK["left"]` 가 없는 링크명이라 왼손 dex 가 아예 안 되던 버그도 사라졌다.
- **센서 프레임이 없는 URDF 는 명확한 에러**를 낸다(테스트는 skip). 동봉
  `dexhand-description` 이 센서 프레임을 실을 때까지 그 손들은 못 쓴다.

### 추가

- **`whatslab.viz.HumanHandViz`** / `RobotHandViz` 를 URDF 메쉬 기반으로 교체.
  `RobotHandViz` 가 dex 내부(`_seq_stage1`) 의존을 버려 두 백엔드 다 쓴다. 사람 손은
  `engine.human_to_robot()` 로 로봇 프레임에 올려 그린다 — 이 변환 없이는 사람은
  손가락이 +z, orca 는 +y 라 90° 어긋나 보인다.
- **`examples/glove_hand_verify.py`** — 실기 글러브 검증(프레임별 접촉·지문오차·|dq|
  기록, `--viz` 로 목표/달성 키포인트 오버레이).

- **`KPHandRetargeter`** (`solvers/hand/kp_retargeter.py`) — 손 리타게팅의 `kp`
  백엔드. `HandRetargetController(..., backend="kp")` 로 선택한다(기본값은 기존
  `dex`). dex-retargeting·nlopt·torch 없이 pinocchio + numpy 만 쓴다.
  - 정렬: 해부학적 팜 프레임(MCP 4점) + 손길이 비율 균일 스케일 + 중립 자세 1회
    구간별 방향 보정 — 전부 URDF 에서 자동 유도, 튜닝 상수 0. 5 손가락 체인을
    자동 추출하지 못하는 손은 `keypoints` 를 명시한다.
  - 목적함수(가중 DLS + IRLS Huber, warm start 틱당 8반복, ~2ms/손): 팜상대
    지문 위치 + 미터벡터 형상 + 엄지쌍 상대벡터 **램프 스냅**(DexPilot 의
    projection 을 연속 보간으로 — 사람 핀치 30mm 이하에서 목표를 접촉으로 강제)
    + 손가락간 최소분리 30mm.
  - 합성 동작 검증(시작점 3개 × 핀치 2 + 쥐기 2, orca/robotis): 핀치 접촉
    29~44mm → 6~12mm(달성 가능 하한 도달, orca 검지만 ~6mm 갭), 지문 대가
    +1~3mm, 자유 동작 영향 0. **실기 글러브 검증은 아직 없다.**
  - 손별 파라미터는 `HandConfig` ClassVar: `_KP_SHAPE_WEIGHT`(orca 2.0,
    robotis 0.5, 기본 1.0), `_KP_COLD_SHAPE`(orca 만 true — 콜드 스타트에서
    형상 전용 solve 로 분기 선택).

## [0.2.0] — 2026-08-07

층 이름을 실제 내용에 맞추고, side 간 상태 공유로 생긴 텔레옵 정확도 문제를
구조적으로 제거한 릴리스. 실기 추종 오차 **332mm → 12mm**.

### 호환 없는 변경

- **`TeleopModel` 의 side별 상태가 `sides: Dict[str, SideModel]` 하나로 합쳐졌다.**
  기존 병렬 딕셔너리 8개(`robots`·`ik`·`retarget`·`calib`·`target`·`raw_target`·`q`
  + 내부 safety)를 제거했다. `SideModel` 은 그 side 의 `robot`·`ik`·`retarget`·
  `calib`·`safety` + 이 틱의 `raw_target`·`target`·`q` 를 갖는다.

  ```python
  model.ik["right"]            → model.sides["right"].ik
  model.calib["right"]         → model.sides["right"].calib
  model.robots["right"]        → model.sides["right"].robot
  model.target.get("right")    → model.sides["right"].target
  model.raw_target.get("right") → model.sides["right"].raw_target
  ```

  `model.q` 는 읽기 전용 property 로 남는다.

- **`RobotModel.solve()` 제거.** reach 스케일 사본이 들어 있었는데, 스케일은
  `teleop.ArmCalibration` 에서만 해야 한다. 실제 텔레옵 경로는 `RobotArmIK` 를
  쓴다 — `RobotArmIK(robot).solve(T_canonical)` 로 대체한다.
- **`RobotModel.from_yaml(path)` 제거.** `RobotModel(path)` 가 동일하다.
- **`whatslab.solvers.xyzquat_to_mat` 제거.** 호출자가 없었다.
- **`TeleopModel` 을 rig 경로/`RigConfig` 로 만들면 side 마다 `RobotModel` 을
  따로 만든다.** 전에는 같은 인스턴스를 공유했다. `RigConfig` 객체는 공유하므로
  캘리브 역기록은 양쪽에 반영된다. `RobotModel` 인스턴스를 직접 넘기면 이전처럼
  공유한다(복제할 수 없으므로).

### 수정 — side 간 상태 공유 (실기 오차의 주원인)

같은 rig 를 쓰는 두 side 는 관절 이름도 같다. 유상태 컴포넌트를 공유하면 두 side
가 서로를 밀어낸다. 세 곳에서 터졌다:

- **`SafetyFilter` 를 양쪽 side 가 공유**했다 — 필터의 `_last` 가 왼손 해와
  오른손 해 사이를 왕복하며 각 side 를 상대 side 의 직전값 기준으로 속도제한했다.
  실기 오차 221mm(오프라인 리플레이는 3.5mm). 이제 side 마다 `clone()` 한다.
- **`RobotModel`(= 유상태 솔버)을 공유**했다. side 마다 따로 만든다.
- **`RobotArmIK._cold_start` 가 시드를 명시하지 않아** 공유 솔버의 `history_data`
  (= 다른 side 가 남긴 자세)를 첫 후보로 썼다. 26.1 → 13.6mm.

`SafetyFilter.clone()` / `SafetyFilter.enabled` 가 추가됐다.

### 수정 — 여분 자유도가 관절 한계에 박히던 문제

- **`k_posture` 기본값 0.0 → 0.05**, **`k_limit`(구 `_k_limit`) 0.15 → 1.0.**
  자세를 중앙으로 되돌리는 힘이 없어서 여분 자유도가 널스페이스에서 표류하다
  한계에 박혔다. joint5 가 가동범위 ±157.6° 로 가장 넓은데도 하한에 45% 프레임
  붙어 있었다(같은 프레임 전역탐색은 포화 0% / 3.1mm, 추종은 54.0mm).
- **`q_neutral` 이 `pin.neutral`(전부 0) → 관절범위 중앙.** joint4 `[-57.9°,
  122.6°]`, carpal `[-65°, 35°]` 처럼 비대칭 범위에서 0 은 중앙이 아니다.
- 실측(실기 녹화 run6 8747프레임 + run5 4058프레임, 시작점 3개 평균):
  21.9mm / 13.3° / 포화 13.6% → **12.5mm / 4.8° / 4.9%**. 시작점 편차가
  ±7.4 → ±0.3 으로 붕괴해, 더는 어느 기저에서 출발했는지에 좌우되지 않는다.

### 추가

- **`joint_weights: {관절명: 비용}`** (rig `solver:`) — 가중 DLS 의 관절 비용 `W`.
  `dq = W⁻¹Jᵀ(JW⁻¹Jᵀ+λ²I)⁻¹e`. 싼 관절이 먼저 쓰이되 커플링을 유지하므로 싼
  관절이 한계에 걸리면 비싼 관절이 이어받는다. `joint1~4 = 2.5` 로 손목을
  우선한다(nero: 31.4 → 13.8mm).
- **`k_posture` / `k_limit`** (rig `solver:`) — 위 기본값을 rig 에서 튜닝.
- **`--sides {left,right,both}`** (`examples/quest_arm.py`) — IK 를 돌릴 side.
  **기본값이 `--side` 하나로 바뀌었다.** 한 팔 rig 에 반대쪽 컨트롤러를 물리면
  도달 불가 목표에 전역 탐색을 태우고 프레임 예산을 넘긴다.
- **`--dump-targets PATH`** — 프레임별 원시입력·목표·해·오차를 npz 로 기록.
  오프라인 리플레이로 설정 비교가 가능하다.
- **`docs/GUIDE.md`** — 새 로봇 올리기 / 캘리브 / IK 튜닝 / 진단 / 실물 전송.

### 수정 — 기타

- `SafetyFilter` 가 고정 dt 대신 **실측 dt** 를 쓴다. 60Hz 를 가정한 채 20Hz 로
  돌아 2.9배 과도 제한하고 있었다. `dt_max` 는 `--rate` 가 아니라 절대값 0.05s 다.
- 텔레옵 루프가 `sleep(period)` 대신 **남은 시간만** 잔다(작업 시간을 빼지 않았다).
- 실물 전송이 **별도 스레드**로 나갔다. CAN/dynamixel 쓰기가 루프를 막았다
  (50ms 블로킹 하드웨어에서 20.4 → 59.7Hz).
- orca 손이 `init_joints()` 를 부른다. 없으면 wrap offset 이 없어 `thumb_cmc`
  (비대칭 새들 관절)가 먼저 죽는다. 손목도 별도 경로가 아니라 매핑에 통합됐다.
- rig yaml 쓰기가 **원자적**이다(임시파일 + `os.replace`). 텔레옵 중에 다른
  프로세스가 잘린 yaml 을 읽던 문제. `save_calibration` 이 `enabled` 를 덮어쓰지
  않는다.
- `calibration.enabled` 가 텔레옵 경로에 실제로 배선됐고, **reach 스케일만**
  게이트한다 — `false` 여도 yaw 캘리브는 동작한다.
- 정지 목표에서 발산하던 문제 수정. 콜드 스타트에는 틱/이동 상한을 걸지 않는다
  (걸면 전역 해가 수 rad 떨어져 있을 때 두 분기 사이에 갇힌다).
- `RobotArmIK` 의 스톨 탈출 제거 — `|Δq|` 0.29 의 EE 순간이동 원인이었다.
  이제 첫 타깃 콜드 스타트 이후에는 백엔드 `solve()` 만 부른다.

### 제거

- **`DecoupledArmIK`**(`backend: decoupled`)와 `solver.orientation_joints`.
  위치·방위를 관절 블록으로 엄격 분리하는 백엔드였다. nero 는 joint5·6·7 축이
  0.0mm 로 교차하는 구형 손목이라 분리가 이론상 정확하지만, joint6 가동범위가
  `[-41.8°, 54.4°]` 뿐이라 팔의 방위 기여 경로가 끊긴다 — 66~72% 프레임에서
  joint6 이 포화하고 방위 오차가 18.3 → 38.1° 로 무너졌다. 손목 3축이 전부 넓은
  로봇에서만 성립하는 방법이다.

### 구조

- `teleop` ↔ `solvers` 이름 교환 — 층 이름을 실제 내용에 맞췄다. `solvers/` 는
  수치 해법만, `teleop/` 이 조립.
- `solvers/core` 제거 + `solvers/arm` 평탄화. `RobotArmIK` 는 `robot/` 으로,
  장치별 조립은 `teleop/models/` 로.
- **함수 안 import 제거.** 전부 모듈 최상단이다. 결과로 `import whatslab.teleop`
  이 pinocchio·dex_retargeting·torch·nlopt·python-osc 를 전부 끌어온다(약 0.9초) —
  extra 를 나눠 설치하는 소비자는 `[all]` 을 써야 한다.
- 파이썬 코드의 독스트링·주석 전부 제거(설명은 `docs/`·`CLAUDE.md`·rig yaml 로).

### 설정

- `nero_orca_right` 에 **`lock_joints: ["joint7"]`**. 실기 조작감을 우선한
  선택이고, 비용은 알고 있다: 방위 도달률 61% → 32%, 추종 14.5 → 22.3mm,
  방위 5.3 → 13.0°. 숫자만 보고 되돌리지 말 것(`CLAUDE.md` 에 근거 기록).
- Quest 앱 1.0.4 → **1.0.5**. `install_quest_app.sh` 는 알파벳순 첫 apk 를
  고르므로 이전 버전은 지워야 한다.
- **`receiver.quest.CONTROLLER_POS_OFFSET` z 0.08 → 0.1**(의도된 변경). 컨트롤러
  마운트 편차 보정으로 모든 컨트롤러 기반 팔 목표가 정준 z 로 2cm 이동한다.
  **Quest 앱(PoseDataTracker)의 같은 상수와 값이 일치해야 한다** — 저장소가 달라
  자동 검증이 안 되므로 앱을 올릴 때 함께 확인한다.

### 제거 (계속)

- **`examples/vive_smoke.py` 추적 해제.** Vive 트래커 스모크 테스트인데 `openvr`
  의존이 선언돼 있지 않고 문서·참조도 없다. 실수로 커밋된 것이라 `.gitignore` 로
  옮겼다(파일은 작업 트리에 남는다).

### 알려진 제약

- **글러브 경로는 Spine 2.3.1 이하만 지원**한다(Quest 경로는 Spine 무관).
- `GloveRobotHandReceiver` 의 `wrist` 프레임 변환(`unpack_wrist` →
  `spine_lh_xyzw` → `wrist_to_canonical`)은 Spine `docs/OSC_Protocol.md` 에서
  유도했고 **실기 검증은 아직 없다.**
- `max_joint_velocity: 5.0` 의 하드웨어 근거가 확인되지 않았다. IK 가 p99 에서
  0.163 rad/틱(≈9.6 rad/s)을 요구하는데 필터가 0.086 에서 자르므로 그만큼 밀린다.

## [0.1.1] — 2026-08

첫 공개 버전대. 상세 이력은 `git log` 참고.
