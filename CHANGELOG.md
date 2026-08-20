# Changelog

이 저장소는 [Semantic Versioning](https://semver.org/lang/ko/) 을 따른다.
1.0 이전이므로 minor 버전에서 호환 없는 변경이 있을 수 있다.

## [0.3.0] — 2026-08-20

### 호환 없는 변경 — 저장소에서 빠진 것

- **`CLAUDE.md` 와 `tools/bench_arm_ik.py` 는 더 이상 저장소에 포함하지 않는다**
  (`.gitignore`). 팔 IK 판정 방법론은 `docs/GUIDE.md` 에 서술로 남겼다.
- **`hand_configs/` 패키지와 `CONFIG_REGISTRY` 제거.** dex/kp 시절 손별 설정
  등록부였고 리타게팅은 `uni_tables.npz` 를 쓴다. 남아 있던 기능(URDF 경로 해석,
  센서 프레임 사슬 유도)의 유일한 실사용처였던 실물 orca 관절 매핑은
  `examples/robot_io.py:_finger_links` 가 URDF 에서 직접 유도한다.
- **`core.JOINT_INDEX` 제거.** `HumanHandFK.positions` 와 함께 소비자가 없어졌다.
  `HUMAN_HAND`·`SENSED_JOINTS` 는 남는다.
- **`HandRetargetController(backend=…)` 인자 제거**, `HandSolverCfg` 는
  `onnx_path`/`tables_path`/`threads` 만 받는다. rig 에 `hand_solver.backend` 가
  남아 있으면 조용히 무시하지 않고 에러를 낸다.
- **viz 의 손 클래스를 `HandViz` 하나로 합쳤다.** `_UrdfHandViz` 상속과
  `RobotHandViz`/`HumanHandViz` 가 사라졌다 — 루트 자세는 호출자가
  `upright_root`/`human_upright_root` 로 만들어 `root_pose` 로 넘긴다.
  `HandSkeletonViz` 는 없는 `_bone_pairs()` 를 부르던 깨진 클래스라 제거.
- **`SafetyFilter.holding` 제거** (`estopped or not enabled` 의 중복).

### 수정

- **오른손이 전혀 움직이지 않던 문제.** `UniRetargeter` 가 사람 관절 이름을
  `human:joints`(= 왼손 목록)로만 인덱싱하고 별칭도 `left_` 하나만 벗겨서,
  오른손 입력이 전부 미스 → `q_human` 이 0 벡터로 남고 ONNX 출력이 상수였다
  (실측: 오른손 이름적중 0/12, `|Δq|` 0.00rad). 표에 있는 `human:{side}:joints`
  를 쓰고 별칭은 양쪽 접두사를 벗긴다.
- **글러브 손목 quat 변환 3단계 제거**(`unpack_wrist` → `spine_lh_xyzw` →
  `wrist_to_canonical`). 프로토콜 문서에서 유도한 값이고 실기 검증이 없었으며
  실제로 손목이 틀어졌다. 이제 4개 float 을 Spine 프레임 그대로 커밋하고, 프레임
  정렬이 필요하면 소비자(`teleop/models/glove.py`)가 한다. **`--arm controller`
  경로의 팔 EE 방위가 이전과 달라진다.**
- **팔 IK 널스페이스 투영자를 SVD rank 절단으로**(`solver.proj_rcond`). 감쇠
  유사역으로 만든 `N = I − J⁺J` 는 멱등이 아니라 널스페이스 항이 주태스크로 새어
  들어간다(`‖N²−N‖` 최대 0.38). 관절가중이 있으므로 가중 좌표계에서 투영하고
  되돌린다. `dtheta_max`·`k_limit`·`limit_margin` 을 rig `solver:` 로 노출.
- **`calibration.enabled` 가 캘리브를 통째로 게이트한다** — off 면 리시버 좌표를
  그대로 목표로 쓴다(스케일도 yaw `W` 도 없음). `quest_arm.py --no-calib` 로 A/B.
- **목표 위치는 캘리브 yaw 로 회전하지 않는다** (`target = scale·p`, `W` 는 회전
  에만). 위치를 `scale·(p0 + W·d)` 로 매핑했더니 `W = Rz(-yaw(G))` 의 `G` 가
  컨트롤러 회전이라 조작자의 "앞"과 무관해서, yaw 가 90° 어긋나면 앞으로 미는
  동작이 좌우 이동으로 나왔다. `p0 + (p − p0) = p` 이므로 `W` 없이는 앵커가 위치에
  영향이 없다 — 캡처는 진단용으로만 남는다.
- **실물 손 연결 실패 메시지**에 원인(`WHATSLAB_MODELS_ROOT` 미설정 → 동봉
  `dexhand_description` URDF 에 센서 프레임 0개)과 해결을 적었다. 리타게팅은 표를
  쓰므로 센서 프레임 없이도 돌아서, 텔레옵은 정상인데 실물 매핑만 죽는다.

### 추가

- **`examples/run_retarget.py --viz`** — 글러브 없이 합성 동작(curl→spread→thumb)
  을 흘려 사람 손 URDF 와 로봇 손을 나란히 띄운다. `--viz` 없으면 지연 측정
  (orca 0.13ms / 7.8kHz, CPU 1스레드).

### 기타

- 저장소 규칙대로 `src`·`tools`·`examples`·`tests` 의 주석·독스트링을 전부 제거
  (예외: `tools/` 4개 파일의 `argparse(description=__doc__)` 모듈 독스트링).
- `[viz]` extra 유지. `whatslab.solvers.hand` 는 pinocchio 를 import 하지 않는다 —
  손 추론은 onnxruntime 하나로 돈다.

### Changed
- 손 리타게팅 엔진을 **학습된 통합 ONNX 모델 하나**(`UniRetargeter`)로 교체.
  사람 FK·인코더·통합 헤드·관절범위 역정규화가 그래프(`assets/uni_all.onnx`) 안에
  있고, 로봇 의존부(관절 기술자·범위·사이드 부호)는 `assets/uni_tables.npz` 에서
  입력으로 들어간다 — 로봇 추가 시 그래프 불변, 표만 는다.
  CPU 1스레드 프레임당 약 1.1ms(900Hz+), 좌우·5개 손(orca/allegro/tesollo/
  robotis/human) 공용. `hand` extra 는 `onnxruntime`·`pin` 으로 축소
  (dex-retargeting / nlopt / torch 제거).

### Removed
- dex(dex_retargeting 2단계 IK)·kp(키포인트 IK) 백엔드 (`retargeter.py`,
  `kp_retargeter.py`), `KPHandViz`, `examples/glove_hand_verify.py`,
  `tools/bench_hand_retarget.py`. `HandRetargetController(backend=...)` 인자는
  하위호환으로 무시된다.

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
- **viz 가 손을 똑바로 세운다** (`viz.upright_root`, `RobotHandViz`/`HumanHandViz`/
  `KPHandViz` 의 `upright=True` 기본). 전에는 URDF 베이스 프레임을 그대로 그려서
  손이 기울어져 보였다 — 각 URDF 의 베이스 축이 제각각이다(사람·robotis 손가락 +z,
  orca +y). 이제 **팜 프레임을 정준으로 올려서** 그린다(팜 y=손가락 방향 → world +z,
  팜 x=너클선 → world +x). 확인: 중지끝 world z = +0.106(orca) / +0.100(robotis),
  팜은 그 아래(−0.070 / −0.124). 두 손이 같은 자세로 서므로 나란히 비교된다.
  `HumanHandViz(offset=…)` 로 사람 손을 옆으로 띄운다.
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
  - 기본 가중치는 **실기 글러브 데이터로 재조정했다**(`w_tip` 2 → 6,
    `w_pair` 6 → 1). 합성 데이터에서 튜닝한 옛 값은 쌍항/스냅이 지문을 끌어당겼다 —
    합성 핀치는 목표 접촉이 ~0 까지 가지만 **실기는 사람 손 모델 자체의 접촉 잔차가
    11~25mm**(Visualizer 캘리브 `fit.after_mm`)라 스냅이 없는 접촉을 만들려다
    지문만 망쳤다. 실측 핀치 4개(orca 왼손) 기준:

    | | 지문오차 | 접촉오차(목표 대비) |
    |---|---|---|
    | 옛 기본값 `w_tip=2,w_pair=6` | 29.9mm | −1.8mm |
    | 새 기본값 `w_tip=6,w_pair=1` | **10.1mm** | +2.7mm |
    | dex 백엔드 | 22.2mm | +17.9mm |

    robotis 도 같은 방향(지문 39.5 → 30.3mm, dex 는 53.2mm)이지만 도달성 한계
    (지문전용 하한 21.2mm)에 걸려 있다. 정지 입력에서는 단조 수렴한다
    (orca |Δq| 10회 1.7e-2 → 50회 3.1e-5 → 200회 4.7e-15).
  - **한계 회피 항 `k_limit`(기본 0.3)** 추가. 없으면 관절이 한계에 박힌다 — 실측
    핀치에서 orca 10/64, **robotis 48/80(60%)** 이 한계 2% 안에 붙어 있었다(팔 IK 에서
    이미 겪은 실패 모드다). 한계에서 `LIMIT_MARGIN`(0.10rad) 안쪽으로만 밀어낸다.

    | | 지문 | 접촉오차 | 포화 | \|dq\|p95 |
    |---|---|---|---|---|
    | orca `k_limit=0` | 10.1mm | +2.7mm | 10/64 | 0.109 |
    | orca `k_limit=0.3` | **7.6mm** | **+0.5mm** | 4/64 | 0.176 |
    | robotis `k_limit=0` | 30.3mm | +27.1mm | 48/80 | 0.442 |
    | robotis `k_limit=0.3` | **29.1mm** | **+10.8mm** | 33/80 | 0.245 |

  - **엄지쌍 상대벡터·램프 스냅·손가락간 분리 제거**(`w_pair`/`w_snap`/`w_sep`).
    지문을 끌어당기는 대가가 컸다 — 제거 시 orca 지문 7.6 → 6.3mm, robotis
    29.1 → 23.2mm. 대가는 접촉오차(orca +0.5 → +6.7mm, robotis +10.8 → +30.8mm)이고,
    접촉은 목표 자체가 사람 손 모델 잔차만큼 벌어져 있어 지문 추종을 택했다.
    `contact_pairs()` / `target_contact_pairs()` 는 진단 지표로 남는다.
  - **dex-retargeting vector 방법론 2건은 측정으로 기각.** ① 사슬에 **팜→너클**
    벡터를 넣는 것(dex stage1 이 하는 것): 효과 **정확히 0** — 사람 너클은 팜 고정이라
    그 방향이 손가락 움직임과 무관한 상수이고, 중립 OFF 보정 후 잔차가 항상 0 이다
    (dex 에서는 사실상 외전을 중립에 묶는 정규화로만 작동한다). ② **벡터 단계 →
    위치 단계 순차 실행**: 지문 6.3 → 6.6mm(악화), 접촉 동일, `|dq|`p95 만
    0.166 → 0.138(15% 개선, 비용 +1.1ms). 웜스타트가 프레임 간 분기를 유지하므로
    벡터 단계의 기저 선택 가치는 콜드 스타트에만 있고, 그건 `_KP_COLD_SHAPE` 가
    이미 한다. 남은 dex 아이디어는 **미터 스케일 벡터 형상 항**(`w_shape`)뿐이고
    그건 처음부터 들어가 있다.
  - **시간 평활 `k_smooth`(기본 0.25)** — ByteDexter/MANUS 가 쓰는 `λ‖q − q_prev‖²`
    를 목적함수에 넣었다. 하드 스텝 캡보다 원칙적이고, **|dq|p95 를 절반으로 줄이면서
    형상도 개선**한다(orca 0.160 → 0.077 / 형상 15.5 → 14.4°, robotis 0.241 → 0.081 /
    13.6 → 12.0°). 0.5 이상은 손이 지연된다(실측: 1.0 에서 정지 입력 총이동
    0.102 → 0.005 rad 로 얼어붙는다).
  - **`tools/bench_hand_retarget.py`** — 목표 구성과 무관한 물리 지표로 백엔드를
    비교하는 하네스. 글러브 캘리브 덤프의 실측 핀치 θ 를 0→θ 로 보간해 궤적을 만들고,
    **뼈 방향각**(스케일·이동 무관) / **엄지-손가락 거리 오차** / **손가락간 거리 오차** /
    `|dq|`p95 / 비용을 낸다. 이 세션에서 잘못된 개선을 두 번 주장했다가 철회한 원인이
    전부 "한 방법의 목표를 정답으로 삼아 다른 방법을 재는" 것이었다 — **목표 기반
    지표로 백엔드를 비교하지 말 것.**

    ```
    side=right, 핀치 4개 × 20프레임          형상°  벌림mm  핀치접촉  펴짐접촉  |dq|p95   ms
    orca_hand       kp                      14.4   1.7     4.5     2.0    0.077   2.96
    orca_hand       dex                     12.1   4.7    14.0    33.5    0.145   5.95
    robotis_hx5_d20 kp                      12.0   3.0    33.4    11.3    0.081   1.99
    robotis_hx5_d20 dex                     16.4   6.8    27.3    27.2    0.196   6.10
    ```

    dex 는 orca 에서 뼈 방향각만 앞서고(12.1 vs 14.4°), 공간 충실도는 크게 뒤진다 —
    손가락별 `_SCALE_FACTOR` 가 손가락 간 거리를 33mm 왜곡한다. kp 가 2~3배 빠르고
    2배 부드럽다.
  - **팜 프레임 y 축을 dorsum 이 아니라 중립 평균 손가락 방향에서 잡는다**
    (`palm_frame_from_fingers`). dorsum·손목·베이스 어느 링크를 써도 **너클까지의
    기준선이 손마다 달라진다** — 실측 사람 dorsum 46.5mm / 사람 wrist 95.0mm /
    orca 70.5mm / robotis 125.4mm(두 로봇은 dorsum 이 베이스 링크와 같은 위치다).
    각 URDF 가 베이스를 하드웨어 사정대로 두기 때문이고, 그러면 y 축이 손마다 다르게
    기울어진다. 중립 손가락 방향은 모든 손에서 같은 의미이고 조건수도 90mm 급이며,
    팜 고정이라 매 프레임 재계산이 필요없다.
  - **robotis `_KP_THUMB_OFFSET` 1.0 → 0.0**(기본값). 1.0 은 낡은 목표 기반 지표로
    정한 값이었고, 프레임 무관 지표로 재니 낮을수록 좋다(핀치접촉 1.0 에서 33.4mm,
    0.0 에서 14.3mm — 단조).

    이 둘을 합친 robotis 개선: **핀치접촉 33.4 → 8.0mm**, 벌림 3.0 → 2.3mm,
    `|dq|` 0.081 → 0.072.

    | | 형상° | 벌림mm | 핀치접촉 | 펴짐접촉 | \|dq\|p95 | ms |
    |---|---|---|---|---|---|---|
    | orca kp | 18.3 | 2.0 | **4.3** | **0.7** | **0.074** | **2.4** |
    | orca dex | **12.1** | 4.7 | 14.0 | 33.5 | 0.145 | 5.4 |
    | robotis kp | **15.8** | **2.3** | **8.0** | **8.9** | **0.072** | **1.9** |
    | robotis dex | 16.4 | 6.8 | 27.3 | 27.2 | 0.196 | 6.0 |

    **형상(뼈 각도)은 프레임 의존 지표라 프레임 정의를 판정할 수 없다** — 각을 비교하려면
    공통 프레임이 필요하므로 측정 프레임을 고정하면 다른 프레임에서 최적화한 쪽이
    불리하게 나온다. 프레임 정의는 **거리 지표(접촉·벌림)** 로만 판정한다. 그 기준으로
    손가락 방향 축이 두 손 모두 낫다.
  - **팜 프레임 축별(비등방) 스케일도 기각.** 손의 폭/길이/두께 비율이 로봇마다 다르다
    (실측, 팜 프레임 x=너클선·y=손가락방향·z=손바닥법선의 로봇/사람 범위 비):
    orca `[1.026, 1.044, 1.150]` — 거의 등방. robotis `[1.457, 1.424, 0.427]` —
    **1.45배 넓고 길지만 두께는 0.43배인 납작한 손.** 축별로 맞추면 접촉은 좋아지는데
    형상이 무너진다:

    | | 형상° | 벌림mm | 핀치접촉 | 펴짐접촉 | \|dq\|p95 |
    |---|---|---|---|---|---|
    | robotis 균일 | **12.0** | **3.0** | 33.4 | 11.3 | **0.081** |
    | robotis z만 | 15.9 | 7.0 | **24.5** | **8.2** | 0.193 |
    | robotis 전축 | 17.8 | 5.6 | 41.5 | 22.3 | 0.138 |
    | orca 균일 | **14.4** | **1.7** | 4.5 | 2.0 | 0.077 |
    | orca z만 | **14.4** | 1.8 | **4.1** | **1.6** | **0.076** |

    이유는 **비등방 스케일이 상사변환이 아니라 각을 보존하지 않는다**는 것이다 —
    z 를 0.43 으로 누르고 x·y 를 1.45 로 늘리면 축에 정렬되지 않은 모든 뼈가 전단되고,
    뼈 방향은 정확히 형상 항이 최적화하는 대상이라 목적함수가 자기 목표와 싸운다.
    접촉이 좋아지는 건 목표가 로봇의 납작한 봉투 안으로 눌려 도달 가능해지기 때문일
    뿐이다. 거의 등방인 orca 는 z 만 4% 조정하면 미세 이득이 있지만(접촉 0.4mm)
    손잡이 하나를 유지할 값이 아니다. **납작한 손의 불일치는 목표를 왜곡해 흡수하지
    말고 솔버(또는 하드웨어 설계)에 맡긴다.**
  - **DexPilot/ByteDexter 의 나머지 기법 2건은 측정으로 기각.** ① **지문→자기 MCP
    기준**(ByteDexter 가 비인간형 팜 때문에 채택): 형상은 14.4 → 13.5° 로 좋아지지만
    핀치접촉 4.5 → 11.1, 펴짐접촉 2.0 → 9.1mm 로 공간 충실도가 무너진다. ByteDexter 는
    팜 기준을 버리는 대신 **손가락간 키벡터로 공간 정보를 대체**하는데 우리는 그 항이
    없으므로 대체물이 없다 — **MCP 기준은 손가락간 항과 짝일 때만 성립한다.**
    ② **DexPilot 스위칭 키벡터**(w=1/200/400, f=β·d/η₁/η₂)를 얹어봤지만 기여가 거의
    없다(orca 핀치 4.5 → 4.3, robotis 33.4 → 32.4). **우리 팜상대 절대 목표가 이미
    손가락간 거리를 담고 있어 상대 벡터가 중복**이기 때문이다 — DexPilot 이 키벡터를
    쓰는 이유는 절대 기준이 없어서다. (구현 중 β=1.6 을 그대로 쓰면 목표가 이중
    스케일된다는 것도 확인했다: 목표 162mm vs 실제 102mm.)
  - **`q_from_named` 가 맞지 않는 관절명을 조용히 버리지 않는다.** 하나도 안 맞으면
    raise, 일부만 안 맞으면 1회 경고한다. side/프로파일이 어긋났을 때 **사람 손이
    중립으로 고정된 채 아무 에러도 없이** 도는 사고가 실제로 났다(왼손 프로파일 각도를
    오른손 URDF 에 넣어 21개 전부 무시됨 → 벤치마크가 전부 무의미했다).
  - **엄지 기저 오프셋 `thumb_offset`**(손별 상수 `_KP_THUMB_OFFSET`) — 엄지 목표
    사슬만 강체 평행이동해 로봇 엄지 기저에 맞춘다(스케일은 균일 유지, 이동만 보정이라
    엄지 형상·상대운동은 보존된다). 엄지는 대립지라 절대 위치가 파지 기능 그 자체이고,
    실측 어긋남이 손가락 4개(2.5~17mm)와 자릿수가 다르다(orca 33.6mm, robotis 66.0mm).

    | | 지문 전체 | 엄지만 | 형상 | 접촉오차 |
    |---|---|---|---|---|
    | robotis 0.0 | 23.2mm | 45.6mm | 34.6° | +30.8mm |
    | robotis **1.0** | **16.3mm** | **11.0mm** | **34.0°** | **−3.3mm** |
    | orca 0.0 | **6.3mm** | **14.4mm** | **33.3°** | +6.7mm |
    | orca 1.0 | 8.7mm | 26.4mm | 36.3° | +1.7mm |

    **손별로 답이 반대인 이유는 구조가 다르게 어긋나 있어서다** — robotis 엄지는
    위치가 66mm 틀렸을 뿐 길이 비율은 0.98 이라 오프셋이 네 지표를 모두 고친다.
    orca 엄지는 길이 비율이 **0.71**(로봇 80mm vs 목표 113mm)이라, 목표를 로봇 기저로
    옮기면 사거리 밖으로 나가 오히려 나빠진다. 그래서 robotis 는 1.0, orca 는 0.0 이다.
    orca 의 엄지는 길이 문제이므로 오프셋으로는 못 고친다.
  - **손가락별 스케일은 채택하지 않았다.** 스케일 무관 지표로는 개선으로 보였지만
    (orca 자기목표 지문 6.3 → 3.8mm, 형상 33.3 → 32.2°) 손가락마다 다른 배율은 손
    실루엣을 사람과 다르게 만든다. 균일 스케일이 기본이다.
  - `anchor_base` / `palm_scaled` / `_shape_targets` 제거. 각 손가락을 자기 너클에
    고정해 전방 체이닝하는 실험이었는데, 형상 항은 세그먼트 **차분**을 로봇 뼈 길이로
    재정규화하므로 두 목표 집합에서 수식이 같다(= 형상에 영향 0). 지문 항에만 영향이
    있었고 공통 기준으로 재보면 개선이 아니었다.
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
