# whatslab-collect — 사람-로봇 상호작용 데이터 수집 앱 설계

작성 2026-08-21. 대상: `~/Desktop/whatslab-collect/` (신규) + `whatslab-sdk` 의 `whatslab.data` 보강.

## 1. 목적과 범위

오른팔 nero + 오른손 orca(촉각형) 을 텔레옵하면서, **상대방 사람과의 상호작용**을
LeRobot v2.1 데이터셋으로 수집한다. 산출물은 `tools/export_targets.py` 로
GR00T(`--groot`) / lerobot v3.0·π0.5(`--v30`) 로 파생된다.

범위에 **포함**: 텔레옵 구동, 로봇 실측·명령 기록, 카메라 3대, 촉각(합력+택셀),
상대방 신호(왼손 관절 q · 오른 컨트롤러 pose · HMD 회전), 에피소드 운용, SDK 리코더 보강.

범위에 **제외**: 정책 학습, depth 저장, Quest 핸드트래킹 리타게팅(SDK 미지원),
상대방 1인칭 영상(Quest OSC 경로에 없음).

## 2. 장치 매핑 (확정)

| 역할 | 장치 | 데이터 |
|---|---|---|
| 조작자 오른손 | 글러브 right | 로봇 손 리타게팅 입력 |
| 조작자 왼손 | Quest **왼쪽** 컨트롤러 | 로봇 팔 EE 목표 |
| 상대방 왼손 | 글러브 left | `peer_hand_q` (사람 관절 rad) |
| 상대방 오른손 | Quest **오른쪽** 컨트롤러 | `peer_ctrl` (pos+quat, 정준 프레임) |
| 상대방 머리 | Quest HMD | `peer_hmd_quat` (회전만 — 위치 신호 없음) |

이 매핑은 SDK 가 **이미** 하는 것이다: `teleop/models/glove.py:41` 의 `_OPPOSITE[s]`
때문에 `GloveModel` 은 로봇 side `right` 의 팔 목표를 왼쪽 컨트롤러에서, 손가락을
오른손 글러브에서 가져온다. 새 리시버도 새 `TeleopModel` 서브클래스도 필요 없다.

상대방 신호는 같은 스트림의 **남는 side** 를 그대로 읽는다:
- `model.hand_source.get("left").hand.joint_angles`
- `model.arm_source.get("right").controller`
- `model.arm_source.get("right").hmd`

OSC 서버는 포트별 싱글턴(`osc_transport._registry`)이라 포트 충돌·중복 수신이 없다.

**`--sides right` 고정.** 상대방 오른쪽 컨트롤러가 켜져 있으면 `GloveModel` 이 side
`left` 의 raw_target 을 그것으로 만든다 — 왼쪽 로봇을 끄지 않으면 도달 불가 목표에
전역탐색이 붙어 프레임 예산을 먹는다(CLAUDE.md 의 실측 문제).

## 3. 관절 구성 (실측으로 확인)

- `RobotModel('rigs/nero_orca_right.yaml').arm_joint_names` = `joint1..joint6` + carpal 1개 = **7**
  (`joint7` 은 rig `lock_joints` 로 잠겨 목록에 없다)
- carpal 은 **orca 하드웨어의 `wrist` 관절**이다 — nero 가 아니라 손으로 명령이 간다
  (`examples/robot_io.py:251` 의 `connect_hand` 가 같은 방식으로 매핑)
- nero 로 가는 관절 = `joint1..joint6` = **6**
- `UniRetargeter('right','orca_hand').joint_names` = 손가락 **16**
- orca `config.yaml:34` 의 `joint_ids` = `wrist` + 손가락 16 = **17**

따라서 상태·행동 벡터는 **6 + 17 = 23**.

## 4. 저장 포맷

LeRobot v2.1, `whatslab.data.LeRobotRecorder` 사용. 출력은
`~/Desktop/whatslab-collect/data/<task_slug>/` 하나(에피소드가 그 안의 parquet/mp4).

### 컬럼

| 키 | 폭 | 내용 |
|---|---|---|
| `observation.state` | 23 | 실측: nero 6 + orca 17, **rad**, orca 는 `joint_ids` 순서 |
| `action` | 23 | 명령: 같은 순서·같은 단위 |
| `observation.images.wrist` | 640×480×3 | RealSense 컬러 (mp4) |
| `observation.images.third_0` | 640×480×3 | UVC 웹캠 (mp4) |
| `observation.images.third_1` | 640×480×3 | UVC 웹캠 (mp4) |
| `replay.tactile_force` | 15 | 손가락 5 × (fx,fy,fz) |
| `replay.tactile_taxel` | 1089 | 택셀 363 × 3 — 아래 레이아웃 참조 |
| `replay.peer_hand_q` | H | 상대방 왼손 사람 관절 rad (H = 글러브 프로파일 길이) |
| `replay.peer_ctrl` | 7 | pos3 + quat4, 정준 프레임 |
| `replay.peer_hmd_quat` | 4 | |
| `replay.operator_ctrl` | 7 | 조작자 왼쪽 컨트롤러 |
| `replay.operator_target` | 16 | IK 입력 목표 4x4 평탄화 (재현·디버깅) |
| `replay.state_valid` | 2 | [arm, hand] 실측 읽기 성공 |
| `replay.peer_valid` | 3 | [hand_q, ctrl, hmd] |
| `replay.t_mono` | 1 | 기록 틱의 monotonic 시각 |
| `replay.cam_t` | 3 | 카메라별 프레임 실제 수신 시각 |
| `replay.tactile_t` | 1 | 촉각 프레임 자체 timestamp |

단위·순서 규약:
- 모든 관절값 **rad**. orca 하드웨어는 degree 로 읽고 쓰므로 경계에서만 변환한다.
- `action` 은 SDK q(관절이름→rad)를 `observation.state` 와 **같은 인덱스 순서**로
  정렬해 넣는다. 순서가 다르면 state−action 차이가 무의미해진다.
- 결측은 **NaN 을 쓰지 않는다** — 직전값 유지 + `*_valid=0`. `save_episode` 가
  min/max/mean/std 통계를 내므로 NaN 하나가 컬럼 정규화를 망친다.

### 택셀 평탄화 레이아웃

택셀 수가 손가락마다 다르다(`sensing/constants.py:22`):
`thumb 51, index 87, middle 87, ring 87, pinky 51` = 363.
평탄화 순서는 **`thumb, index, middle, ring, pinky` 고정**, 손가락 안에서는 택셀
인덱스 순, 택셀 안에서는 `fx, fy, fz`. 이 레이아웃을 `modality.json` 과 앱 README 에
명시한다(하드웨어 원본은 int8/int8/uint8 양자화값이지만 parquet 컬럼이 float32 라
그대로 float32 로 올린다 — 프레임당 4.4KB).

## 5. SDK 보강 (`whatslab.data`)

코드를 바꿔야 하는 것은 **2건**(5.1 스트리밍 라이터, 5.4 `discard_episode`)이고,
5.2·5.3 은 리코더 변경 없이 features 구성으로 해결된다.

### 5.1 프레임 버퍼 → mp4 스트리밍 append

지금은 `self._buf` 가 원시 uint8 프레임을 `save_episode()` 까지 RAM 에 보관한다
(`lerobot_recorder.py:38`). 3대 × 640×480×3 × 30Hz = **초당 26MB** → 60초 에피소드
1.6GB. 또 `imageio.mimwrite` 가 에피소드 종료 시 동기 실행되어 수 초 멈춘다.

바꿀 것: `start_episode()` 에서 카메라별 `imageio.get_writer` 를 열고 `add_frame`
에서 바로 append, `save_episode()` 는 writer 를 닫고 parquet/통계만 쓴다.
이미지 통계(`_reduce_image_stats`)는 프레임을 모아두지 않아도 되게 **누적
합/제곱합/최소/최대**로 온라인 계산한다.

### 5.2 실제 시각 보존

`timestamp` 컬럼은 규격상 `i/fps` 를 유지하고(`:68`), 실제 시각은 위 표의
`replay.t_mono`/`cam_t`/`tactile_t` 로 남긴다. 즉 리코더 변경 없이 features 로 해결.

### 5.3 임의 추가 컬럼

`_vec_value` 가 `replay.` 접두어만 `replay` 딕셔너리에서 찾는다(`:52`) — 그 외
비디오·`observation.state`·`action` 이 아닌 키는 KeyError 다. 위 컬럼 전부
`replay.*` 로 두면 **코드 변경 없이** 동작한다. 이름이 의미와 안 맞지만 스키마
호환을 위해 그대로 쓰고, 뜻은 `modality.json` 과 README 에 적는다.

공개 API 가 바뀌므로 `README.md`·`README.ko.md`·`docs/API.md` 3종을 함께 맞춘다.

### 5.4 `discard_episode()`

재촬영(`r`)·중단(Ctrl-C) 에서 현재 에피소드를 버려야 한다. 버퍼를 비우고 열려 있던
mp4 writer 를 닫고 그 에피소드의 parquet/mp4 파일을 지우고 `self._ep` 를 되돌린다.
지금은 이 경로가 없어 반쪽 에피소드가 데이터셋에 남는다.

## 6. 앱 구조

```
~/Desktop/whatslab-collect/
  collect.py        엔트리 — 텔레옵 루프 + 에피소드 운용
  teleop_rig.py     GloveModel 조립, SafetyFilter, 캘리브 스레드
  robot.py          NeroSender / OrcaTouchSender (명령·실측·촉각)
  peer.py           상대방 신호 추출
  cameras.py        RealSenseColor + UvcCamera
  keys.py           논블로킹 키 입력
  README.md         컬럼 레이아웃·실행 순서·캘리브 절차
  data/<task_slug>/ LeRobot v2.1 데이터셋
```

각 모듈의 경계:
- `cameras.py` — 입력: 장치 스펙. 출력: `latest() -> (frame, t)`. 카메라별 스레드가
  최신 프레임만 보관하고 드롭을 허용한다. 로봇도 리코더도 모른다.
- `robot.py` — 입력: SDK q(이름→rad). 출력: 실측 상태 + 촉각 스냅샷. 카메라를 모른다.
- `peer.py` — 입력: `GloveModel` 인스턴스. 출력: 고정 폭 벡터 + valid 플래그.
- `collect.py` — 위 3개와 `LeRobotRecorder` 를 엮는 유일한 곳.

## 7. 스레드와 타이밍

- **메인 60Hz**: `model.get_q()` → SafetyFilter → 로봇 전송.
- **기록 30Hz**: 2틱마다 스냅샷을 큐에 넣는다. 큐가 밀리면 드롭 수를 콘솔에 찍는다
  (침묵 절단 금지 — CLAUDE.md 원칙).
- **카메라 3스레드**: 최신 프레임 + 수신 시각.
- **하드웨어 읽기 1스레드**: nero `get_joint_angles()`, orca `get_joint_position()`,
  `get_tactile_data()` 를 최대 속도로 폴링해 캐시. 메인 루프에서 직접 읽으면 60Hz 가
  무너진다.
- **리코더 스레드**: `add_frame` (mp4 append 포함).

## 8. orca touch 연결 절차 (실측으로 확인)

```python
hand = orca_core.OrcaHandTouch(config_path=".../models/v2/orcahand_touch_right/config.yaml")
hand.connect()                       # 모터 버스 + 센서 시리얼 동시
hand.init_joints(move_to_neutral=False)
hand.start_tactile_stream(resultant=True, taxels=True)
hand.capture_taxel_offsets()         # 손에 접촉이 없는 상태에서 영점
```

확인된 함정:
- **`start_tactile_stream()` 전에는 촉각 게터가 전부 `None`** 이고
  `get_tactile_stats().frames_ok == 0` 이다.
- 기본값이 `taxels=False`(`hardware_hand.py:1434`) 라서 스트림을 켜도
  `get_tactile_taxels()` 가 계속 `None` 이다 — **`taxels=True` 를 명시**해야 한다.
- 센서 시리얼이 921600 baud 이고 결합 모드 페이로드가 프레임당 약 1.1KB 라 이론
  상한이 약 80Hz 다. 30Hz 는 되지만 여유가 크지 않다.
- 영점을 안 잡으면 택셀 바이어스가 세션마다 다르게 실린다.

## 9. 시작 전 점검 (하나라도 실패하면 수집을 시작하지 않는다)

1. 카메라 3대에서 **실제 프레임이 도착**했는가
2. Quest 왼쪽·오른쪽 컨트롤러가 둘 다 유효한가
3. 글러브 left·right 관절각이 둘 다 오는가
4. nero enable + `get_joint_angles()` 성공
5. orca `calibrated == True`, `get_joint_position()` 성공
6. **1초간 촉각 프레임 수 ≥ 30** (`get_tactile_stats().frames_ok` 차분).
   미달이면 **택셀을 끄고 합력만** 받는 모드로 폴백하고 그 사실을 데이터셋 메타에 남긴다
7. yaw 캘리브가 수행되었는가 (미수행 시 경고 후 진행 가능)

## 10. 에피소드 운용

LeRobot 관례를 그대로 쓴다:
- `n` 또는 `→` : 저장하고 다음 에피소드
- `r` 또는 `←` : 현재 에피소드 버리고 같은 인덱스로 재촬영
- `q` 또는 `Esc` : 종료 (`finalize()` 로 meta 기록)

- task 문자열은 `--task "..."` 로 세션 고정.
- 길이는 가변, `--max-seconds`(기본 60) 상한.
- 재촬영은 `discard_episode()`(5.4) 로 처리한다 — 버퍼를 버리고 그 에피소드의
  parquet/mp4 를 지운 뒤 같은 인덱스를 다시 쓴다.

## 11. 오류 처리

- 카메라 스톨: 직전 프레임 재사용. `replay.cam_t` 가 정체되어 사후 탐지 가능.
  1초 초과 시 콘솔 경고.
- nero/orca 읽기 실패: 직전값 유지 + `replay.state_valid` 0.
- 상대방 신호 끊김: 직전값 유지 + `replay.peer_valid` 0.
- 촉각 프레임 없음: 직전값 유지, `replay.tactile_t` 정체로 탐지.
- Ctrl-C: 현재 에피소드는 버리고(`discard_episode`) `finalize()` 만 수행 — 반쪽
  에피소드를 남기면 학습 쪽에서 조용히 섞인다.
- nero CAN 예외: e-stop 후 종료. 데이터보다 하드웨어가 우선이다.

## 12. 검증

- `--dry-run`: 로봇·카메라 없이 합성 프레임으로 전 경로 통과. 스키마 회귀 테스트용.
- SDK pytest 추가: 스트리밍 mp4 라이터의 프레임 수 일치, 온라인 이미지 통계가
  기존 `_reduce_image_stats` 와 일치(같은 입력 → 같은 값), `discard_episode` 후
  인덱스 재사용, `replay.*` 컬럼 왕복.
- 앱 pytest: `peer.py` 를 가짜 리시버로 단위 테스트(폭·valid 플래그·side 혼동 방지).
  side 혼동은 이 저장소에서 반복된 실측 버그 유형이라 반드시 테스트한다.
- 실기 스모크: 10초 수집 → 프레임 수 = 30×10 ±2, 실제 폴링 주기 출력,
  `export_targets.py --groot` 통과.

## 13. 알려진 미검증 리스크

- **nero CAN + orca serial + 촉각 폴링이 30Hz 를 받치는지 미측정.** 별도 스레드로
  격리했으니 루프는 안 무너지지만, 느리면 `observation.state` 가 몇 틱 묵은 값이 된다.
  첫 스모크에서 실제 주기를 찍어 판단한다.
- **mp4 인코딩이 손실이고 비가역.** 원본 픽셀을 다른 코덱으로 다시 뽑을 수 없다.
  h5 원본을 버리는 대가로 받아들인 선택이다.
- **`observation.state` 에 팔 실측과 손 실측이 섞이는데 두 하드웨어의 지연이 다르다.**
  `replay.t_mono` 하나로는 분리되지 않는다. 지연 차가 문제로 드러나면 소스별
  타임스탬프 컬럼을 추가해야 한다.
