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
| `GloveModel(robot, hand_source="openxr", arm_source="quest", arm_kwargs=None, hand_kwargs=None)` | 프리셋: 팔=컨트롤러 IK(위치=반대손 컨트롤러, 회전=글러브 손목 — 손목이 없으면 컨트롤러 회전, `"openxr"` 소스는 손목을 내지 않으므로 항상 컨트롤러 회전), 손=글러브. 햅틱 지원. `hand_source` = `HAND_SOURCES` 키 — `"openxr"`(기본, `GloveOpenXRReceiver`) / `"angles"`·`"robot"`(레거시, Spine ≤ 2.3.1). `hand_kwargs`/`arm_kwargs` 는 리시버 생성자 인자(예: `{"glove_port": 4040}`). |
| `HandModel(hand_config="dex:orca", side="right", urdf_root=None, hand_source=None, hand_kwargs=None, **kwargs)` | 프리셋: 손 리타게팅 단독(팔 IK 없음). `hand_config` 가 `dex:<이름>` 이면 OpenXR `xr_joints` → `XRHandRetargeter`, 그 밖이면 레거시 관절각 → ONNX. `hand_source` 를 안 주면 `GloveOpenXRReceiver(**hand_kwargs)`. 인스턴스와 `hand_kwargs` 를 같이 주면 `ValueError`. 남은 `kwargs` 는 `HandRetargetController` 로 간다. |
| `RobotArmIK(...)` | 팔 IK 컴포넌트. 정준 목표 4x4 → 팔 관절각. 프레임 추종은 rig 백엔드의 `solve()`, 전역 재탐색(`solve_robust`)은 **첫 타깃(`_cold_start`)과 `reseed()`** 에서만(후보 하나가 수 ms라 매 프레임 불가). 튜닝은 `cold_*` 속성. |
| `ArmCalibration(reach_max, input_reach, enabled=True)` | yaw 정렬 + 캘리브 원점(`p0`) + reach 스케일 소유. `enabled=False` 면 리시버 좌표를 그대로 통과시킨다(rig `calibration.enabled`). |

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
| `QuestControllerReceiver` | `InputSample(controller=Pose)` | Quest 컨트롤러 6D 위치/자세(OSC/UDP, 포트 9000). `connected(side)`. |
| `WebXRControllerReceiver` | `InputSample(controller=Pose, hmd=Pose)` | 브라우저 WebXR 컨트롤러 6D(WebSocket, 포트 8443). OSC 경로와 **병행** 가능. HMD 는 회전뿐 아니라 **위치도** 온다. `connected(side)`, `url`. |
| `QuestHandReceiver` | `InputSample(hand=HandPose)` | Quest 핸드트래킹(손목 6D + 손가락). `connected(side)`. |
| `GloveOpenXRReceiver(glove_port=4040, listen_ip="127.0.0.1", target_ip="127.0.0.1", client_port=4042, stale_timeout=0.2, pair=1)` | `InputSample(hand=HandPose(wrist=None, xr_joints=(26,8)))` | **현재 Spine**(프로토콜 3) 글러브 수신(아래 「OpenXR 글러브」). `airglove_client.Client` 를 감싼다. `connected(side)`, `send_haptic(side, values)`, `raw_wrist(side)`, `spine_protocol()`, `running`. |
| `GloveHumanAnglesReceiver` | `InputSample(hand=HandPose(joint_angles=…))` | **레거시(Spine ≤ 2.3.1).** 사람 손 URDF 관절각(`/{side}/joint_angles/get`) + 손목. 현재 Spine 에서는 받는 게 없다. |
| `GloveHumanHandReceiver` | `InputSample(hand=HandPose(joint_rot=…))` | **레거시(Spine ≤ 2.3.1).** AirGlove 손가락 회전(`/{side}/quat/get`). 전송 계층만. |
| `GloveRobotHandReceiver` | `InputSample(joint_q=…, hand=wrist만)` | **레거시(Spine ≤ 2.3.1).** Spine 이 IK 를 끝낸 URDF 관절각을 직접 받는다(손 리타게팅 바이패스). `joint_map` = Spine 이름→로봇 관절명. |

공통: `start()`, `stop()`, `get(side) -> InputSample`.

### OpenXR 글러브 (`GloveOpenXRReceiver`)

```python
from whatslab.receiver.glove import GloveOpenXRReceiver, XR_JOINT_NAMES
rx = GloveOpenXRReceiver()      # Spine → 127.0.0.1:4040 수신, 요청·햅틱 → 127.0.0.1:4042
rx.start()                      # 여기서 airglove_client.Client 를 만든다(포트 독점 바인드)
s = rx.get("right")
s.hand.xr_joints                # (26, 8) [qx qy qz qw px py pz radius], 손목 공간
```

전송 계약의 정본은 AirGlove-Client-Core `docs/spine_protocol.md`, 클라이언트 API 는
AirGlove-Client `python/README.md` 다. 이 SDK 는 주소·페이로드를 직접 파싱하지 않는다.
현재 Spine 은 프로토콜 **3**(모든 메시지 첫 인자가 번호 문자열, 손·장치 주소 앞에 `/pair/N`,
Spine 은 임시 송신 포트에서 `127.0.0.1:4040` 으로 보낸다)이고 이를 말하는 것은
`airglove_client` 0.4.0 이다(`receiver` extra 가 `@v0.4.0` 으로 고정). `start()` 는 라이브러리가 0.4.0 미만이면 경고하고, 첫 하트비트 응답이 프로토콜
3 이 아니면 `get()` 이 한 번 경고한다(`spine_protocol()` = 보고된 번호 또는 `None`).

- 생성자 인자 → `Client`: `glove_port` → `listen_port`, `listen_ip` → `listen_address`,
  `target_ip` → `spine_address`, `client_port` → `spine_port`, `pair` → `Client.set_pair`
  (1 이 기본이라 1 이면 부르지 않는다. 1 이 아닌데 패키지에 `set_pair` 가 없으면 `start()` 가
  `AirGloveError`). `set_stale_threshold` 는 0.4.0 에서 효과가 없어 부르지 않는다.
  **`Client` 는 `start()` 에서 만들고**
  `stop()` 에서 닫는다 — 생성만으로는 포트를 잡지 않는다. 한 포트에 클라이언트는 하나뿐이라
  두 번째 바인드는 `airglove_client.AirGloveError`.
- `xr_joints` — 26 OpenXR `XrHandJointEXT` 관절(`XR_JOINT_NAMES`: palm, wrist, thumb
  metacarpal..tip, index..little metacarpal..tip)을 받은 그대로: 손목 공간(손목 = 단위,
  `XR_WRIST_INDEX` = 1), OpenXR 관절 축(−Z = 뼈 따라 손끝, +Y = 손등, X = Y×Z, 양손 같은
  규칙), 미터. palm 은 Spine 이 합성한다. OpenXR[i+1] == WebXR[i]. **정준 변환하지 않는다.**
- `wrist` — **항상 `None`.** 프로토콜 3 의 손목(18/19)은 SDK 의 world 손목 자세가 아니라
  손등 IMU 의 raw 쿼터니언을 Spine 이 `[y, x, z, -w]` 로 재배열한 값이다. 캘리브·heading
  정렬이 없는 센서 프레임이라 정준 프레임으로 옮길 근거가 없고, 팔 목표로 쓰면 팔이 틀린
  방향으로 움직인다. 그래서 `HandPose.wrist` 에 넣지 않고 `GloveModel` 은 팔 회전을 반대손
  컨트롤러 회전으로 쓴다(손마다 경고 한 번). raw 값은 `raw_wrist(side)` → `(4,)` 배열(받은
  그대로, 정규화·재배열 없음) 또는 `None`(없음·`stale_timeout` 넘음)으로만 낸다.
  `openxr_quat_to_canonical` 은 OpenXR 쿼터니언용 변환으로 남아 있다.
- `tracked` = 프레임이 있고 `age_s < stale_timeout`(0 이하면 나이 검사 없음)이고
  26관절이 전부 `valid`. 오래된 프레임도 `xr_joints` 는 그대로 싣고 `tracked=False` 만 낸다.
  `age_s` 는 클라이언트가 받은 뒤 지난 시간이다 — 프로토콜 3 에는 Spine 시각·순번이 없고
  `seq`·`sender_time_us` 는 클라이언트가 센 값·받은 시각이라 신선도 판단에 쓰지 않는다.
  `timestamp` = `time.monotonic() - age_s`.
- `connected(side)` = 2 s 안에 받은 Spine `device_status` 가 그 손을 연결로 보고하거나
  신선한 프레임이 있음. 2 s 보다 오래된 상태(Spine 이 응답을 멈춤)는 무시한다.
- `send_haptic(side, values)` → `bool`. 값은 손가락별(엄지..소지, 앞 5개까지) Spine 세기
  단계로 본다: `round` 후 `[0, 3]` 클램프(0 끔, 1 약, 2 중, 3 강), 숫자가 아니거나 NaN 이면 0
  (`haptic_levels`). 항상 정확히 5개를 보낸다 — 빠진 손가락은 0(빈 값 = 전부 끔).
  글러브는 0 을 받을 때까지 계속 진동한다(Spine 은 클라이언트 하트비트가 3 s 끊기면 끈다).
  시작 전·전송 실패면 `False`.

### WebXR 경로

```python
from whatslab.receiver.webxr import WebXRControllerReceiver
rx = WebXRControllerReceiver()           # 기본이 무선 = 자체 서명 HTTPS, 포트 8443
rx.start()                               # 정적 페이지 + WebSocket 을 같은 포트에서 연다
print(rx.url)                            # https://192.168.1.88:8443/
```

헤드셋 브라우저로 `rx.url` 을 열고 **VR 시작**을 누르면 매 XR 프레임마다
`gripSpace` 포즈(+ `getViewerPose`)가 JSON 으로 올라온다.

- **패스스루 + 컨트롤러 마커.** 세션은 `immersive-ar` 를 먼저 시도하고(Quest 패스스루,
  `alpha-blend`) 안 되면 `immersive-vr` 로 떨어진다. 컨트롤러 그립마다 초록 큐브와
  RGB 축을 그리므로 **헤드셋 안에서 켜졌는지 눈으로 확인된다** — 큐브가 손을 따라오면
  포즈가 나오고 있는 것이다. AR 일 때는 `clearColor` 의 알파를 0 으로 두어 실제 주변이
  비치고, VR 일 때만 바닥 그리드를 그린다.

- **보안 컨텍스트가 필요하다.** WebXR 은 HTTPS 이거나 `localhost` 에서만 뜬다.
  - **무선(기본, `tls=True`)**: 자체 서명 인증서를 `~/.cache/whatslab/webxr/` 에 만들어
    HTTPS + WSS 로 연다. URL 호스트는 기본 경로의 src IP(= 헤드셋이 붙을 LAN IP)다.
    인증서 SAN 에 `localhost` 와 이 머신의 모든 전역 IPv4 가
    들어간다(`openssl` 실행파일 필요, 유효기간 10년, 키 퍼미션 0600). IP 가 바뀌면
    자동으로 재발급한다. **헤드셋에서 최초 1회 인증서 경고를 수락**해야 한다 —
    페이지와 WebSocket 이 같은 오리진(같은 포트)이라 한 번 수락하면 WSS 도 통과한다.
    직접 만든 인증서를 쓰려면 `certfile=`/`keyfile=` 을 넘긴다.
  - **유선**: `tls=False` + `adb reverse tcp:8443 tcp:8443` — 헤드셋이 PC 서버를
    `localhost` 로 본다(인증서 불필요). URL 도 이때만 `localhost` 가 된다.
- **위치는 HMD 상대로 낸다.** `ArmCalibration.apply()` 가 위치를 그대로 쓰므로
  (`T[:3,3] = scale * p` — `p0` 빼기도 `W` 곱하기도 없다) 리시버가 이미 상대 위치를
  주어야 한다. 브라우저는 `local-floor` 절대좌표를 보내고, 변환은 리시버에서 한다.

  | `pos_frame` | 계산 | 용도 |
  |---|---|---|
  | `hmd_yaw` (기본) | `Rz(−yaw_hmd)·(p_ctrl − p_hmd)` | 회전의 `_head_relative` 와 같은 규약 |
  | `hmd` | `p_ctrl − p_hmd` | 머리 위치만 빼고 방 축은 유지 |
  | `room` | 변환 없음 | 원시 `local-floor` — 진단용 |

  HMD 신호가 없으면 자동으로 원시값으로 떨어진다. 실측 비교(머리 `[1,1.6,2]`,
  컨트롤러 `[1.3,1.2,1.5]`): `room` 이면 `|p| = 2.319` 로 `reach_max 0.9204` 에
  걸려 클램프되고, `hmd`/`hmd_yaw` 는 `0.707` 로 도달 가능하다.
- 프레임은 **WebXR 프레임**(오른손계, +Y 위, −Z 앞)으로 오고 `WebXRReceiverBase._M`
  이 정준으로 보낸다. 이 행렬은 `receiver/vive.py` 의 OpenVR 변환과 같다 —
  두 API 의 축 관례가 같기 때문이다.
- 전송은 WebSocket(TCP)이라 UDP 처럼 늦은 프레임을 버리지 않는다. 브라우저 쪽에서
  `bufferedAmount` 가 밀리면 그 프레임을 **보내지 않고 버린다**(지연 누적 방지).
- 의존성이 없다. WebSocket 핸드셰이크·프레임 디코딩은 표준 라이브러리로 구현했다
  (`receiver/webxr/ws_server.py`) — 새 extra 가 필요 없다.
- **핸드트래킹은 제공하지 않는다.** 손 리타게팅 입력은 사람 손 URDF 관절각이고
  WebXR `XRHand` 는 키포인트라, 역산 경로가 없다(`spherical_fk.py` 와 함께 제거됨).

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
from whatslab.solvers.hand import HandRetargetController, XRHandRetargeter
```

엔진은 입력에 따라 둘이다. `config_name`(rig `retarget:`)이 `dex:` 로 시작하면 고른다.

- **`dex:<이름>` — 현재 글러브(OpenXR).** 입력은 `HandPose.xr_joints` `(26, 8)`.
  dex-retargeting 0.4.6 의 DexPilot(손끝·손가락 사이 벡터 + 핀치 투영)을 쓴다.
- **그 밖 — 레거시 관절각.** 입력은 **사람 손 URDF 관절각** — `HandPose.joint_angles`
  (이름→rad, 옛 Spine 의 `/{side}/joint_angles/get`). 엔진은 학습된 통합 ONNX 모델 하나
  (`assets/uni_all.onnx`)로, 사람 FK·인코더·헤드·관절범위 역정규화가 전부 그래프
  안에 있다. 로봇 의존부(관절 기술자·범위·사이드 부호)는 `assets/uni_tables.npz`
  표에서 입력으로 들어간다 — 로봇이 늘어도 모델은 안 바뀐다.

| 심볼 | 설명 |
|---|---|
| `HandRetargetController(hand_type, config_name="base_hand", urdf_root=None, **kwargs)` | 손 리타게팅 컨트롤러. `compute(InputSample) -> HandCommand`. `xr_input`(bool) = dex 엔진인가. 엔진 입력(`xr_joints` / `joint_angles`)이 없거나 `tracked=False` 면 직전 명령 유지(dex 는 처음에 0 을 관절범위로 자른 값). `kwargs` 는 엔진 생성자로 간다 — dex 는 `max_evals` 와 dex-retargeting `RetargetingConfig` 필드만 받고 그 밖은 `TypeError`(rig `hand_solver:` 는 ONNX 전용이라 dex 와 같이 쓰면 `RobotModel` 이 `ValueError`). |
| `XRHandRetargeter(config, side=None, urdf_root=None, max_evals=None, **overrides)` | OpenXR → 로봇 손 DexPilot. `config` = `configs/hands` 이름 / `dex:` 이름 / yaml 경로 / `DexHandConfig`. `side` 가 config 의 side 와 다르면 입력을 거울상(`mirror_xr_joints`)으로 바꿔 쓴다. `compute(xr_joints) -> q`(`joint_names` 순서, rad). `joint_names`, `joint_limits` `(n, 2)`, `robot_joint_names`(고정·mimic 관절 포함), `full_qpos(q)`(고정 관절 값 + URDF mimic 관절을 채운 전 관절), `human_keypoints(xr)`(MANO 21점, 로봇 루트 축), `last_keypoints`, `scaling_factor`, `reset()`. 손당 프레임 p50 1.0~2.0ms, p99 약 4ms(아래 측정). |
| `DexHandConfig.load(name, side=None)` / `from_dict(d)` | `configs/hands/*.yaml` 로더(아래 키). `retargeting` 에 dex-retargeting 이 모르는 키가 있으면 `ValueError`. |
| `hand_config_names()` | 동봉 `configs/hands` 이름 목록. |
| `xr_to_mano(xr_joints)` | OpenXR 26 → MANO 21 손목 상대 위치. 인덱스 `[1,2,3,4,5,7,8,9,10,12,13,14,15,17,18,19,20,22,23,24,25]`, 축 `[[0,-1,0],[-1,0,0],[0,0,-1]]`(양손 같음 — 손목 공간 위치를 손목 회전으로 한 번 더 풀어 손목 자세와 무관하다). |
| `mirror_xr_joints(xr_joints)` | 손목 공간 x 축 거울상(왼손 ↔ 오른손). 두 번 하면 원래대로. |
| `HumanHandFK(side, urdf_path=None)` | 사람 손 URDF FK — 지금은 **viz 전용**이다(추론 경로에 FK 가 없다). `points(angles)` = 손가락별 키포인트 4개 + `palm`, `neutral_points()`, `q_from_named(이름→rad)`. `joint_names` = URDF revolute 관절(base 프로파일 21개). 링크명으로 참조하고 손끝은 `{side}_sensor_{finger}_distal` → `{side}_{finger}_tip` 순으로 찾는다. |
| `UniRetargeter(hand_type, config_name, threads=1)` | 레거시 관절각용 통합 ONNX 엔진. `compute(dict 이름→rad) -> q_robot(rad)`. CPU 1스레드 프레임당 약 1.1ms(900Hz+). 담당 로봇: orca / allegro / tesollo / robotis / human(base_hand), 좌우 모두 한 그래프. 표에 없는 손이면 가능한 목록과 함께 에러. |

### 손 리타게팅 config (configs/hands)

`dex:<이름>` 이 읽는 yaml. `<이름>.yaml` 을 먼저, 없으면 `<이름>_{side}.yaml` 을 찾는다.

| `dex:` 이름 | URDF (models root 기준) | 손목 링크 → 손끝 링크 | 비고 |
|---|---|---|---|
| `orca` | `orca_hand/urdf/{side}.urdf` | Carpals → `{side}_sensor_*_distal` ×5 | 손목 관절 0 고정 |
| `dg5f` | `tesollo_dg5f/dg5f_{side}.urdf` | `?l_dg_palm` → `?l_dg_[1-5]_tip` | PIP·DIP −5..90° |
| `robotis` | `robotis_hx5_d20/urdf/hx5_d20_{side}.urdf` | `hx5_d20_{side}_base` → `{side}_sensor_*_distal` ×5 | |
| `allegro` | `allegro_hand/allegro_hand_{side}.urdf` | `wrist` → `link_{15,11,7,3}.0_tip`(왼손) / `link_{15,3,7,11}.0_tip`(오른손) | 손가락 4개, 16축 |
| `leap` | `leap_hand/leap_hand_{side}.urdf` | `base` → `{thumb,index,middle,ring}_tip_head` | 손가락 4개, 16축 |
| `ability` | `ability_hand/ability_hand_{side}.urdf` | `base_link` → `{thumb,index,middle,ring,pinky}_tip` | 구동 6축, 손가락 q2 는 mimic |
| `schunk` | `schunk_hand/schunk_svh_hand_{side}.urdf` | `{side}_hand_base_link` → `{th,ff,mf,rf,lf}tip` | 구동 9축, 나머지는 mimic |

allegro·leap·ability·schunk 의 손목·손끝 링크와 `target_joint_names` 는 dex-retargeting
`configs/teleop/<hand>_{left,right}_dexpilot.yml` 그대로다. `scaling_factor`·`low_pass_alpha` 는
그 config 값(1.0~1.6, 0.2) 대신 다른 손과 같은 규칙(길이비, 0.5)을 쓴다.

| 키 | 뜻 |
|---|---|
| `side` | 이 URDF 가 어느 손인가(`left`/`right`). |
| `urdf` | models root 기준 URDF 경로. |
| `palm_links` | `{index, middle, pinky}` — q=0 에서 손바닥 프레임을 잡을 MCP 링크(손가락 4개인 손은 `pinky` 에 약지). z = 손목 링크 → 중지 MCP, y = 소지 → 검지(오른손; 왼손은 반대), x = y × z. 사람 쪽도 같은 규칙(MANO 0·5·9·17)으로 매 프레임 잡아 사람 → 로봇 루트 회전을 만든다. |
| `mano_to_root.rpy` | (선택) 위 자동 정렬 대신 고정 MANO → 루트 회전(xyz 오일러, rad). |
| `fixed_joints` | `{관절: rad}` — 리타게팅에서 빼고 고정한다. ORCA 손목 관절은 팔 IK(rig `target_ee` = Carpals)가 쓰므로 여기서 0 으로 둔다. |
| `joint_limits_deg` | `{관절 이름 glob: [lo, hi]}` — 최적화 한계를 덮어쓴다. DG-5F 손가락 PIP·DIP 는 URDF 가 ±90° 라 역방향으로 꺾는 해가 나와 −5..90° 로 자른다. |
| `human_hand_length` | 사람 손목 → 중지 끝(m, 기본 0.19). `retargeting.scaling_factor` 가 없으면 로봇 같은 길이(q=0)를 이것으로 나눠 쓴다. |
| `max_evals` | 틱당 SLSQP 평가 상한(동봉값 20). 무제한과 프레임 무관 지표가 같고(아래) p99 지연만 준다. |
| `retargeting` | dex-retargeting `RetargetingConfig` 그대로(`type`, `wrist_link_name`, `finger_tip_link_names`, `low_pass_alpha`, …). `urdf_path` 는 위 `urdf` 로 채우고 `target_joint_names` 를 안 주면 `fixed_joints` 를 뺀 전 관절이다. URDF 에 mimic 관절이 있으면 `target_joint_names` 에서 빼서 줘야 한다(dex-retargeting 이 mimic 으로 채운다 — ability·schunk). |

측정(`tools/bench_hand_retarget.py`, 목표와 무관한 물리 지표 — 정의는 그 도구의
독스트링). 입력은 agaSDK Python 손 모델이 실데이터에서 낸 OpenXR 26관절이다:
Quest 핸드트래킹 녹화 11개 × 양손을 센서 회전으로 바꿔 모델에 넣은 것(`quest→aga`)과
실제 글러브 raw 녹화 16개(캘리브 없음, `glove`).

| 입력 | config | 펴짐 접촉 오차 mm | 핀치 로봇 손끝 거리 mm (n) | 뼈 방향각 ° | \|dq\| p95 rad | 한계 2% 비율 | 손당 ms p50 / p99 | 2손 p99 ms |
|---|---|---|---|---|---|---|---|---|
| quest→aga (20718) | orca_right | 3.2 | 5.0 (562) | 11.6 | 0.106 | 0.087 | 1.15 / 3.79 | 7.10 |
| | orca_left | 3.2 | 5.0 (562) | 11.5 | 0.106 | 0.085 | 1.16 / 3.84 | 6.97 |
| | dg5f_right | 4.2 | 5.1 (562) | 12.1 | 0.089 | 0.133 | 1.05 / 4.21 | 7.65 |
| | dg5f_left | 4.2 | 5.1 (562) | 12.1 | 0.089 | 0.133 | 1.06 / 4.20 | 7.70 |
| | robotis_right | 2.6 | 5.5 (562) | 11.1 | 0.103 | 0.204 | 1.18 / 4.03 | 7.42 |
| | robotis_left | 2.5 | 7.2 (562) | 11.5 | 0.104 | 0.229 | 1.18 / 4.11 | 7.27 |
| | allegro_right | 7.7 | 2.4 (418) | 16.6 | 0.079 | 0.151 | 0.95 / 3.21 | 5.93 |
| | allegro_left | 8.0 | 2.6 (418) | 16.5 | 0.079 | 0.155 | 0.96 / 3.26 | 5.94 |
| | leap_right | 5.0 | 2.1 (418) | 29.9 | 0.103 | 0.079 | 0.95 / 3.70 | 6.84 |
| | leap_left | 6.0 | 2.3 (418) | 30.5 | 0.101 | 0.081 | 0.94 / 3.70 | 6.84 |
| | ability_right | 6.5 | 6.2 (562) | 13.9 | 0.091 | 0.142 | 0.93 / 2.36 | 4.23 |
| | ability_left | 6.5 | 6.2 (562) | 13.9 | 0.091 | 0.142 | 0.93 / 2.33 | 4.14 |
| | schunk_right | 9.9 | 9.6 (562) | 16.8 | 0.078 | 0.425 | 0.95 / 2.96 | 5.16 |
| | schunk_left | 9.4 | 10.2 (562) | 17.1 | 0.077 | 0.410 | 0.95 / 2.91 | 5.18 |
| glove (24528) | orca_right | 4.5 | 3.4 (812) | 13.3 | 0.176 | 0.168 | 1.28 / 3.78 | — |
| | orca_left | 4.4 | 3.5 (812) | 13.5 | 0.177 | 0.162 | 1.31 / 3.80 | — |
| | dg5f_right | 5.4 | 4.1 (812) | 15.9 | 0.158 | 0.267 | 1.44 / 4.22 | — |
| | dg5f_left | 5.4 | 4.1 (812) | 15.9 | 0.158 | 0.267 | 1.45 / 4.26 | — |
| | robotis_right | 4.9 | 4.4 (812) | 13.7 | 0.195 | 0.258 | 1.95 / 4.27 | — |
| | robotis_left | 4.7 | 4.9 (812) | 14.2 | 0.182 | 0.303 | 1.94 / 4.24 | — |
| | allegro_right | 12.1 | 2.2 (693) | 16.7 | 0.154 | 0.244 | 1.50 / 3.63 | — |
| | allegro_left | 12.4 | 2.2 (693) | 16.7 | 0.154 | 0.244 | 1.50 / 3.65 | — |
| | leap_right | 7.7 | 1.9 (693) | 33.8 | 0.161 | 0.182 | 1.46 / 3.79 | — |
| | leap_left | 9.7 | 2.3 (693) | 34.8 | 0.159 | 0.182 | 1.48 / 3.91 | — |
| | ability_right | 9.5 | 4.6 (812) | 21.7 | 0.130 | 0.255 | 1.14 / 2.48 | — |
| | ability_left | 9.5 | 4.6 (812) | 21.7 | 0.130 | 0.255 | 1.13 / 2.45 | — |
| | schunk_right | 12.7 | 7.0 (812) | 24.4 | 0.116 | 0.474 | 1.14 / 3.35 | — |
| | schunk_left | 12.0 | 7.1 (812) | 24.7 | 0.119 | 0.472 | 1.14 / 3.37 | — |

2손 p99 = 같은 틱의 왼손 + 오른손(양손 녹화가 있는 quest→aga 만). 판정 기준(펴짐·핀치 ≤ 10mm,
2손 p99 ≤ 8ms)은 전부 통과하지만 2손 p99 는 7.0~7.7ms 로 여유가 작다. `max_evals` 를 빼면(무제한)
프레임 무관 지표는 거의 같고(뼈 방향각 ±0.1°) 2손 p99 만 는다(같은 부하에서 20 → 6.5~7.7ms, 무제한 → 7.7~8.4ms). 핀치 "오차"(로봇 거리 −
스케일된 사람 거리)는 10.5~15mm 인데, DexPilot 투영이 사람 3cm 안쪽을 접촉으로 당기는 설계라서다.

allegro·leap·ability·schunk(`--mirror`, 같은 입력): 손가락 4개인 allegro·leap 는 소지 핀치를 빼서
핀치 n 이 적다. 핀치는 전부 ≤ 10.2mm, 2손 p99 ≤ 6.9ms 로 통과한다. 펴짐은 quest→aga 에서 ≤ 10mm 지만
glove 에서 allegro(12.1~12.4mm)·schunk(12.0~12.7mm)가 기준을 넘는다 — allegro 는 손이 커서
(배율 1.28) 엄지 위치 차가 커지고, schunk 는 관절범위가 좁아 한계 2% 비율이 0.41~0.47 이다.
leap 의 뼈 방향각 30~35° 는 링크 원점이 손가락 축에서 벗어나 있는 탓이 크다(도구가 관절 원점으로
뼈를 잡는다). ability·schunk 의 bone/limit 은 구동 관절 + mimic 관절 기준이다.

지원하는 손(레거시 ONNX) — `config_name` 에 넣는 값(괄호는 별칭)과 `dexhand-description` 안의 URDF:

| 손 | `config_name` | URDF |
|---|---|---|
| 사람 기준 손 | `human` (`base_hand`) | `base_hand/urdf/{side}.urdf` |
| ORCA Hand | `orca` (`orca_hand`) | `orca_hand/urdf/{side}.urdf` |
| Allegro Hand | `allegro` (`allegro_hand`) | `allegro_hand/allegro_hand_{side}.urdf` |
| Tesollo DG-5F | `tesollo` (`tesollo_dg5f`) | `tesollo_dg5f/dg5f_{side}.urdf` |
| ROBOTIS HX5-D20 | `robotis` (`robotis_hx5_d20`) | `robotis_hx5_d20/urdf/hx5_d20_{side}.urdf` |

별칭은 `uni_retargeter._ALIAS`, URDF 경로 규칙은 `_URDF_STEM` 이 정본이다.
`urdf_path` 는 `<root>/<sub>/urdf/<stem>.urdf` → `<root>/<sub>/<stem>.urdf` 순으로 찾고
없으면 `None` 을 낸다(pinocchio 없이 경로만 계산).

새 로봇 손 온보딩: retarget_net 의 `tools/onboard_urdf.py` 가 URDF 하나에서 표를
뽑는다(센서 프레임 계약 필요). 학습에 없던 손은 표만으로는 성능이 안 나온다 —
몇백 스텝 미세조정 후 그래프를 재수출한다.

## whatslab.core — 계약(타입 + Protocol), 의존성 0

| 심볼 | 설명 |
|---|---|
| `types.Pose` | 위치 + quaternion(xyzw). |
| `types.HandPose` | 손목 6D + `joint_angles`(관절명→rad, 손 리타게팅 입력) + `joint_rot`(관절명→회전, quat 전송용) + `xr_joints`(`Optional[np.ndarray]` `(26,8)`, OpenXR 글러브 관절, 손목 공간 — `XRHandRetargeter` 입력). |
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
| `add_frame(state, action, images, replay, task)` | 한 프레임 누적. 이미지는 누적하지 않고 **에피소드 mp4 에 즉시 append** 한다(RAM 상수). 벡터만 버퍼에 쌓인다. 에피소드 인덱스가 `chunks_size` 를 넘으면 여기서 `NotImplementedError`. |
| `save_episode()` | mp4 를 닫고 현재 에피소드를 v2.1 parquet + 통계로 저장. 프레임이 없으면 `discard_episode()` 와 같다. |
| `discard_episode()` | 현재 에피소드를 버린다 — mp4 를 닫고 그 에피소드의 mp4·parquet 파일을 지우고 인덱스를 재사용한다. 재촬영·중단 처리용. |
| `finalize()` | 데이터셋 메타 마감. 열린 mp4 가 있으면 닫는다. |

이미지 통계는 프레임을 모아두지 않고 누적 합/제곱합/최소/최대로 온라인 계산한다
(`lerobot_schema.ImageStats`) — 배치 계산(`_reduce_image_stats`)과 같은 값을 낸다.

## whatslab.viz — viser 웹 3D 시각화 (`whatslab-sdk[viz]`)

`get_server(port=8080)` 로 포트당 서버 공유(여러 viz 공존). `http://localhost:8080`.

| 클래스 | 내용 |
|---|---|
| `URDFScene` | URDF 하나를 메쉬(STL)/스켈레톤 자동 판별 렌더 + 관절 구동. `set_root`, `q_from_named`, `fk`, `frame_pose`. |
| `RobotArmViz` | 팔+손 URDF 메쉬를 solver q 로 구동 + 목표 EE 프레임. `start`, `update`. |
| `HandViz(urdf, joint_names, root_pose=None)` | 손 URDF 하나를 q 로 구동. 사람 손이든 로봇 손이든 같은 클래스다(상속 없음) — 손을 세우려면 `root_pose` 에 `upright_root`/`human_upright_root` 결과를 넘긴다. |
| `upright_root(palm_origin, palm_R)` / `human_upright_root(fk, offset=None)` | 팜 프레임을 화면 기준으로 세우는 루트 4x4. `offset` 으로 옆에 띄운다. |

## whatslab.safety — 운동학 안전 유틸 (dep-light)

로직만 제공. 강제(watchdog/e-stop 배선)는 소비자(ROS safety_gate 등)가, 최종 권위는 하드웨어가 갖는다.

| 심볼 | 설명 |
|---|---|
| `JointLimit` | 관절 pos/vel 한계. |
| `load_limits_from_urdf(urdf_xml)` | URDF → `{joint: JointLimit}`. |
| `tighten(base, ...)` | 한계를 보수적으로 조임. |
| `SafetyFilter` | clamp + rate-limit + hold/estop 상태기. `step(desired, dt=None)`, `trip`, `reset`, `estopped`, `enabled`, `set_enabled`, `seed`, `clone`. **상태(`_last`)를 들고 있으므로 side 마다 하나씩 필요하다** — `clone()` 으로 복제한다. |

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
  그 결과 dex-retargeting 을 거쳐 `torch` 도 최상단에서 들어온다 — `import
  whatslab.teleop` 약 0.9초(없을 때 0.25초), Python < 3.13.

## whatslab.paths — 자산 경로 해석

- `models_root()` — URDF/메쉬 루트. `WHATSLAB_MODELS_ROOT` > `dexhand_description` 패키지 share(lazy import).
- `configs_root()` — rig/robot config 루트. `WHATSLAB_CONFIGS_ROOT` > 동봉 `whatslab/configs`.
