# whatslab API

공개 API 목록. import 루트는 `whatslab` (PEP 420 네임스페이스). 시그니처는 소스에서
추출한 것이며 `self` 는 생략한다. 개념·사용 흐름은 [README](../README.md) 참고.

의존 규칙: `core ← receiver`, `core·paths ← retarget`, `core·receiver·retarget ← hand`.
`hand` 가 맨 위이고, 아래 패키지는 화살표 방향으로만 안다.

범위는 **손**이다(입력 수신 → 로봇 손 관절각). 손목 자세·팔 IK·팔 제어는 이 SDK 에 없다 — 손목은 별도 트래커로
얻어 사용하는 쪽 팔 제어기에 넘긴다.
컴포넌트를 엮는 조립은 소비자 몫이다.

---

## whatslab.hand — 손 하나를 돌리는 최상위 API

```python
from whatslab.hand import HandModel
m = HandModel("geort:orca_right", side="right")   # 기본 입력 = AirGloveReceiver
m.start()
q = m.get_q()          # {"right": {관절: rad}} — 추적이 끊긴 틱에는 {"right": {}}
```

| 심볼 | 설명 |
|---|---|
| `HandModel(hand_config="dex:orca", side="right", urdf_root=None, hand_source=None, hand_kwargs=None, **kwargs)` | 수신기 하나 + 손 리타게팅(`HandRetargetController`). `hand_config` = `dex:<손>`(DexPilot) 또는 `geort:<설정>`(GeoRT). `hand_source` 를 안 주면 `AirGloveReceiver(**hand_kwargs)`(`receiver` extra 필요). 다른 수신기(예: `WebXRHandReceiver`)는 인스턴스로 넘긴다. 인스턴스와 `hand_kwargs` 를 같이 주면 `ValueError`. 남은 `kwargs` 는 `HandRetargetController` 로 간다. 실제 손 URDF 범위로 자르려면 `safety` 에 `SafetyFilter` 를 넣는다. |
| `HandModel.start()` / `stop()` | 수신기 시작/정지. |
| `HandModel.get_q()` | 이번 틱 `{side: {관절: rad}}`. 추적이 끊기면 손 관절이 빠지고 엔진 상태(low-pass·warm start)를 한 번 비운다. `safety` 를 넣었으면 측정 dt 로 통과시킨다. |
| `HandModel.safety` | `SafetyFilter`(또는 `step(q, dt)` 를 가진 객체). 기본 `None`. |
| `HandModel.retarget`, `HandModel.hand_source`, `HandModel.q` | 리타게팅 컨트롤러(`joint_names`, `engine` …), 수신기, 마지막 결과. 햅틱은 `hand_source.send_haptic(side, values)`. |

## whatslab.receiver — 입력 소스 (텔레옵 무관, 단독 사용 가능)

```python
from whatslab.receiver import AirGloveReceiver, WebXRHandReceiver
```

`side` = 물리적 기기의 좌/우(채널 재해석 금지). 출력은 손목 기준 OpenXR 26관절이다. Quest 는 전용 앱 없이
**헤드셋 브라우저 WebXR** 로만 받는다.

| 클래스 | `get(side)` → | 설명 |
|---|---|---|
| `WebXRHandReceiver(port=8443, listen_ip="0.0.0.0", stale_timeout=0.2, tls=True, certfile=None, keyfile=None)` | `InputSample(hand=HandPose(xr_joints=(26,8)))` | 브라우저 WebXR **핸드트래킹**. WebXR 25관절(OpenXR 26 에서 palm 만 빠진 같은 순서·축)을 받아 palm 을 합성(중지 중수골·근위 중점, 방향 = 중수골 — Spine 과 같은 규칙)하고 손목 기준으로 바꾼다. `url`, `port`, `connected(side)`. `webxr_hand_to_openxr26(joints25)` 로 변환만 쓸 수도 있다. |
| `AirGloveReceiver(glove_port=4040, listen_ip="127.0.0.1", target_ip="127.0.0.1", client_port=4042, stale_timeout=0.2, pair=1)` | `InputSample(hand=HandPose(xr_joints=(26,8)))` | **현재 Spine**(프로토콜 3) 글러브 수신(아래 「OpenXR 글러브」). `airglove_client.Client` 를 감싼다. `connected(side)`, `send_haptic(side, values)`, `spine_protocol()`, `running`, `frame(side) -> GloveFrame`(seq·수신 시각·age, 신선도 필터 없음), `device_status()`, `alarms()`, `stats()`. |

공통: `start()`, `stop()`, `get(side) -> InputSample`.

### OpenXR 글러브 (`AirGloveReceiver`)

```python
from whatslab.core.constants import XR_JOINT_NAMES
from whatslab.receiver.glove import AirGloveReceiver
rx = AirGloveReceiver()      # Spine → 127.0.0.1:4040 수신, 요청·햅틱 → 127.0.0.1:4042
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
  (1 이 기본이라 1 이면 부르지 않는다). `set_stale_threshold` 는 0.4.0 에서 효과가 없어 부르지 않는다.
  **`Client` 는 `start()` 에서 만들고**
  `stop()` 에서 닫는다 — 생성만으로는 포트를 잡지 않는다. 한 포트에 클라이언트는 하나뿐이라
  두 번째 바인드는 `airglove_client.AirGloveError`.
- `xr_joints` — 26 OpenXR `XrHandJointEXT` 관절(`XR_JOINT_NAMES`: palm, wrist, thumb
  metacarpal..tip, index..little metacarpal..tip)을 받은 그대로: 손목 공간(손목 = 단위,
  `XR_WRIST` = 1), OpenXR 관절 축(−Z = 뼈 따라 손끝, +Y = 손등, X = Y×Z, 양손 같은
  규칙), 미터. palm 은 Spine 이 합성한다. OpenXR[i+1] == WebXR[i]. **정준 변환하지 않는다.**
- 손목 — 프로토콜 3 의 손목(18/19)은 손목 자세가 아니라 손등 IMU 의 캘리브되지 않은 raw 쿼터니언이라
  받지 않는다. 손목 자세가 필요하면 별도 트래커로 얻는다.
- `tracked` = 프레임이 있고 `age_s < stale_timeout`(0 이하면 나이 검사 없음)이고
  리타게팅이 쓰는 21점(`OPENXR26_TO_HAND21`)이 모두 `valid`(palm·검지~소지 중수골은 안 본다). 관절별 유효성은
  `HandPose.xr_valid` 로 온다. 오래된 프레임도 `xr_joints` 는 그대로 싣고 `tracked=False` 만 낸다.
  `age_s` 는 클라이언트가 받은 뒤 지난 시간이다 — 프로토콜 3 에는 Spine 시각·순번이 없고
  `seq`·`sender_time_us` 는 클라이언트가 센 값·받은 시각이라 신선도 판단에 쓰지 않는다.
  `timestamp` = `time.monotonic() - age_s`.
- `connected(side)` = 2 s 안에 받은 Spine `device_status` 가 그 손을 연결로 보고하거나
  신선한 프레임이 있음. 2 s 보다 오래된 상태(Spine 이 응답을 멈춤)는 무시한다.
- `send_haptic(side, values)` → `bool`. 값은 손가락별(엄지..소지, 앞 5개까지) Spine 세기
  단계로 본다: `round` 후 `[0, 3]` 클램프(0 끔, 1 약, 2 중, 3 강), NaN 이면 0, 숫자가 아니면 예외
  (`haptic_levels`). 항상 정확히 5개를 보낸다 — 빠진 손가락은 0(빈 값 = 전부 끔).
  글러브는 0 을 받을 때까지 계속 진동한다(Spine 은 클라이언트 하트비트가 3 s 끊기면 끈다).
  시작 전이면 `False`, 전송 실패는 `AirGloveError` 를 그대로 올린다.

### WebXR 경로 (`WebXRHandReceiver`)

```python
from whatslab.receiver.webxr import WebXRHandReceiver
rx = WebXRHandReceiver()                 # 기본이 무선 = 자체 서명 HTTPS, 포트 8443
rx.start()                               # 정적 페이지 + WebSocket 을 같은 포트에서 연다
print(rx.url)                            # https://192.168.1.88:8443/
```

헤드셋 브라우저로 `rx.url` 을 열고 **VR 시작**을 누른 뒤 컨트롤러를 내려놓으면 매 XR 프레임마다 손 관절
25개(`getJointPose`, 자세 + 반지름)가 `hands` 로 올라온다(`{"t": …, "hands": {"right": [25×8]}}`).
세션은 `hand-tracking` 을 선택 기능으로 요청한다.

- **패스스루.** 세션은 `immersive-ar` 를 먼저 시도하고(Quest 패스스루, `alpha-blend`) 안 되면
  `immersive-vr` 로 떨어진다.
- **보안 컨텍스트가 필요하다.** WebXR 은 HTTPS 이거나 `localhost` 에서만 뜬다.
  - **무선(기본, `tls=True`)**: 자체 서명 인증서를 `~/.cache/whatslab/webxr/` 에 만들어
    HTTPS + WSS 로 연다. URL 호스트는 이 PC 의 사설 IP 중 192.168 → 10 → 172 순으로 고른다(docker 브리지 172.x 를 피하려고).
    인증서 SAN 에 `localhost` 와 이 머신의 모든 전역 IPv4 가
    들어간다(`openssl` 실행파일 필요, 유효기간 10년, 키 퍼미션 0600). IP 가 바뀌면
    자동으로 재발급한다. **헤드셋에서 최초 1회 인증서 경고를 수락**해야 한다 —
    페이지와 WebSocket 이 같은 오리진(같은 포트)이라 한 번 수락하면 WSS 도 통과한다.
    직접 만든 인증서를 쓰려면 `certfile=`/`keyfile=` 을 넘긴다.
  - **유선**: `tls=False` + `adb reverse tcp:8443 tcp:8443` — 헤드셋이 PC 서버를
    `localhost` 로 본다(인증서 불필요). URL 도 이때만 `localhost` 가 된다.
- 전송은 WebSocket(TCP)이라 UDP 처럼 늦은 프레임을 버리지 않는다. 브라우저 쪽에서
  `bufferedAmount` 가 밀리면 그 프레임을 **보내지 않고 버린다**(지연 누적 방지).
- 서버는 `websockets`(`webxr` extra)의 동기 서버다. 같은 포트에서 WebSocket 이 아닌 요청에는 페이지를 준다.
  메시지가 JSON 객체가 아니거나 처리 중 예외가 나면 그 연결만 닫히고(로그), 페이지가 다시 붙는다.
  ping·무응답 끊기는 라이브러리 기본값(20 s)이다.

## whatslab.retarget — 손 리타게팅

```python
from whatslab.retarget import HandRetargetController, XRHandRetargeter
```

엔진은 둘이고 입력은 같다 — `HandPose.xr_joints` `(26, 8)`(OpenXR 26관절). `config_name` 의
접두사로 고른다. 그 밖의 이름은 `ValueError`.

- **`dex:<이름>`** — dex-retargeting(0.4.6 이상)의 DexPilot(손끝·손가락 사이 벡터 + 핀치 투영). URDF 만 있으면
  되고 `hand` extra(pinocchio·dex-retargeting·torch)가 필요하다.
- **`geort:<이름>`** — GeoRT 소프트 프롬프트 신경망(numpy 만). `configs/geort/<이름>.yaml` 또는 yaml 경로.
  학습된 모델 npz 가 필요하다(아래 `whatslab.retarget.geort`). 모델의 손 좌우는 로봇 손 기준이다 —
  오른손 모델(동봉 전부)은 왼손 사람 입력을 반전해 풀 수 있지만 왼손 로봇 손은 구동하지 못한다.

| 심볼 | 설명 |
|---|---|
| `HandRetargetController(hand_type, config_name="dex:orca", urdf_root=None, **kwargs)` | 손 리타게팅 컨트롤러(`HandController`). 엔진(`XRHandRetargeter` / `GeortHandSolver`)은 계산만 하고, 공통 정책은 여기 한 곳에 있다: 추적이 없거나 이번 프레임을 못 풀면(`xr_input.HandInputError` — geort 의 `GeortInputError` 도 이것이다. 다른 예외는 그대로 올라온다) 직전 명령 유지(못 푼 이유는 `last_error`, 연속 실패의 첫 번에 경고 로그), 추적이 끊기는 순간 엔진 상태 초기화(`reset()`), 첫 입력 전 명령은 0 을 관절범위로 자른 값. `engine_name` = `"dex:"` / `"geort:"`. `robot_side` = 엔진이 내는 관절각의 로봇 손 side(dex 는 설정 side, geort 는 모델 side — 오른손 모델은 왼손 사람 입력에도 `"right"`). `HandPose.xr_valid` 가 거짓인 관절은 두 엔진 모두 쓰지 않는다(쓰는 관절이 무효면 그 프레임은 직전 명령 유지). `last_error` 는 지금 프레임 기준(추적이 없으면 `None`). `geort:<이름>` 은 동봉 `configs/geort/<이름>.yaml`(현재 디렉토리의 같은 이름 파일이 아니라), `.yaml` 로 끝나면 사용자 경로. `urdf_root` 는 dex 만 쓰고 geort 에서는 무시한다. 나머지 `kwargs` 는 엔진으로 간다. |
| `engine_of(name)` | `"dex:"` / `"geort:"` 접두사. 아니면 `ValueError`. |
| `XRHandRetargeter(config, side=None, urdf_root=None, max_evals=None, **overrides)` | OpenXR → 로봇 손 DexPilot. `config` = `configs/hands` 이름 / `dex:` 이름 / yaml 경로 / `DexHandConfig`. `side` 가 config 의 side 와 다르면 입력을 거울상(`mirror_xr_joints`)으로 바꿔 쓴다. `compute(xr_joints) -> q`(`joint_names` 순서, rad). `joint_names`, `joint_limits` `(n, 2)`, `robot_joint_names`(고정·mimic 관절 포함), `full_qpos(q)`(고정 관절 값 + URDF mimic 관절을 채운 전 관절), `human_keypoints(xr)`(MANO 21점, 로봇 루트 축), `scaling_factor`, `reset()`. 손당 프레임 p50 1~2ms, p99 약 4ms. |
| `DexHandConfig.load(name, side=None)` / `from_dict(d)` | `configs/hands/*.yaml` 로더(아래 키). `retargeting` 에 dex-retargeting 이 모르는 키가 있으면 `ValueError`. |
| `hand_config_names()` | 동봉 `configs/hands` 이름 목록. |
| `xr_to_mano(xr_joints)` | OpenXR 26 → MANO 21 손목 상대 위치. 인덱스 `[1,2,3,4,5,7,8,9,10,12,13,14,15,17,18,19,20,22,23,24,25]`, 축 `[[0,-1,0],[-1,0,0],[0,0,-1]]`(양손 같음 — 손목 공간 위치를 손목 회전으로 한 번 더 풀어 손목 자세와 무관하다). |
| `mirror_xr_joints(xr_joints)` | 손목 공간 x 축 거울상(왼손 ↔ 오른손). 두 번 하면 원래대로. |
| `xr_input.masked_keypoints(xr_joints, xr_valid)` | 무효 관절 위치를 NaN 으로 — 엔진에 넣기 전에 쓴다(`HandRetargetController` 는 알아서 한다). |

### 손 리타게팅 config (configs/hands)

`dex:<이름>` 이 읽는 yaml. `<이름>.yaml` 을 먼저, 없으면 `<이름>_{side}.yaml` 을 찾는다.

| `dex:` 이름 | URDF (models root 기준) | 손목 링크 → 손끝 링크 | 비고 |
|---|---|---|---|
| `orca` | `orca_hand/urdf/{side}.urdf` | Carpals → `{side}_sensor_*_distal` ×5 | 손목 관절 0 고정 |
| `dg5f` | `tesollo_dg5f/dg5f_{side}.urdf` | `?l_dg_palm` → `?l_dg_[1-5]_tip` | PIP·DIP −5..90° |
| `robotis` | `robotis_hx5_d20/urdf/hx5_d20_{side}.urdf` | `hx5_d20_{side}_base` → `finger_end_{r,l}_link[1-5]` ×5 | URDF 에 센서 링크가 없어 손가락 끝 링크를 쓴다 |
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
| `fixed_joints` | `{관절: rad}` — 리타게팅에서 빼고 고정한다. ORCA 손목 관절은 손 리타게팅 대상이 아니라 0 으로 둔다. |
| `joint_limits_deg` | `{관절 이름 glob: [lo, hi]}` — 최적화 한계를 덮어쓴다. DG-5F 손가락 PIP·DIP 는 URDF 가 ±90° 라 역방향으로 꺾는 해가 나와 −5..90° 로 자른다. |
| `human_hand_length` | 사람 손목 → 중지 끝(m, 기본 0.19). `retargeting.scaling_factor` 가 없으면 로봇 같은 길이(q=0)를 이것으로 나눠 쓴다. |
| `max_evals` | 틱당 SLSQP 평가 상한(동봉값 20). 무제한과 결과는 거의 같고 p99 지연만 줄인다. |
| `retargeting` | dex-retargeting `RetargetingConfig` 그대로(`type`, `wrist_link_name`, `finger_tip_link_names`, `low_pass_alpha`, …). `urdf_path` 는 위 `urdf` 로 채우고 `target_joint_names` 를 안 주면 `fixed_joints` 를 뺀 전 관절이다. URDF 에 mimic 관절이 있으면 `target_joint_names` 에서 빼서 줘야 한다(dex-retargeting 이 mimic 으로 채운다 — ability·schunk). |

## whatslab.retarget.geort — GeoRT 소프트 프롬프트

```python
from whatslab.retarget import HandRetargetController
ctrl = HandRetargetController("right", "geort:orca_right")     # 텔레옵용 (직전 값 유지·초기화 포함)

from whatslab.retarget.geort import GeortHandSolver
from whatslab.retarget.xr_input import masked_keypoints
solver = GeortHandSolver.from_config("geort/orca_right.yaml", human_side="right")
q = solver.solve(masked_keypoints(xr_joints, xr_valid))       # 계산만
```

GeoRT 소프트 프롬프트 모델(여러 손이 공유하는 신경망 + 손별 프롬프트·출력층)의 numpy 추론이다.
학습 도구는 이 SDK 에 없다. 지원하는 모델은 하나의 조합이다 — 모델 구조 `format = geort-prompt-v1`(MLP + 손별
soft prompt), 손 입력 형식 `input_frame = openxr-wrist-kabsch-v2`(손목 원점 + Kabsch 정렬). 다른 값이면
`ValueError`. 설정·모델 파일이 없으면 `FileNotFoundError` 가 그대로 올라온다.

| 심볼 | 설명 |
|---|---|
| `GeortHandSolver.from_config(path, human_side="right", low_pass_alpha=None)` | 설정 YAML 로 솔버를 만든다. `low_pass_alpha` 를 주면 설정값 대신 쓴다. |
| `GeortHandSolver(embodiment, human_side="right", low_pass_alpha=1.0)` | 26×8 → q (`solve`, 모델 학습 범위로 자름). 계산만 한다 — 직전 값 유지·추적 끊김 처리는 `HandRetargetController`. 모델 손과 반대쪽 입력은 좌우 반전해 푼다. 실제로 쓰는 21점의 위치가 유한하지 않으면 `GeortInputError`(`masked_keypoints` 가 무효 관절을 NaN 으로 바꾼다 — palm·중수골은 안 쓴다). `reset()`, `joint_names`, `joint_limits`. |
| `read_config(path)` | 설정 YAML: `name`(모델 안 손 이름), `side`, `model`(생략 시 `$WHATSLAB_GEORT_MODEL` → 패키지 동봉 `retarget/geort/assets/geort_prompt.npz`), `low_pass_alpha`. 설정의 상대경로는 설정 파일 기준, 환경변수의 상대경로는 현재 디렉토리 기준. |
| `load_prompt_model(path)` | npz → `PromptModel`(`embodiments`). 형식이 위 조합이 아니거나 배열 모양이 틀리면 `ValueError`. |
| `keypoints` 모듈 | OpenXR 26 → MediaPipe 21 → `openxr-wrist-kabsch-v2` 입력 좌표계(`InputFrame`, `canonical_points`, `to_geort` …). |
| `GeortConfigError` / `GeortInputError` | `code`·`cause`·`remedy` 를 가진 `ValueError`. `GeortConfigError` 는 설정 내용 검사(필수 키·값 범위·모델 안 손 이름), `GeortInputError` 는 이번 프레임 입력 오류이자 `HandInputError`. |

동봉 설정(`configs/geort/`): `orca_right`(ORCA 오른손 16축), `hx5_right`(ROBOTIS HX5-D20 오른손 20축). 관절 이름은
dexhand-description URDF 와 같다(테스트로 확인). 모델에는 Allegro·DG-5F-S 프롬프트도 있지만 관절 이름이 아직
dexhand-description 에 없는 URDF 기준이다.

## whatslab.core — 계약(타입 + Protocol), 의존성 0

공유 상수는 `whatslab.core.constants` 가 단일 출처다 — `SIDES`, `FINGER_COUNT`, OpenXR 26관절
(`XR_JOINT_NAMES`·`XR_JOINT_COUNT`·`XR_JOINT_FIELDS`·`XR_WRIST`), 사람 손 21점(`OPENXR26_TO_HAND21`,
`HAND21_WRIST`·`HAND21_*_MCP`·`HAND21_TIPS`), `OPENXR_TO_HAND21_AXES`. 한 모듈만 쓰는 상수는 그 모듈에 두고,
두 곳 이상이 쓰면 여기로 옮긴다.

| 심볼 | 설명 |
|---|---|
| `types.HandPose` | `xr_joints`(`(26,8)` OpenXR 26관절, 손목 공간 — 손 리타게팅 입력) + `xr_valid`(`(26,)` bool) + `tracked`·`timestamp`. |
| `types.InputSample` | 리시버 출력 컨테이너(`hand`, `tracked`, `timestamp`). |
| `types.HandCommand` | 리타게팅 출력(로봇 손 관절각). |
| `interfaces.Receiver` / `HandController` | 컴포넌트 Protocol(구조적 타이핑). |

## whatslab.viz — viser 웹 3D 시각화 (`whatslab-sdk[viz]`)

`get_server(port=8080)` 로 포트당 서버 공유(여러 viz 공존). `http://localhost:8080`.

| 클래스 | 내용 |
|---|---|
| `URDFScene` | URDF 하나를 메쉬(STL)/스켈레톤 자동 판별 렌더 + 관절 구동. `set_root`, `q_from_named`, `fk`. |
| `HandViz(urdf, joint_names, port=8080, root_path="/hand", root_pose=None)` | 로봇 손 URDF 하나를 q 로 구동한다(리타게팅 결과 확인). 사람 손 시각화는 Spine 이 맡는다. |

## whatslab.safety — 운동학 안전 유틸 (dep-light)

로직만 제공. 강제(watchdog/e-stop 배선)는 소비자(ROS safety_gate 등)가, 최종 권위는 하드웨어가 갖는다.

| 심볼 | 설명 |
|---|---|
| `JointLimit` | 관절 pos/vel 한계. |
| `load_limits_from_urdf(urdf_xml)` | URDF → `{joint: JointLimit}`. |
| `tighten(base, ...)` | 한계를 보수적으로 조임. |
| `SafetyFilter` | clamp + rate-limit + hold/estop 상태기. `step(desired, dt=None)`, `trip`, `reset`, `estopped`, `enabled`, `set_enabled`, `seed`. **상태(`_last`)를 들고 있으므로 손마다 하나씩 만든다.** |

## whatslab.paths — 자산 경로 해석

- `models_root()` — URDF/메쉬 루트. `WHATSLAB_MODELS_ROOT` > `dexhand_description` 패키지 share(lazy import).
- `configs_root()` — 손 리타게팅 설정(`configs/hands`, `configs/geort`) 루트. `WHATSLAB_CONFIGS_ROOT` > 동봉 `whatslab/configs`.
