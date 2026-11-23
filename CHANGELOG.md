# Changelog

이 저장소는 [Semantic Versioning](https://semver.org/lang/ko/) 을 따른다.
1.0 이전이므로 minor 버전에서 호환 없는 변경이 있을 수 있다.

## [0.5.0] — 2026-11-19

Spine 3.0.0(OSC 프로토콜 3) 대응을 마무리하고 손 리타게팅을 OpenXR 26관절 입력 하나로 통일한다.
아래 [Unreleased] 항목(글러브 OpenXR 수신·DexPilot)도 이 버전에 들어간다.

### 변경 — 이름 (호환 없는 변경)

- **`GloveOpenXRReceiver` → `AirGloveReceiver`**, 모듈 `receiver.glove.openxr` → `receiver.glove.airglove`.
  `GloveModel(hand_source=…)` 키도 `"openxr"` → `"airglove"`.

### 변경 — Quest 입력을 브라우저 WebXR 로 일원화 (호환 없는 변경)

- **`WebXRHandReceiver` 추가** — 헤드셋 브라우저의 WebXR 핸드트래킹(25관절)을 OpenXR 26관절(`HandPose.xr_joints`,
  palm 은 Spine 과 같은 규칙으로 합성)과 손목 자세로 준다. 글러브와 같은 dex·geort 리타게팅을 맨손으로 쓸 수 있다.
  웹 페이지(`receiver/webxr/static/index.html`)는 `hand-tracking` 을 선택 기능으로 요청하고 `hands` 를 보낸다.
- **WebXR 서버가 메시지를 모든 수신기에 준다**(`set_handler` → `add_handler`/`remove_handler`). 같은 포트에서
  컨트롤러·손 수신기를 함께 쓸 수 있다. 서버를 함께 쓰려면 고정 포트를 준다(`port=0` 은 공유하지 않는다).
- **제거:** Quest 앱(PoseDataTracker) 경로 — `receiver/quest`(`QuestControllerReceiver`·`QuestHandReceiver`),
  `receiver/osc_transport`, `python-osc` 의존성, `assets/quest/*.apk`, `scripts/install_quest_app.sh`, 앱 전용
  `UNITY_TO_CANONICAL`·컨트롤러 마운트 보정(`CONTROLLER_POS_OFFSET`, WebXR grip 에는 해당 없음 — 필요하면
  `pos_offset`). 앱은 손가락 16관절만 보내고 손끝이 없어 리타게팅 입력을 만들 수 없었다.
- `QuestModel` 은 `WebXRHandReceiver`(손목 → 팔, 손 → 리타게팅), `GloveModel` 의 `arm_source` 는 `"webxr"` 만
  (기본). `examples/quest_arm.py` 의 `--arm-source`·`--hand-config` 옵션 제거.
- 알려진 제약: WebXR 손목 축(OpenXR)이 앱(Unity OVR)과 달라 `QuestModel` 팔 EE 방위는 실기 재확인이 필요하다.

### 변경 — 의존성

- **`robot` extra(pyAgxArm) 제거** — `src/` 는 쓰지 않고 `examples/robot_io.py` 만 쓰는 하드웨어 드라이버라
  `examples/requirements-robot.txt` 로 옮겼다. `scripts/install_robot_deps.sh` 가 함께 설치한다.

### 추가 — GeoRT 모델 동봉

- 학습된 GeoRT 소프트 프롬프트 모델(`geort_prompt.npz`, 약 5MB — orca_right·hx5_right·allegro_right·dg5f_s_right)을
  `solvers/hand/geort/assets/` 에 넣었다. 모델 경로는 설정의 `model` > `$WHATSLAB_GEORT_MODEL` > 동봉 모델 순이다.
- `whatslab.solvers` 도 지연 import 한다 — 전에는 `whatslab.solvers.hand.geort` 만 써도 패키지 `__init__` 이 팔
  IK(pinocchio)를 불러왔다. 이제 numpy·scipy·pyyaml 만 있는 환경에서 GeoRT 리타게팅이 돈다(확인함).

### 수정 — 코드 리뷰

- **손 컨트롤러 구조:** 엔진(`XRHandRetargeter`, `GeortHandSolver`)은 계산만 하고, 직전 값 유지·추적 끊김 시 초기화·
  `last_error`·관절범위로 자른 초기값은 `HandRetargetController` 한 곳에 있다(엔진별로 중복·차이가 나던 것).
  `GeortHandController`(·`compute_q`)는 제거 — 텔레옵은 `HandRetargetController(side, "geort:…")`, 계산만은
  `GeortHandSolver.from_config(...).solve(masked_keypoints(...))`.
- **안전:** rig 경로(`RobotModel.make_hand_controller`)의 geort 출력을 실제 손 URDF 관절 범위로도 자른다
  (`RobotModel.hand_joint_limits()` → `robot_limits`). 전에는 학습 모델 범위로만 잘랐다. geort 의 첫 명령(입력 전)도
  0 이 아니라 관절범위로 자른 값이다.
- WebXR: 수신기가 `start()` 에서 서버에 핸들러를 붙이고 `stop()` 에서 뗀다(버린 수신기가 계속 메시지를 받던 누수),
  서버는 핸들러마다 예외를 격리한다(한 수신기의 예외가 다른 수신기·연결을 끊던 문제), `WebXRControllerReceiver`
  `stale_timeout` 기본 0.2 s(손 추적으로 바뀌어 grip 이 끊겨도 마지막 자세를 추적 중으로 내던 문제).
- `geort:<이름>` 은 동봉 설정을 쓴다 — 현재 디렉토리에 같은 이름의 `geort/<이름>.yaml` 이 있어도 그것을 읽지 않는다.
- geort 에 `urdf_root` 를 주면 에러 대신 무시한다(호출자가 엔진을 알 필요 없게).
- 추적이 끊기거나 입력 오류가 나면 low-pass·warm start 상태를 비운다 — 다시 잡혔을 때 몇 초 전 자세와 섞이던 문제.
- GeoRT 는 실제로 쓰는 21점만 유한성을 검사한다 — 쓰지 않는 palm·검지~소지 중수골이 무효여도 프레임을
  버리지 않는다(엄지 중수골은 21점에 들어간다).
- `$WHATSLAB_GEORT_MODEL` 의 상대경로는 현재 디렉토리 기준이다(설정 yaml 의 `model` 만 설정 파일 기준).
- `examples/airglove_hand.py --config geort:…` 가 `WHATSLAB_MODELS_ROOT` 환경에서 `urdf_root` 때문에 죽던 문제.
- `QuestModel` 팔 목표의 방위도 위치와 같은 `pos_frame` 기준이다 — `WebXRHandReceiver` 가 `hmd_yaw` 에서 위치만
  머리 yaw 를 빼고 방위는 방 기준으로 내 한 `Pose` 안에 두 좌표계가 섞였다(고개를 돌리면 EE 방위가 그만큼 틀어짐).
  `GloveModel` 도 같은 규약이다 — 전에는 `pos_frame` 과 관계없이 방위에서 머리 yaw 를 뺐다(`room`·`hmd` 에서는
  위치는 방/머리 기준인데 방위만 머리 yaw 기준). `WebXRControllerReceiver.get()` 의 `controller.quat` 도
  이제 `pos_frame` 기준이다(**동작 변경** — 전에는 `hmd_yaw` 에서도 방 기준 방위를 냈다).
- 텔레옵 경로에서 손 추적이 끊기면 `SideModel` 이 리타게팅 엔진 `reset()` 을 부른다 — 끊긴 프레임에는 `compute()` 를
  부르지 않아 위의 「추적이 끊기면 상태를 비운다」가 텔레옵에서는 실행되지 않았다.
- GeoRT `robot_limits` 의 관절 이름이 모델과 하나도 안 맞으면 `GEORT_LIMITS_UNMATCHED` 오류, 일부만 맞으면 경고 —
  전에는 URDF 범위 클램프가 조용히 꺼졌다. 오른손 GeoRT 모델을 왼손 로봇 손 rig 에 쓰면 이 오류가 난다(미지원).
- WebXR 수신기 `start()`/`stop()` 은 여러 번 불러도 한 번만 동작한다 — 같은 수신기를 두 번 시작하면 서버 참조 수가
  2 가 돼 `stop()` 한 번으로 포트가 풀리지 않던 문제(`examples/quest_arm.py` 가 그렇게 불렀다).
- GeoRT 모델 파일을 경로·수정시각별로 한 번만 읽는다(양손 rig 가 5MB npz 를 side 마다 두 번씩 열던 것).
- `TeleopModel.samples()`: 한 틱에 수신기를 side 마다 한 번만 읽는다(QuestModel 은 같은 수신기를 3번 읽어 팔·손의
  tracked 판정 시각이 달랐다). 직접 만든 `_get_raw_target()` 도 수신기 대신 이것을 쓰면 된다.
- `MANIFEST.in` 이 삭제된 `solvers/hand/assets` 대신 `solvers/hand/geort/assets` 를 담는다.
- 틱 샘플(`samples()`)은 스레드별이다 — 캘리브 스레드(`calibrate_yaw`/`calibrate_reach`)와 메인 루프가
  동시에 `get_data()` 를 부르면 한쪽이 다른 쪽 틱을 비워 `TypeError` 로 죽을 수 있었다.
- WebXR 서버는 bind 가 성공한 뒤에만 참조 수를 센다 — 포트가 점유돼 `start()` 가 실패한 뒤 다시 시작하면
  `stop()` 으로 포트가 풀리지 않던 문제. 실패한 수신기는 핸들러도 뗀다.
- `GloveModel`: 컨트롤러 추적이 끊기면(`stale_timeout`) 마지막 자세를 팔 목표로 내지 않는다.
- `make_hand_controller` 는 geort 일 때만 rig 손 URDF 범위를 읽는다(dex 는 자기 설정의 URDF 를 쓴다).
- `whatslab.receiver` 는 하위 패키지를 쓸 때 불러온다 — WebXR 만 쓰면 airglove-client 가 없어도 된다.
  `whatslab.teleop` 도 같다 — `GloveModel` 은 쓸 때 불러오고 `HandModel` 은 `hand_source` 를 안 줄 때만
  `AirGloveReceiver` 를 불러온다(전에는 `from whatslab.teleop import QuestModel` 도 airglove-client 를 요구했다).
  두 패키지의 `__all__`(star import)에는 airglove-client 가 필요한 이름이 없다.
- 캘리브(`calibrate_yaw`·`calibrate_reach`·`set_reach`)는 진행 중인 `get_q()` 틱(보정 적용·IK·안전필터)이
  끝난 뒤에 적용된다 — 캘리브 스레드가 IK 도중 `reseed()` 하거나 새 yaw 와 옛 원점을 섞어 읽게 할 수 있었다.
  캘리브가 읽은 샘플은 `SideModel.raw_target` 에 쓰지 않는다(메인 루프의 진단·`--dump-targets` 행이 섞이던 문제).
- `HandRetargetController.compute` 는 `GeortInputError`(이번 프레임 입력 오류)만 삼킨다 — 전에는 모든
  `ValueError`(입력 모양 오류 같은 버그 포함)를 삼켜 손이 첫 명령에 멈춘 채 아무 로그도 없었다. 연속 실패의 첫 번에
  경고를 남긴다. dex 도 같은 정책이다 — 퇴화(손바닥 프레임을 못 정함)·비유한 입력이나 비유한 해는
  `xr_input.HandInputError` 로 이번 프레임만 거른다. 전에는 퇴화 입력이 `ValueError` 로 텔레옵 루프를 죽이고,
  NaN 입력은 NaN 관절각을 그대로 냈다.
- dex 도 `HandPose.xr_valid` 를 따른다 — 전에는 무효 관절(원점·항등으로 남은 값)이 그대로 DexPilot 에 들어갔다.
  두 엔진 모두 무효 관절 위치를 NaN 으로 가리고, 쓰는 관절이 무효면 그 프레임은 직전 명령을 유지한다(`last_error`).
- `mirror_xr_joints` 는 0·NaN 쿼터니언(무효 관절)을 항등으로 둔다 — 반대 손 dex 설정에서 무효 palm 이 0 쿼터니언이면
  `ValueError` 로 텔레옵 루프가 죽었다.
- WebXR `hmd_yaw` 의 머리 yaw 는 머리 앞 축을 바닥에 투영한 방향이다(롤 무관). 정확히 위·아래를 봐 투영이 0 이면
  위 축의 수평 방향을 쓴다 — 전에는 그때 yaw 가 정의되지 않았다. 피치가 ±90° 에 가까우면 머리를 조금만 움직여도
  yaw 가 크게 바뀐다(알려진 제약 — 손을 거의 수직으로 내려다보며 고개를 기울이는 경우).
- `examples/quest_arm.py` 의 다른 IP 후보가 URL IP 의 접두사인 주소(192.168.0.1 vs .10)를 빠뜨리던 문제.
  `scripts/install_robot_deps.sh` 는 저장소 루트에서 돈다(다른 디렉토리에서 실행하면 `-e .` 가 엉뚱한 곳을 가리켰다).
- `TeleopModel.start()` 는 수신기 하나가 시작에 실패하면 그 앞에서 시작한(이미 돌던 것 포함) 수신기를 멈추고 예외를 올린다 — 글러브
  UDP 소켓이 열린 채 남아 같은 프로세스에서 다시 시작할 수 없었다. 수신기 `start()`·`stop()` 은 여러 번 불러도
  한 번만 동작해야 한다(`Receiver` 프로토콜 규약). 수신기 인스턴스 하나를 두 모델이 함께 쓰는 것은 지원하지 않는다.
  WebXR 수신기에 `running`(시작했고 아직 멈추지 않음) 추가.
- `scripts/install_robot_deps.sh` 의 마지막 확인이 numpy 1 을 요구하던 것 — 저장소에 numpy<2 핀이 없어 새 환경
  (numpy 2)에서 설치를 다 하고도 실패했다. `examples/quest_arm.py` 는 나머지 준비가 끝난 뒤 수신기를 시작한다.
- `safety.load_limits_from_urdf`: `<limit>` 에 `lower`·`upper` 가 없으면 URDF 사양대로 0(continuous 관절만 ±inf).
  전에는 ±inf 로 읽어 geort 출력의 URDF 범위 클램프가 그 관절에서 꺼졌다.
- `HandRetargetController.reset()` 은 `last_error` 도 비운다(텔레옵은 추적이 끊기면 `reset()` 만 부른다).
- `HandRetargetController.last_error` 는 지금 프레임 기준이다 — 추적이 없는 프레임에서는 `None`(전에는 이전 오류가 남았다).
- `HandRetargetController.robot_side` 추가 — 엔진이 내는 관절각의 로봇 손 side(dex: 설정 side, geort: 모델 side).
  오른손 geort 모델은 왼손 사람 입력에도 `"right"` 이므로 호출자가 URDF side 를 추측하지 않아도 된다.
- WebXR 서버: 같은 핸들러의 예외 트레이스백은 처음 한 번만 남긴다(전에는 프레임마다 — 72–90 Hz). 핸들러 예외는
  `stats` 의 버린 프레임이 아니라 `WebXRServer.handler_errors` 로 센다.
- WebXR 페이지 송신 버퍼 상한 8 KB → 64 KB — 양손 메시지(≈4 KB)는 수십 ms 의 Wi-Fi 지연에도 프레임을 버렸다.
- WebXR 서버: 프레임 헤더만 오고 payload 가 유휴 ping 주기(5 s)보다 늦으면 읽은 헤더를 버려 다음 프레임부터
  payload 를 헤더로 읽던 문제(연결 끊김) — 읽은 바이트를 되돌려 다시 읽는다.
- WebXR 서버가 클라이언트 프레임 마스크를 바이트별 파이썬 루프 대신 정수 XOR 로 푼다(손 메시지는 수 KB × 72–90 Hz).
- **제거(호환 없는 변경):** `WebXRReceiverBase.to_canonical(pos, quat)`·`to_canonical_quat(quat)` 메서드 —
  `receiver.base.to_canonical_pose(Y_UP_RH_TO_CANONICAL, pos, quat)`·`to_canonical_quat(Y_UP_RH_TO_CANONICAL, quat)`
  를 쓴다. 컨트롤러 수신기의 `_relative` 는 기반 클래스의 `relative()`·`relative_quat()` 가 됐다.
- `AirGloveReceiver`: `tracked` 는 리타게팅이 쓰는 21점이 모두 유효한지로 정한다 — 전에는 26관절 전부라 안 쓰는 palm·
  중수골 하나만 무효여도 손이 멈췄다. 관절별 유효성은 그대로 `xr_valid` 로 온다.
- `WebXRReceiverBase.__init__` 의 `pos_frame` 은 키워드 전용이다 — 위치 인자 순서는 0.4 와 같은
  `(port, listen_ip, tls, certfile, keyfile)`.
- `examples/quest_arm.py`: yaw 캘리브가 실패해도 「완료」를 찍던 문제 — side 별 결과를 찍는다.
- `examples/quest_arm.py --arm wrist` 는 airglove-client 없이 돈다(`GloveModel` 은 controller 모드에서만 불러온다).
- WebXR: 모양이 틀리거나 유한하지 않은 `hmd`·컨트롤러 항목(`pos` 3개·`quat` 4개가 아님)은 버린다 — 저장했다가
  이후 모든 `get()` 이 메인 루프에서 죽던 문제.
- **WebXR `port=0` 서버는 공유하지 않는다(동작 변경)** — 수신기마다 따로 임시 포트를 잡고 레지스트리에 넣지 않는다
  (전에는 `port=0` 수신기끼리 서버를 공유했다). 서버를 함께 쓰려면(컨트롤러·손 수신기) 고정 포트를 준다. 한 번 받은
  임시 포트는 다시 시작해도 유지한다(페이지가 그 포트에 붙어 있다 — 그 사이 다른 프로세스가 잡았으면 `start()` 가
  포트 사용 중 오류를 낸다). **제거:** `ws_server.clear_registry()`(테스트 전용이었다).
- 같은 포트의 WebXR 수신기를 다른 설정(`listen_ip`·`tls`·인증서)으로 만들면 `ValueError` — 전에는 먼저 만든
  서버 설정을 조용히 써 `tls=True` 를 줘도 평문으로 열릴 수 있었다.
- `examples/airglove_hand.py` 가 한 틱에 수신기를 두 번 읽던 것(추적 표시와 q 가 어긋날 수 있었다),
  `examples/quest_arm.py` 상태 줄의 손 관절 수가 팔 해가 없을 때 틀리던 것.
- WebXR 서버를 멈추고 곧바로 다시 시작할 때: 옛 리슨 소켓이 아직 열려 있어 bind 가 실패할 수 있었고, 멈추기 전에
  붙은 페이지 연결이 끊기지 않고 지난 세션의 자세를 새 실행의 수신기에 계속 넘겼다.
- WebXR 수신기 `start()` 가 bind 외의 이유(인증서 생성 실패 등)로 실패해도 서버에서 핸들러를 뗀다.
- `whatslab.solvers`·`whatslab.solvers.hand` 의 `__all__` 에는 numpy 만 쓰는 이름만 둔다 — star import 가
  pinocchio·dex-retargeting 을 요구하지 않는다.
- GeoRT `prompt.FINGERS` 를 `core.constants.FINGER_COUNT` 로 바꿨다.
- `docs/API.md` 예제가 `receiver.glove` 에서 `XR_JOINT_NAMES` 를 import 하던 것 — `whatslab.core.constants` 에서 가져온다.
- 좌표계 헬퍼 통합: `receiver.base.conjugate`·`to_canonical_quat`·`to_canonical_pose`(같은 `M R Mᵀ` 가 4곳에 있었다),
  상수 배열용 `core.constants.readonly_array`(두 벌이던 `_frozen`).
- `HandPose` 필드는 `wrist, xr_joints, xr_valid, tracked, timestamp` 순서다 — 위치 인자로 만들던 코드는 키워드 인자로
  바꿔야 한다. 쓰지 않는 import·중복 import·낡은 주석 정리.

### 변경 — PEP 8 정리 (호환 없는 변경)

- `pyproject.toml` 에 ruff 설정(`E`·`W`·`F`·`I`·`N`, 줄 길이 99 — PEP 8 이 허용하는 팀 상한)을 넣고
  `src`·`tests`·`examples`·`tools` 전체가 통과하게 했다(`ruff check`).
- 이름(PEP 8): `Origin.T` → `Origin.transform`, `robot.config.origin_to_T` → `origin_to_transform`.
  행렬 인자·지역 변수도 소문자다 — `RobotModel.to_base(tf_canonical)`·`to_canonical(tf_base)`·
  `clamp_reach(tf_base)`, `receiver.base.conjugate(basis, rot)`·`to_canonical_quat(basis, q)`·
  `to_canonical_pose(basis, …)` 등. 위치 인자로 부르던 코드는 그대로 동작하고, 키워드로 넘기던
  코드만 이름을 바꿔야 한다.
- `tools/export_combined_usd.py`: 선택 의존성 `pxr.PhysxSchema` 를 `importlib` 로 읽는다.

### 수정 — 기존 테스트 실패

- `dex:robotis` 가 시작하자마자 `URDF 에 없는 링크: right_sensor_middle_distal` 로 죽던 문제 —
  upstream dexhand_description 의 HX5-D20 URDF 에는 센서 링크가 없다. 손끝을 두 URDF(upstream·센서 패치판)에 모두
  있는 손가락 끝 링크 `finger_end_{r,l}_link1..5` 로 바꿨다. 센서 패치판 URDF 를 쓰던 환경은 손끝 기준점이 센서
  패드에서 손가락 끝 링크로 바뀌어 DexPilot 목표·자동 `scaling_factor` 가 조금 달라질 수 있다.
- `test_neutral_is_mid_range_and_posture_pull_is_on` 이 rig `solver.posture`(0.4.x 에서 추가) 이전
  기대값(자세 목표 = 범위 중앙)을 검사하던 것 — rig 에 posture 가 있으면 그것이 자세 목표다.
  별도 테스트 `test_rig_posture_overrides_neutral_as_posture_target` 로 나눴다.

### 추가 — GeoRT 소프트 프롬프트 (`whatslab.solvers.hand.geort`)

- robotPlatform `plugins/hand_solvers/geort` 의 추론(소프트 프롬프트 모델 읽기, numpy 순전파, 입력 좌표계)을
  옮겼다. robotPlatform 은 이 모듈을 정본으로 쓴다. 실제 Spine 캡처 794프레임에서 원본과 출력 차이 최대 1.2e-14 rad.
- rig `retarget: geort:<이름>`(`configs/geort/<이름>.yaml`). 동봉: `orca_right`, `hx5_right`.
  모델 npz 는 패키지에 동봉한다(아래 「추가 — GeoRT 모델 동봉」).
- `HandPose.xr_valid`(관절별 유효), `HandRetargetController.engine_name` / `last_error`, `engine_of()`.

### 제거 (호환 없는 변경)

- **관절각 ONNX 엔진 `UniRetargeter`** 와 자산(`uni_all.onnx`·`uni_igris_right.onnx`·`uni_tables*.npz`),
  `examples/run_retarget.py`. 입력인 사람 손 URDF 관절각을 Spine 3 이 보내지 않아 실시간 입력이 없었다.
  `retarget:` 은 `dex:` / `geort:` 접두사만 받는다. `hand` extra 에서 onnxruntime 을 뺐다.
- rig `hand_solver:` 블록(`onnx_path`·`tables_path`·`threads`, ONNX 전용)과 `HandSolverCfg`. 있으면 `load_rig` 가 에러.
- `HandPose.joint_angles`·`joint_rot`·`to_sensor_array`·`from_sensor_array`, `HUMAN_HAND`·`SENSED_JOINTS`·`JointSpec`
  (옛 AGA 17-quat 형식). 손 입력은 `xr_joints`(OpenXR 26관절) 하나다.
- `tools/` 의 관절각 데이터셋 파이프라인(BHaM·NinaPro → URDF q, URDF 피팅, IK 넷 학습·렌더 16개).
  `align_frames`·`export_*`·`bench_hand_retarget` 는 남는다.

### 변경 — 구조 정리

- **공유 상수를 `whatslab.core.constants` 한 곳으로 모았다**(`typing.Final`). `SIDES`, `FINGER_COUNT`,
  OpenXR 26관절(`XR_JOINT_NAMES`·`XR_JOINT_COUNT`·`XR_JOINT_FIELDS`·`XR_WRIST`), 사람 손 21점
  (`OPENXR26_TO_HAND21`·`HAND21_*`), OpenXR → 손 21점 축 변환(`OPENXR_TO_HAND21_AXES`). 같은 값이 dex
  (`xr_input`)·geort(`keypoints`·`solver`)·`airglove`·`teleop` 에 따로 정의돼 있었다. **옛 이름은 별칭으로 남기지
  않았다(호환 없는 변경)** — `OPENXR_TO_MANO`·`XR_TO_MANO`·`MANO_WRIST`·`MANO_TIPS`(xr_input),
  `OPENXR26_TO_MEDIAPIPE21`·`MEDIAPIPE21_COUNT`·`WRIST`·`INDEX_MCP`·`MIDDLE_MCP`·`RING_MCP`·`LITTLE_MCP`·`TIP_IDS`·
  `CANONICAL_TO_GEORT`(geort.keypoints), `XR_WRIST_INDEX`·`AGA_FINGER_COUNT`(airglove), `whatslab.receiver.glove`
  의 `XR_JOINT_NAMES`·`XR_JOINT_COUNT`·`XR_JOINT_FIELDS`·`XR_WRIST_INDEX` 재export, `TeleopModel.SIDES`,
  수신기의 `_CANONICAL_M`·`_M`, `vive.M` 은 `core.constants` / `receiver.base` 의 이름을 직접 쓴다.
  규칙: 한 모듈만 쓰는 상수는 그 모듈에, 두 곳 이상이 쓰면 `core.constants` 로. 튜닝 값은 인자·yaml.
  같은 값에 이름을 하나 더 붙이는 별칭(`X = Y`, 프로젝트 내부 `import … as …`)은 쓰지 않는다 —
  외부 라이브러리 관용 표기(`np`, `pin`, `pa`, `pq`, `ET`)만 예외.
- 기기 좌표계 → 정준 프레임 행렬을 `receiver/base.py` 로 모았다: `Y_UP_RH_TO_CANONICAL`(WebXR·OpenVR·OpenXR —
  webxr·vive·airglove 가 각자 정의하거나 webxr 의 private 이름을 import 하던 것).
- `DEX_PREFIX` 는 `solvers/hand/controller.py` 하나만 정의한다(dex_retargeter 는 거기서 가져온다).
- 모듈 이동·이름: `robot/arm_ik.py` → `robot/arm.py`(`solvers/arm/arm_ik.py` 와 이름이 같았다),
  `receiver/glove/base.py`(포트 상수뿐) → `airglove.py`.
- **사람 손 시각화 제거** — `human_fk`(`HumanHandFK`)·`human_upright_root`·`upright_root`. 관절각 입력이 사라져
  쓰는 곳이 없었고, 사람 손 화면은 Spine 이 맡는다. `whatslab.viz` 는 로봇(팔·손 URDF) 시각화만 남는다.
- `FINGER_NAMES`(URDF 센서 링크용 손가락 이름)를 `core.constants` 로 — tests·bench 도구에 같은 튜플이 있었다.

### 변경

- `whatslab.solvers` / `whatslab.solvers.hand` 는 지연 import 한다 — GeoRT 만 쓰면 pinocchio·dex-retargeting(torch)을
  불러오지 않는다.

## [Unreleased]

### 추가 — 글러브 프레임 메타데이터

- `GloveOpenXRReceiver.frame(side)` → `GloveFrame`(`xr_joints`(26, 8)·`valid`·`seq`·
  `sender_time_us`·`age_s`). 신선도 필터 없이 최신 프레임을 준다 — `seq` 로 새 프레임을 가리고
  여러 소비자가 같은 프레임을 받았는지 맞추는 용도(robotPlatform 입력 노드).
- `wrist_frame(side)` → `GloveWrist`(Spine 원본 `[y, x, z, -w]` 손등 쿼터니언 + 메타),
  `device_status()`·`alarms()`·`stats()` 는 airglove-client 값을 그대로 넘긴다.

### 제거 — 레거시 글러브 수신기 (호환 없는 변경)

- `GloveReceiverBase`·`GloveHumanHandReceiver`·`GloveHumanAnglesReceiver`·
  `GloveRobotHandReceiver` 와 `GloveModel(hand_source="angles"|"robot")`. 헤더·`/pair/N` 없는
  옛 주소와 Spine 이 더는 보내지 않는 #16/#17 에 의존해 현재 Spine(프로토콜 3)에서 아무것도 받지
  못했다. `osc_transport`·`python-osc` 는 Quest 수신기가 쓰므로 남긴다.

### 변경 — 의존성

- `numpy>=1.24`(상한 제거), `hand` extra 의 `dex-retargeting>=0.4.6`. numpy 1 소비자(Isaac Sim
  5.1)는 자기 쪽에서 `numpy<2`·`dex-retargeting<0.5` 를 고정한다. numpy 2.5 + dex-retargeting
  0.5.0 에서 테스트 통과(기존 실패 3건 제외: `test_arm` posture, robotis URDF 센서 링크).

### 추가 — 글러브 OpenXR 손 리타게팅 (DexPilot)

- **`XRHandRetargeter`**(`solvers/hand/dex_retargeter.py`) — `HandPose.xr_joints`(OpenXR 26관절)
  → 로봇 손 관절각. dex-retargeting 0.4.6 의 DexPilot 을 쓴다. 입력 처리(`solvers/hand/xr_input.py`):
  손목 상대 위치를 손목 회전으로 풀고(손목 자세 무관), OpenXR 26 → MANO 21
  (`[1,2,3,4,5,7,8,9,10,12,13,14,15,17,18,19,20,22,23,24,25]`), 축 `[[0,-1,0],[-1,0,0],[0,0,-1]]`
  (양손 같음), 그 뒤 **매 프레임** 사람 손바닥 프레임(손목 + 검지·중지·소지 MCP)을 로봇의 q=0
  손바닥 프레임(config `palm_links`)으로 돌린다. 고정 회전을 쓰려면 `mano_to_root.rpy`.
  config 의 반대쪽 손은 입력 거울상(`mirror_xr_joints`)으로 처리한다.
- **`configs/hands/`** — `orca_{left,right}`, `dg5f_{left,right}`, `robotis_{left,right}`. ORCA 는
  손목 관절을 빼고 0 고정(`fixed_joints`, 팔 IK 가 Carpals 를 쓴다), DG-5F 는 손가락 PIP·DIP 를
  −5..90° 로 자른다(`joint_limits_deg` — URDF ±90° 에서 역방향으로 꺾는 해가 나왔다). 전부
  `max_evals: 20`(틱당 SLSQP 평가 상한)·`low_pass_alpha: 0.5`.
- **`configs/hands/`** 추가 — `allegro`, `leap`, `ability`(PSYONIC), `schunk`(SVH) `{left,right}`.
  손목·손끝 링크·`target_joint_names` 는 dex-retargeting `configs/teleop/*_dexpilot.yml` 그대로.
  allegro·leap 는 손가락 4개(`palm_links.pinky` 에 약지), ability·schunk 는 URDF mimic 관절을
  target 에서 빼고 dex-retargeting mimic 어댑터가 채운다.
- `XRHandRetargeter.full_qpos` 가 URDF mimic 관절도 채운다. `examples/airglove_hand.py --viz` 는
  dex config 이면 `full_qpos` 로 그린다. `tools/bench_hand_retarget.py` 가 손가락 4개·mimic 손을 잰다.
- **`HandRetargetController`** — `config_name` 이 `dex:<이름>` 이면 `XRHandRetargeter` 를 쓰고
  `xr_joints` 를 읽는다(`xr_input` 속성). 그 밖은 지금까지처럼 ONNX + `joint_angles`.
  `robots/orca_right.yaml` 의 `retarget:` 가 `orca_hand` → `dex:orca`, `HandModel` 기본
  `hand_config` 도 `dex:orca`. rig `hand_solver:`(ONNX 전용)와 `dex:` 를 같이 쓰면 `ValueError`.
- `hand` extra 에 `dex-retargeting==0.4.6`(numpy<2 를 받는 마지막 버전; torch·nlopt<2.8·pin>=2.7,
  Python < 3.13). `import whatslab.teleop` 이 0.25 → 약 0.9초.
- **`tools/bench_hand_retarget.py`** 를 OpenXR 입력용으로 다시 넣었다 — 목표 구성과 무관한 물리
  지표(펴짐 접촉 오차, 핀치 시 로봇 손끝 거리, 뼈 방향각, |dq| p95, 한계 비율, 손당·2손 지연).
  입력은 npz(`xr` (N,26,8) + `side`, `*_left/_right.npz` 짝은 2손 지연용) 또는 Quest WebXR 녹화 json.
- `examples/airglove_hand.py` 기본 `--config dex:orca`(OpenXR 이면 사람 손 URDF viz 는 끈다).

측정 — agaSDK Python 손 모델이 실데이터에서 낸 OpenXR 26관절. `quest→aga` = Quest 핸드트래킹
녹화 11개 × 양손을 센서 회전으로 바꿔 모델에 넣은 것, `glove` = 실제 글러브 raw 녹화 16개
(캘리브 없음). `--mirror` 로 반대손도 거울상으로 넣었다. 핀치 = 사람 엄지-손가락 끝 < 20mm
(OpenXR 끝 관절이 손끝 표면이라 패드 접촉도 10~15mm 로 남는다).

| 입력 | config | 펴짐 접촉 오차 mm | 핀치 로봇 손끝 거리 mm (n) | 뼈 방향각 ° | \|dq\| p95 rad | 한계 2% 비율 | 손당 ms p50 / p99 | 2손 p99 ms |
|---|---|---|---|---|---|---|---|---|
| quest→aga (20718) | orca_right | 3.2 | 5.0 (562) | 11.6 | 0.106 | 0.087 | 1.15 / 3.79 | 7.10 |
| | orca_left | 3.2 | 5.0 (562) | 11.5 | 0.106 | 0.085 | 1.16 / 3.84 | 6.97 |
| | dg5f_right | 4.2 | 5.1 (562) | 12.1 | 0.089 | 0.133 | 1.05 / 4.21 | 7.65 |
| | dg5f_left | 4.2 | 5.1 (562) | 12.1 | 0.089 | 0.133 | 1.06 / 4.20 | 7.70 |
| | robotis_right | 2.6 | 5.5 (562) | 11.1 | 0.103 | 0.204 | 1.18 / 4.03 | 7.42 |
| | robotis_left | 2.5 | 7.2 (562) | 11.5 | 0.104 | 0.229 | 1.18 / 4.11 | 7.27 |
| glove (24528) | orca_right | 4.5 | 3.4 (812) | 13.3 | 0.176 | 0.168 | 1.28 / 3.78 | — |
| | orca_left | 4.4 | 3.5 (812) | 13.5 | 0.177 | 0.162 | 1.31 / 3.80 | — |
| | dg5f_right | 5.4 | 4.1 (812) | 15.9 | 0.158 | 0.267 | 1.44 / 4.22 | — |
| | dg5f_left | 5.4 | 4.1 (812) | 15.9 | 0.158 | 0.267 | 1.45 / 4.26 | — |
| | robotis_right | 4.9 | 4.4 (812) | 13.7 | 0.195 | 0.258 | 1.95 / 4.27 | — |
| | robotis_left | 4.7 | 4.9 (812) | 14.2 | 0.182 | 0.303 | 1.94 / 4.24 | — |

2손 p99 = 같은 틱의 왼손 + 오른손(양손 녹화가 있는 quest→aga 만). 판정 기준(펴짐·핀치 ≤ 10mm,
2손 p99 ≤ 8ms)은 전부 통과하지만 2손 p99 는 7.0~7.7ms 로 여유가 작다. `max_evals` 를 빼면(무제한)
프레임 무관 지표는 거의 같고(뼈 방향각 ±0.1°) 2손 p99 만 는다(같은 부하에서 20 → 6.5~7.7ms, 무제한 → 7.7~8.4ms). 핀치 "오차"(로봇 거리 −
스케일된 사람 거리)는 10.5~15mm 인데, DexPilot 투영이 사람 3cm 안쪽을 접촉으로 당기는 설계라서다.

예전 dex 백엔드 기각(아래 0.3.0, 펴짐 접촉 33.5mm)과 다른 점: 손가락별 `_SCALE_FACTOR`
없이 균일 스케일(로봇/사람 손 길이) 하나, 매 프레임 손바닥 정렬, 입력이 OpenXR 손목 공간이다.

### 알려진 제한 — OpenXR 리타게팅

- **펴진 손에서 ORCA 엄지가 한계에 붙는다.** Quest `finger_spread_flat` 에서 엄지 CMC·외전이
  각각 하한 −45°·상한 55° 에 붙고 IP 가 46° 굽어 엄지 원위 뼈 방향 오차가 37° 다(다른 손가락
  4~6°, 약지·소지 근위 약 20°). 로봇 엄지 가동범위 밖의 목표라 리타게터 설정으로는 안 풀린다.
- 핀치 투영(DexPilot `eta1=1e-4`)은 손끝 **프레임**을 맞붙인다. ORCA 손끝은
  `{side}_sensor_*_distal` 프레임이라 실물 패드는 그보다 일찍 닿는다 — 충돌 검사는 없다.
- 글러브 녹화는 캘리브 없이 모델에 넣었다(옛 캘리브 덤프는 현재 형식과 다르다). 실기 캘리브
  후 데이터로 다시 재야 한다.
- 실기(로봇 손) 검증 없음.

### 추가 — 현재 Spine(OpenXR 손 출력) 글러브 수신

- **`GloveOpenXRReceiver`**(`receiver/glove/openxr.py`) — `airglove-client` 의
  `Client` 를 감싸 손마다 `HandPose(wrist=Pose, xr_joints=(26, 8))` 를 낸다.
  `xr_joints` 는 OpenXR `XrHandJointEXT` 26관절 `[qx qy qz qw px py pz radius]` 를 받은
  그대로(손목 공간, OpenXR 관절 축, 미터), `wrist` 는 글러브의 캘리브된 OpenXR 손목
  (+Y 위 world, heading = IMU)을 WebXR 경로와 같은 `M R Mᵀ` 로 정준 프레임에 옮긴 값.
  `tracked` = 프레임 있음 · `age_s < stale_timeout`(기본 0.2 s) · 26관절 valid.
  `send_haptic` 은 값을 `round` 후 0..3 으로 클램프하고 항상 5개(빠진 손가락 = 0)를 보낸다
  (`haptic_levels`). `connected` 는 2 s 보다 오래된 `device_status` 를 무시한다.
  계약 정본: AirGlove-Client-Core `docs/spine_protocol.md`, AirGlove-Client `python/README.md`.
- **`HandPose.xr_joints`** (`Optional[np.ndarray]`, 기본 `None`).
- `receiver` extra 에 `airglove-client @ git+…AirGlove-Client.git@v0.1.0#subdirectory=python`.
- `GloveModel(…, hand_kwargs=None)`, `HandModel(…, hand_kwargs=None)` — 리시버 생성자
  인자(`glove_port` 등)를 생성 시점에 넘긴다(만든 뒤 리시버를 갈아 끼울 필요 없음).

### 변경 — Spine OSC 프로토콜 3 (airglove_client 0.4.0)

- `GloveOpenXRReceiver` 가 Spine 프로토콜 3(번호 헤더 + `/pair/N`)을 기준으로 한다. `pair`
  인자(기본 1, `Client.set_pair`), `spine_protocol()`, 라이브러리 0.4.0 미만·프로토콜 3 아님 경고.
  `set_stale_threshold` 는 0.4.0 에서 무효라 부르지 않는다 — 신선도는 원래대로 클라이언트 수신
  나이(`age_s`)만 본다(Spine 은 시각·순번을 보내지 않는다).
- **`HandPose.wrist` 를 더는 채우지 않는다(항상 `None`).** 프로토콜 3 의 손목은 raw 손등 IMU
  쿼터니언(`[y, x, z, -w]`, 캘리브 없는 센서 프레임)이라 팔 목표로 쓰면 틀린 동작이 나온다.
  `GloveModel` 의 팔 회전은 컨트롤러 회전으로 대체된다. raw 값은 `raw_wrist(side)` 로만 낸다.
- 종단 테스트를 AirGlove-Client-Core `eed7eb5` 의 `tools/spine_replay.py`·재녹화 캡처로 돌린다
  (라이브러리 0.4.0 미만이면 건너뜀).

### 변경

- **`GloveModel` 의 기본 `hand_source` 가 `"angles"` → `"openxr"`**, **`HandModel` 의 기본
  리시버가 `GloveHumanAnglesReceiver` → `GloveOpenXRReceiver`.** 옛 Spine(≤ 2.3.1)을
  쓰면 `hand_source="angles"` 를 명시해야 한다.
- ~~팔 회전 목표의 글러브 손목이 raw 손등 IMU → 캘리브된 OpenXR 손목(정준 변환)으로 바뀌었다.~~
  프로토콜 3 에서 Spine 이 다시 raw 손등 IMU 만 보내므로 `"openxr"` 소스는 손목을 내지 않는다(위).
- `GloveHumanAnglesReceiver`·`GloveHumanHandReceiver`·`GloveRobotHandReceiver` 는
  **레거시**(Spine ≤ 2.3.1)로 남는다. 현재 Spine 에서는 아무것도 받지 못한다.

### 알려진 제한

- ~~`xr_joints` 를 쓰는 손 리타게터가 아직 없다~~ — 위 「글러브 OpenXR 손 리타게팅」으로 해소.
  rig `retarget:` 가 ONNX 이름이면 여전히 `joint_angles` 만 받아 손이 직전 명령을 유지한다.
- `receiver` extra 의 `airglove-client` 핀을 `@v0.4.0` 으로 올렸다. 지금 Spine 은 프로토콜 **3**
  (모든 메시지 첫 인자가 번호 문자열, 손·장치 주소 앞에 `/pair/N`, Spine 은 임시 송신 포트에서
  `127.0.0.1:4040` 으로 보내고 `127.0.0.1:4042` 에서 받는다, AirGlove-Client-Core `docs/spine_protocol.md`)이고
  이전 태그(v0.1.0)는 프로토콜 1 이라 받지 못한다.
- 글러브 손목 자세가 없어 `GloveModel` 의 팔 회전은 컨트롤러 회전이다. raw 손등 IMU 를 팔 목표로
  쓰려면 센서 장착·heading 정렬 캘리브가 따로 필요하다(없음).
- 레거시 리시버(`GloveHumanAnglesReceiver`·`GloveHumanHandReceiver`·`GloveRobotHandReceiver`)는
  `/pair/N` 주소를 모른다 — 프로토콜 3 Spine 에서도 받는 것이 없다.
- 종단 테스트는 `AIRGLOVE_CORE_DIR`(AirGlove-Client-Core 체크아웃)의 `tools/spine_replay.py` 로
  프로토콜 3 Spine 을 흉내 낸다. 환경변수가 없거나 라이브러리가 0.4.0 미만이면 건너뛴다.

## [0.4.0] — 2026-08-21

### 추가 — 손 리타게팅 데이터 파이프라인 (`tools/`)

사람 손 데이터셋에서 **사람 손 URDF 관절각**을 만들고, URDF 의 구조·관절범위를
데이터에 맞추는 도구 묶음. 리타게팅 엔진(`solvers/hand/uni_retargeter.py`) 의 입력을
데이터셋에서 대량으로 뽑기 위한 것이다.

- `retarget_hand_dataset.py` — 키포인트 데이터셋 → URDF q. 블록 병렬
  (`--workers`/`--block`), 본길이 가중(`--w-bone`), 스켈레톤 지터 자기검증
  (`--selftest`/`--skeleton-jitter`).
- `nina_to_urdf_q.py` — 글러브 trial + 기준자세 → URDF q 시퀀스.
- `bham_clean.py` / `bham_joint_stats.py` / `bham_urdf_plan.py` — BHaM 시퀀스 정리
  (프레임 유효성 플래그), 관절 통계, 관절범위·축 제안(`--sd` 로 표준편차 배수).
- `urdf_fit_bham.py` / `fit_urdf_structure.py` — 키포인트에 맞춰 URDF 의 origin rpy 와
  axis 를 피팅한다.
- `urdf_apply_limits.py` — 제안 npz 를 URDF 에 적용. **기본 dry-run**, `--apply` 로 기록.
- `render_hand_q.py` / `render_bham_osim.py` / `render_osim_seq.py` /
  `render_osim_decor.py` — q 시퀀스·OpenSim 모델·decor 확인용 렌더.

### 추가

- **`LeRobotRecorder.discard_episode()`** — 진행 중이던 에피소드를 버린다(mp4 를 닫고
  그 에피소드의 mp4·parquet 을 지우고 인덱스를 재사용). 재촬영·중단에서 반쪽
  에피소드가 데이터셋에 조용히 섞이는 경로를 막는다.

### 변경

- **`LeRobotRecorder` 가 이미지를 버퍼링하지 않고 에피소드 mp4 로 즉시 스트리밍한다.**
  전에는 `save_episode()` 까지 원시 프레임을 RAM 에 들고 있어 640x480 3대 30Hz 에서
  초당 26MB 가 쌓였고(1분 에피소드 1.6GB), 종료 시 `mimwrite` 3개가 동기로 돌아 수 초
  멈췄다. 이미지 통계는 누적 합/제곱합으로 온라인 계산한다(`lerobot_schema.ImageStats`,
  배치 계산과 동일한 값). 단일 chunk 초과 검사는 `save_episode` 에서
  **`add_frame`** 으로 옮겨졌다 — 스트리밍이라 mp4 를 쓰기 전에 걸러야 한다.
- **viz 가 URDF 의 `package://` 경로를 models root 로 재기준한다.** 해당 이름의
  디렉토리가 root 에 없을 때만 치환하므로, 패키지를 설치하지 않은 URDF 도 메쉬를
  찾는다. 지오메트리·메쉬 하나가 실패해도 그것만 건너뛰고(로그 level info) 나머지를
  렌더하며, 전부 실패했을 때만 스켈레톤 모드로 강등한다.
- **실물 orca 연결을 `OrcaHandTouch` 로 바꿨다**(`examples/robot_io.py`) — 촉각형
  하드웨어(`orcahand_touch_*`)를 쓴다.

### 문서

- **지원하는 손 표를 추가했다**(`README.md`·`README.ko.md`·`docs/API.md`).
  통합 ONNX 표에 실제로 들어 있는 5종(human/orca/allegro/tesollo/robotis)과 그
  `retarget:` 값·별칭·`dexhand-description` 안의 URDF 경로를 명시한다. 값의 정본은
  `uni_retargeter._ALIAS`·`_URDF_STEM` 이다.

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
