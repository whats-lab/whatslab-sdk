# 실물 텔레옵 빠른 시작

새 PC 에서 클론부터 nero 팔 + ORCA 손 텔레옵까지. 개념은 [README](../README.ko.md),
튜닝·진단은 [GUIDE.md](GUIDE.md), 공개 심볼은 [API.md](API.md) 를 본다.

아래 `$PY` 는 이 저장소를 설치한 파이썬이다.

---

## 0. 준비물

| | |
|---|---|
| OS | Linux (CAN·시리얼 경로가 리눅스 전제다) |
| 파이썬 | 3.9 이상 |
| 팔 | AgileX nero, CAN 으로 연결 (`can0`) |
| 손 | ORCA hand, USB 시리얼 |
| 입력 | Quest 컨트롤러·맨손 — 브라우저 WebXR |
| 손 입력(`--arm controller`, 기본) | AirGlove + Spine 3.0.0 (같은 PC, UDP 4040) |

손·팔이 없어도 **시각화까지는 돈다**(3단계). 하드웨어는 5단계부터 필요하다.

---

## 1. 설치

```bash
git clone https://github.com/whats-lab/whatslab-sdk.git
cd whatslab-sdk
python3 -m venv .venv

PY=.venv/bin/python ./scripts/install_robot_deps.sh
```

스크립트가 하는 일은 세 가지다.

1. `whatslab-sdk[all,dev]` + `examples/requirements-robot.txt` — SDK + 리타게팅/IK/viz + pytest, 그리고
   예제 전용 하드웨어 드라이버 `pyAgxArm`(CAN).
   URDF·메쉬는 [`dexhand-description`](https://github.com/whats-lab/dexterous-hand-urdf)
   이 git 에서 자동으로 딸려온다.
2. `orca_core` 를 소스 클론해 editable 로 설치. **extra 에 못 넣는 이유가 두 가지**다 —
   전 태그가 `numpy>=2.2.6` 을 선언해 numpy 1 환경(Isaac Sim 5.1 등)에서 pip 해석이
   실패하고(코드 자체는 numpy 1.26 에서 동작한다), wheel 에 손 설정
   `models/<버전>/<모델>/config.yaml` 이 들어가지 않는다.
3. 설치 확인 — numpy 버전, 드라이버 import, 손 모델 경로 해석.

설치 위치를 바꾸려면 `ORCA_SRC`, orca_core 버전을 바꾸려면 `ORCA_REF` 를 준다.

```bash
ORCA_SRC=~/src/orca_core ORCA_REF=v0.3.0 PY=.venv/bin/python ./scripts/install_robot_deps.sh
```

### 확인

```bash
$PY -m pytest -q -rs
```

실패 없이 끝나면 정상이다. skip 은 선택 의존성(`lerobot`, 일부 URDF 등)이 없어서 나는 것이다 —
`lerobot` 은 런타임 의존이 아니다(`whatslab.data` 는 lerobot 없이 v2.1 을 쓴다).

> 설치 중 뜨는 `orca-core requires numpy>=2.2.6, but you have 1.26.4` 경고는
> **의도된 것**이다. 위 2번 이유 그대로다.

---

## 2. 입력 경로

Quest 입력은 **헤드셋 브라우저 WebXR** 하나다(전용 앱·APK 없음). HTTPS 인증서는 수신기가 시작할 때
`~/.cache/whatslab/webxr/` 에 알아서 만든다(IP 가 바뀌면 재발급). `--arm controller`(기본) 는 컨트롤러로 팔, AirGlove 로 손,
`--arm wrist` 는 맨손 핸드트래킹으로 팔(손목)과 손(관절 26개)을 함께 움직인다.

---

## 3. 하드웨어 없이 먼저 띄운다

로봇을 붙이기 전에 입력과 IK 가 도는지부터 본다. `--robot` 을 빼면 아무것도
움직이지 않는다.

```bash
$PY examples/quest_arm.py --rig rigs/nero_orca_right.yaml --viz
```

시작하면 이렇게 뜬다.

```
[webxr] 헤드셋 브라우저에서 열어라: https://192.168.0.x:8443/
[calib] 기준 자세로 Enter → yaw 캘리브 | 'r'+Enter → reach 캘리브. Ctrl-C 종료.
```

1. PC 브라우저로 `http://localhost:8080` — viser 3D 뷰.
2. 헤드셋 브라우저로 위 `https://...:8443/` — **자체 서명 인증서 경고를 1회 수락**한 뒤
   「VR 시작」. 페이지와 WebSocket 이 같은 오리진이라 한 번 수락하면 `wss://` 도 통과한다.
3. 패스스루로 주변이 보이고 컨트롤러 위치가 그려지면 연결된 것이다.
4. 터미널의 `[q] arm=[...] target=on` 에서 `target=on` 이면 목표가 들어오고 있다.

IP 가 여러 개면 로그가 후보를 같이 찍는다. 이 머신의 IPv4 는 모두 인증서에 들어가 있으므로, 안 붙으면
헤드셋이 그 IP 와 같은 네트워크에 있는지부터 본다. `scripts/make_webxr_cert.sh -i <IP>` 로 넣은 추가 IP 는
다음 실행 때 수신기가 인증서를 자동으로 다시 만들면서 빠진다 — 직접 만든 인증서를 쓰려면 코드에서
`certfile=`/`keyfile=` 을 넘긴다(`quest_arm.py` 에는 이 옵션이 없다).

유선(`adb reverse`)으로 쓸 거면 `--webxr-no-tls` 를 주고 `localhost` 로 붙는다.

---

## 4. 캘리브레이션

`target=on` 이 뜬 뒤에 한다. 순서가 중요하다.

| 입력 | 하는 일 | 언제 |
|---|---|---|
| `Enter` | yaw 정렬 — 지금 향한 방향을 로봇의 앞으로 잡는다 | 매 세션, 기준 자세에서 |
| `r` + `Enter` | reach 측정 — 8초간 팔을 최대로 뻗는다 | 사람이 바뀔 때 |

reach 결과는 rig yaml 의 `calibration.input_reach` 에 자동 기록된다(커밋 diff 에
그 줄만 바뀌는 게 정상이다). yaw 는 저장되지 않으므로 **매번 해야 한다**.

캘리브가 이상하면 `--no-calib` 로 끄고 A/B 한다. 자세한 것은
[GUIDE.md §2](GUIDE.md#2-캘리브레이션).

---

## 5. 실물 연결

여기서부터 토크가 들어간다. **팔 주변을 비운다.**

### 먼저 CAN 부터 확인

```bash
ip -details -brief link show can0     # UP 이어야 한다
```

내려가 있으면 올린다. nero 는 1Mbps 다.

```bash
sudo ip link set can0 up type can bitrate 1000000
```

붙었는지는 토크를 넣지 않고 확인할 수 있다 — `connect()` 는 읽기만 한다.

```bash
$PY -c "
import time
from pyAgxArm import AgxArmFactory, ArmModel, NeroFW, create_agx_arm_config
r = AgxArmFactory.create_arm(create_agx_arm_config(
    robot=ArmModel.NERO, firmeware_version=NeroFW.DEFAULT,
    interface='socketcan', channel='can0'))
r.connect()
time.sleep(0.5)            # 피드백 스레드가 첫 프레임을 받을 때까지
print(r.joint_nums, '축', r.get_joint_angles().msg)
"
```

### 실행

```bash
$PY examples/quest_arm.py --rig rigs/nero_orca_right.yaml \
    --viz --robot --speed 20
```

`--robot` 은 `--viz` 와 **함께** 써야 한다. 연결 버튼이 viser 패널의 「실물 로봇」
폴더에 있고, **기본은 미연결**이다. 팔·손을 각각 켤 수 있다.

- 속도는 낮게 시작한다(`--speed 20`). 텔레옵은 높일 이유가 없다.
- 손 모델은 `orca_core` 기본값을 쓴다. 다른 손이면 `--hand-model orcahand_touch_right`.
- 촉각 센서를 안 쓰거나 못 붙으면 `--no-tactile` — `OrcaHand` 로 붙어 센서 포트를
  아예 열지 않는다.

---

## 6. 자주 걸리는 것

### 손 토크 인가가 17개 ID 전부 실패한다

```
Could not set torque enabled for IDs: [1, 2, ... 17]
```

거의 항상 **촉각 클라이언트가 모터 버스를 열어서** 생긴다. 손 config 의
`sensors.port` 가 모터 어댑터를 가리키고 있는지 본다 — 둘 다 `/dev/ttyACM*` 로
잡혀서 섞이기 쉽다.

```bash
ls -l /dev/serial/by-id/            # 어느 장치가 어느 ACM 인지
```

`sensors.port` 를 촉각 어댑터의 `by-id` 경로로 바꾸거나, 급하면 `--no-tactile` 로
돌린다(센서 포트를 열지 않으므로 이 문제가 사라진다). 모터 쪽 `port` 도
`/dev/ttyACMn` 이 아니라 `by-id` 경로로 적어야 재부팅 후에도 안 바뀐다.

### 손가락 하나가 통째로 안 움직인다

관절 이름 매핑이 어긋난 것이다. 하드웨어 SDK 가 모르는 관절명을 **조용히 버린다**.
연결 시 찍히는 경고를 본다.

```
[orca] WARN: 하드웨어에 없는 관절명 [...]
[orca] 명령하지 않는 하드웨어 관절: [...]
```

둘 다 없고 `관절 매핑 N/N 일치` 가 뜨면 매핑은 정상이다.

### WebXR 페이지는 열리는데 데이터가 안 온다

「VR 시작」을 눌렀는지 확인한다. WebXR 세션이 시작되기 전에는 컨트롤러 포즈를
읽을 수 없다. 헤드셋을 벗었다 쓰면 세션이 끊기지만 재연결은 자동이다 — 페이지를
새로고침할 필요는 없다.

### 60Hz 를 못 맞춘다

종료 시 `[rate] 60Hz 를 못 맞춘 프레임 N개` 가 찍힌다. 원인 분리는
[GUIDE.md §4](GUIDE.md#4-진단) 를 본다.

### 팔이 엉뚱한 곳으로 간다

대개 yaw 캘리브를 안 했거나(매 세션 필요) 한 팔 rig 에 반대쪽 컨트롤러가 물린 것이다(`--sides right`).
`--diag` 로 보는 법은 [GUIDE.md §4](GUIDE.md#4-진단).

---

## 다음

- 튜닝·진단 — [GUIDE.md](GUIDE.md)
- 공개 API — [API.md](API.md)
- 직접 만든 하드웨어 조합 — [README 빠른 시작](../README.ko.md#빠른-시작)(`TeleopModel` 상속)
