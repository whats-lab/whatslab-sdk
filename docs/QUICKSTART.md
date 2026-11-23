# 빠른 시작

새 PC 에서 클론부터 로봇 손이 글러브를 따라오기까지. 개념은 [README](../README.ko.md), 공개 심볼은
[API.md](API.md) 를 본다. 아래 `$PY` 는 이 저장소를 설치한 파이썬이다.

이 SDK 는 **손**만 다룬다. 팔 IK·손목 추적·팔 제어는 사용하는 쪽 몫이다.

---

## 0. 준비물

| | |
|---|---|
| 파이썬 | 3.9 이상 (DexPilot 엔진은 3.13 미만) |
| 글러브 입력 | AirGlove + **Spine 3.0.0** (같은 PC, UDP 4040) |
| 또는 맨손 입력 | Quest 헤드셋 브라우저(WebXR) — 앱 설치 없음 |
| 로봇 손 | 없어도 된다 — 시각화로 먼저 확인한다 |

---

## 1. 설치

```bash
git clone https://github.com/whats-lab/whatslab-sdk.git
cd whatslab-sdk
python3 -m venv .venv
PY=.venv/bin/python
$PY -m pip install -e '.[all,dev]'
$PY -m pytest -q
```

실패 없이 끝나면 정상이다. skip 은 선택 의존성(`lerobot`, 일부 URDF 등)이 없어서 나는 것이다.
GeoRT 엔진(`geort:`)만 쓸 거면 `.[receiver,viz]` 로 충분하다 — torch 가 딸려오지 않는다.

---

## 2. 글러브 → 로봇 손 (시각화)

Spine 3.0.0 을 켜고 글러브를 연결한 뒤:

```bash
$PY examples/airglove_hand.py --config dex:orca --viz
$PY examples/airglove_hand.py --config geort:orca_right --viz \
    --viz-urdf "$($PY -c 'from whatslab.paths import models_root; print(models_root())')/orca_hand/urdf/right.urdf"
```

PC 브라우저로 `http://localhost:8080` 을 열면 로봇 손이 보인다. 터미널 로그로 상태를 본다.

| 로그 | 뜻 |
|---|---|
| `[no-signal] 글러브 손 프레임 수신 없음` | Spine 에서 프레임이 안 온다 — 아래 6장 |
| `[정지] 글러브는 붙었지만 손이 안 움직인다` | 받고는 있는데 값이 그대로다 — Spine 화면에서 손이 움직이는지 본다 |
| `[추적] \|dq\|max …` | 정상 |

엔진 고르는 법: GeoRT(`geort:orca_right`, `geort:hx5_right`)는 numpy 만 쓰고 빠르다. 그 밖의 손은
DexPilot(`dex:<손>`, README 「지원하는 손」).

---

## 3. Quest 맨손으로

글러브 없이 Quest 핸드트래킹으로도 같은 리타게팅이 된다.

```python
from whatslab.receiver import WebXRHandReceiver
from whatslab.teleop import HandModel

m = HandModel("dex:orca", hand_source=WebXRHandReceiver())
m.start()
print(m.hand_source.url)      # https://192.168.0.x:8443/
```

1. 헤드셋 브라우저로 출력된 주소를 연다 — **자체 서명 인증서 경고를 1회 수락**한 뒤 「VR 시작」.
   페이지와 WebSocket 이 같은 오리진이라 한 번 수락하면 `wss://` 도 통과한다.
2. 컨트롤러를 내려놓으면 손 관절이 들어온다.

HTTPS 인증서는 수신기가 시작할 때 `~/.cache/whatslab/webxr/` 에 알아서 만든다(IP 가 바뀌면 재발급).
이 머신의 IPv4 는 모두 인증서에 들어가 있으므로, 안 붙으면 헤드셋이 같은 네트워크에 있는지부터 본다.
`scripts/make_webxr_cert.sh -i <IP>` 로 넣은 추가 IP 는 다음 실행 때 자동 재발급되면서 빠진다 — 직접 만든
인증서는 `WebXRHandReceiver(certfile=…, keyfile=…)` 로 넘긴다. 유선은 `tls=False` + `adb reverse tcp:8443 tcp:8443`.

---

## 4. 내 파이프라인에 붙이기

```python
from whatslab.safety import SafetyFilter, load_limits_from_urdf

m = HandModel("geort:orca_right")
limits = load_limits_from_urdf(open(urdf_path).read())
m.safety = SafetyFilter(limits, dt=1 / 60)   # 선택: 위치·속도 제한(dt 는 측정값이 없을 때 쓰는 기본 주기)
m.start()
while True:
    q = m.get_q()["right"]    # {관절: rad} — 손이 안 잡힌 틱에는 {}
    send_to_hand(q)
```

- `get_q()` 는 부를 때마다 수신기를 한 번 읽는다. 주기는 사용하는 쪽 루프가 정한다(30~100Hz).
- 손목 자세는 주지 않는다. 손을 팔에 달았다면 팔 쪽에서 손목을 따로 추적·제어한다.
- 햅틱: `m.hand_source.send_haptic("right", [0, 1, 2, 3, 0])` (손가락별 0 끔 ~ 3 강).

---

## 5. 실물 손에 보내기

하드웨어 드라이버는 SDK 에 없다(손마다 다르다). 보낼 때 두 가지를 지킨다.

- **전송은 별도 스레드로 한다.** 시리얼·CAN 쓰기가 수십 ms 블로킹하면 그만큼 루프가 멈춘다.
- **관절 이름 매핑을 확인한다.** 하드웨어 SDK 가 모르는 관절명을 조용히 버리는 경우가 있다 — 그러면
  손가락 하나가 통째로 안 움직인다. 시작할 때 `m.retarget.joint_names` 와 하드웨어 관절 목록을 대조한다.

---

## 6. 자주 걸리는 것

### 글러브 프레임이 안 온다

- Spine 이 **3.0.0** 인지 본다. 2.x 는 지원하지 않는다(whatslab 0.4.x 를 쓴다).
- Spine 은 PC 당 클라이언트 하나에만 보낸다(UDP 4040). 다른 프로세스(Spine 설정창의 다른 앱,
  whatslab-ros2 드라이버 등)가 이미 받고 있으면 두 번째 수신기는 포트를 못 잡고 `AirGloveError` 가 난다.

### WebXR 페이지는 열리는데 데이터가 안 온다

「VR 시작」을 눌렀는지, 컨트롤러를 내려놓았는지 확인한다. 헤드셋을 벗었다 쓰면 세션이 끊기지만 재연결은
자동이다 — 페이지를 새로고침할 필요는 없다.

### 왼손 글러브인데 오른손 관절이 나온다

GeoRT 모델은 오른손용이라 왼손 입력을 거울상으로 풀어 **오른손 로봇 관절**을 낸다(정상). 왼손 로봇 손은
`dex:<손>` 엔진을 쓴다.
