# whatslab 사용 가이드

새 로봇을 올리고, 캘리브레이션하고, 팔 IK 를 튜닝하고, 문제를 진단하는 절차.
공개 심볼 목록은 [API.md](API.md), 개념 요약은 [README](../README.ko.md).

예제는 `$PY` 를 whatslab 이 설치된 파이썬으로 읽는다.

---

## 1. 새 로봇 올리기

config 는 2계층이다. **무엇인가**(설치 무관 스펙)와 **어떻게 조립했나**(설치)를
분리한다.

```
configs/robots/nero.yaml        # 이 팔이 무엇인가 — urdf, axis_align, ee
configs/robots/orca_right.yaml  # 이 손이 무엇인가 — urdf, base_frame, ee_align
configs/rigs/nero_orca_right.yaml   # 조립 — robots 참조 + mount/attach/lock_joints/solver
```

`arm`·`hand` 는 둘 다 optional이다. 팔만, 손만도 유효한 rig 다.

### 회전값은 추측하지 말고 튜너로 확정한다

`axis_align`·`attach`·`ee_align` 은 전부 URDF origin 관례(xyz + rpy, rpy 는
**고정축 XYZ**)다. 손으로 90° 씩 넣으면 축이 순환해서 작업공간이 눕는다.
`tools/align_frames.py` 를 3단계로 쓴다:

```bash
$PY tools/align_frames.py robot  --robot robots/nero.yaml        # 1) 로봇 → 정준축
$PY tools/align_frames.py attach --rig rigs/nero_orca_right.yaml # 2) 팔 ↔ 손 부착
$PY tools/align_frames.py ee     --rig rigs/nero_orca_right.yaml # 3) EE 프레임
```

viser 에서 결과를 보며 슬라이더로 맞추고, 확정값을 yaml 에 적는다.
**코드에 하드코딩 회전을 넣지 않는다.**

### `mount` 는 절대 방향이 아니다

`mount` 는 `robots/*.yaml` 의 `axis_align` **위에** 곱해진다:

```
M = mount.T @ axis_align.T          # 베이스 → 정준
```

`nero.yaml` 의 `axis_align` 이 이미 `rpy: [0, 0, π]`(URDF 가 −x 를 봄)이므로
`mount rpy: [0,0,0]` 도 항등이 아니다. `mount` 에는 **설치 보정분만** 넣는다 —
팔을 뒤로 20° 기울였다면 `rpy: [0, -0.35, 0]` 처럼 작은 각도 하나다.

### 도달 범위 기록

```bash
$PY examples/verify_rig.py --rig rigs/nero_orca_right.yaml --write
```

관절공간을 샘플링해 `solver.reach_max` 를 rig 에 기록한다. 이 값은 **안전망**
(먼 목표를 구로 클램프)이지, 작업공간의 정확한 경계가 아니다 — 실제 작업공간은
구가 아니므로 `reach_max` 안에도 도달 불가 자세가 많다.

---

## 2. 캘리브레이션

두 가지가 있고 서로 독립이다.

| | 무엇 | 언제 | 저장 |
|---|---|---|---|
| **yaw 정렬** | 사람의 정면을 로봇 정면에 맞추는 회전 스냅샷(`W`) | 매 세션, 기준 자세에서 | 안 됨(런타임) |
| **reach 스케일** | 사람 팔 길이 → 로봇 팔 길이 비율 | 사용자가 바뀔 때 | rig yaml `calibration.input_reach` |

`examples/quest_arm.py` 실행 중:

```
Enter          → yaw 캘리브 (기준 자세로 서서 누른다)
r + Enter      → reach 캘리브 (8초간 팔을 최대 범위로 뻗는다) → rig 에 기록
```

코드에서는 `model.calibrate_yaw()` / `model.calibrate_reach(persist=True)`.

**`rig calibration.enabled` 는 reach 스케일만 게이트한다.** `false` 로 둬도 yaw
캘리브는 그대로 동작한다. 스케일링은 `teleop/calibration.py` 한 곳에서만 한다.

reach 캘리브는 rig yaml 을 다시 써서 `input_reach` 를 남긴다. 쓰기는 원자적이지만
**yaml 을 재직렬화하므로 rig yaml 의 주석은 사라진다** — rig 값의 근거는 주석이
아니라 이 문서나 커밋 메시지에 쓴다.

---

## 3. 팔 IK 튜닝

기본값으로 대부분 동작한다. 손댈 곳은 rig `solver:` 뿐이다 — 코드 기본값을 고치면
diff 에 남지 않는다.

```yaml
solver:
  backend: diff          # diff = 틱당 소수 스텝(텔레옵 권장) / dls = 매 프레임 수렴(정밀)
  w_pos: 20.0            # 태스크 가중: 위치 vs 방위
  w_ori: 10.0
  iters_per_call: 20     # diff 백엔드의 틱당 반복
  reach_max: 0.9204      # verify_rig.py 가 기록
  max_joint_velocity: 5.0    # SafetyFilter rate-limit (rad/s)
  joint_weights:         # 관절 비용 — 싼 관절이 먼저 쓰인다
    joint1: 2.5
    joint2: 2.5
    joint3: 2.5
    joint4: 2.5
  k_posture: 0.05        # 자세를 관절범위 중앙으로 되돌리는 힘 (0 이면 코너에 박힌다)
  k_limit: 1.0           # 관절 한계 근처에서 밀어내는 힘
calibration:
  enabled: true
  input_reach: 0.8314
```

### 여분 자유도를 어디에 쓸지

7축 + 손목이면 6D 태스크에 자유도가 남는다. 남는 자유도는 **가만히 있지 않고
널스페이스에서 표류한다** — 방치하면 관절 한계에 박혀 못 나온다.

- `k_posture > 0` **필수**. 이게 0 이면 추종은 되는데 관절이 서서히 코너로 밀려가
  결국 방위를 못 맞춘다. `k_limit`(한계 0.10rad 안에서만 작동)은 이미 늦다.
- `q_neutral` 은 **관절범위 중앙**이다(0 이 아니다). 비대칭 범위 관절
  (예: `[-57.9°, 122.6°]`)에서 0 은 중앙이 아니다.
- `joint_weights` 로 선호를 준다. 팔을 무겁게 하면 손목이 먼저 쓰인다. 커플링을
  유지하므로 손목이 포화하면 팔이 이어받는다.

### 관절을 잠글 때

`lock_joints` 는 마지막 수단이다. **도달 가능한 목표로만 재면 잠금의 비용이 안
보인다** — FK 로 만든 궤적은 도달 가능이 보장돼서 여분 관절을 잠가도 오차가 같다.
잠금은 **실제 작업 위치 × 임의 방위의 도달률**로 판정한다.

### 변경을 판정하는 방법

세 가지를 지킨다. 어기면 잘못된 결론이 나온다(전부 실측으로 겪은 것):

1. **정확도는 FK 궤적으로만 잰다** — 유효 관절각을 FK 로 돌려 목표를 만든다.
   오차 하한이 0 이므로 남는 오차가 전부 솔버 탓이다. 좌표계로 합성한 궤적은
   도달 불가 구간을 지나 솔버 품질과 도달성을 섞는다. 고정 기준선 벤치는 내부
   도구이고 이 저장소에 포함하지 않는다.

2. **실기 데이터로 교차 검증한다.** 녹화를 떠서 오프라인 리플레이한다:
   ```bash
   $PY examples/quest_arm.py --dump-targets run.npz --diag
   ```
   `run.npz` 는 프레임별 원시입력·목표·해·오차를 담는다. 같은 목표를 다른 설정으로
   재생하면 공정 비교가 된다.

3. **데이터셋 2개 × 시작점 여러 개**로 잰다. 단일 시작점 리플레이는 초기 basin
   탈출 여부가 결과를 지배해서 후보 순위가 뒤집힌다(실측: 같은 설정이 시작점 0
   에서 89mm, 15% 지점에서 5mm). 합성 궤적에서는 시드를 여러 개(40개 수준) 잡아
   시드별 값을 받고 **대응차(paired)** 로 비교한다.

프레임별 **달성 가능 하한**(전역 탐색으로 그 목표에 최대한 붙인 값)을 함께 본다.
추종 오차가 하한에 붙어 있으면 솔버는 할 일을 다 한 것이고, 남은 오차는 도달성이나
rig 문제다.

---

## 4. 진단

증상별로 볼 곳이 정해져 있다.

### 목표는 오는데 팔이 이상한 곳으로 간다

```bash
$PY examples/quest_arm.py --diag
```

한 줄에 입력율 / `|p|`(정준) / `tgt|p|`(캘리브 후) / `base|p|`(베이스 좌표) /
clamp 비율 / IK 오차 / 틱당 `dq` / 캘리브 상태(`W`)가 나온다.

- `calib -` 이면 **yaw 캘리브를 안 한 것**이다. Enter 를 누른다.
- `base|p|` 가 `reach_max` 에 붙어 clamp 가 자주 뜨면 `mount` 나 reach 스케일이
  실제 설치와 안 맞는다.
- `cold start: 전역 탐색이 … 까지만 간다` 경고가 뜨면 그 목표가 **도달 불가**다.

### 라이브가 오프라인 리플레이보다 나쁘다

같은 목표를 리플레이했을 때 오차가 훨씬 작으면, 솔버가 아니라 **라이브 파이프라인
상태**가 문제다. 실측으로 세 번 나온 원인은 전부 side 간 상태 공유였다. 지금은
`SideModel` 이 그걸 구조적으로 막지만, 새 side별 상태를 추가할 때는
`SideModel` 필드로 넣는다 — side 를 도는 루프에 유상태 객체를 하나만 끼우면 버그다.

한 팔만 쓸 때는 반대쪽을 끈다. 반대쪽 컨트롤러가 이 팔 rig 에 도달 불가라
전역 탐색을 계속 태우고 프레임 예산을 넘긴다:

```bash
$PY examples/quest_arm.py --sides right     # 기본값이 --side 하나
```

### EE 가 순간이동한다

관절공간에서 멀리 떨어진 다른 IK 분기로 튄 것이다. 전역 탐색(`solve_robust`)은
**첫 타깃과 `reseed()`(캘리브 직후)에서만** 돌아야 한다. 매 프레임 부르면 정확도는
그대로인데 비용이 6배 들고 프레임마다 다른 분기로 튄다.

### 추종이 밀린다 / 60Hz 를 못 맞춘다

종료 시 `[rate] … 못 맞춘 프레임 N개` 가 나온다. 그리고 IK 가 로봇 속도 한계보다
빠른 명령을 내면 `SafetyFilter` 가 자르므로 그만큼 밀린다:

```
틱당 |dq| p99  vs  max_joint_velocity × dt
```

전자가 크면 `max_joint_velocity` 를 실제 하드웨어 한계까지 올리거나, 올릴 수
없으면 `dq_max_tick` 으로 IK 를 예산에 맞춘다.

---

## 5. 실물 로봇에 보내기

`examples/robot_io.py` 가 참조 구현이다. 두 가지가 중요하다:

- **전송은 별도 스레드**로 한다. CAN/dynamixel 쓰기가 수십 ms 블로킹하면 텔레옵
  루프가 그만큼 멈춘다.
- **관절 이름 매핑을 검증**한다. 하드웨어 SDK 가 모르는 관절명을 조용히 버리는
  경우가 있다(그래서 손가락 하나가 통째로 안 움직인다).

```bash
$PY examples/quest_arm.py --viz --robot     # viser 패널에서 연결 버튼
```

`--robot` 은 `--viz` 와 함께 쓴다. 연결은 기본 꺼져 있고 패널에서 켠다.
