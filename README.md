# WHATs LAB — 손 리타게팅 ONNX 추론 (데모/텔레옵 브랜치)

주식회사 왓츠랩(WHATs LAB Corp)의 손 리타게팅 배포 경로만 남긴 브랜치다.
사람 손 관절각이 들어가면 로봇 손 관절각이 나온다. 그게 전부다.

이 브랜치는 `feat/uni-onnx-retargeter` 에서 갈라져 나왔고 **되돌려 병합하지
않는다** — SDK 의 수신기·팔 솔버·로봇 IO·시각화·테스트를 전부 지웠기 때문이다.
그쪽 작업은 원 브랜치에서 계속한다.

## 왜 이렇게 작은가

리타게팅 경로 전체가 ONNX 그래프 하나에 들어 있다.

```
q_human (21) 라디안
  -> [오른손이면 부호벡터로 정준 좌측 자세 변환]
  -> 사람 FK + 센서 상태
  -> 통합 헤드 (MorphHead)
  -> 관절범위 역정규화
q_robot (n_joints) 라디안
```

사람 FK 까지 그래프 안이라 추론측에 torch·pinocchio·nlopt·dex-retargeting 이
필요 없다. `numpy` 와 `onnxruntime` 둘뿐이다.

로봇별 상수(관절 기술자, 관절 한계, 좌우 부호)는 `assets/uni_tables.npz` 에
표로 들어 있고 실행 시 그래프에 먹인다. 그래서 **모델 파일 하나가 손 5종 ×
좌우 2를 전부 담당한다** — 로봇마다 학습된 파라미터가 없다.

## 설치

```bash
pip install -e .
```

`urdf_path` 를 쓰려면 URDF 자산이 추가로 필요하다.

```bash
pip install -e '.[urdf]'
# 또는 이미 받아둔 경로를 가리킨다
export WHATSLAB_MODELS_ROOT=/path/to/dexhand_description
```

## 사용

```python
from whatslab.solvers import UniRetargeter

rt = UniRetargeter("left", "orca_hand")       # 손 방향, 로봇
q_robot = rt.compute({"thumb_mcp_flex": 0.4})  # 이름:라디안 사전
```

`compute()` 는 `rt.joint_names` 순서의 로봇 관절각(라디안) 배열을 돌려준다.
입력 사전에서 빠진 관절은 0 으로 둔다.

지원 로봇: `base_hand`, `orca_hand`, `allegro_hand`, `tesollo_dg5f`,
`robotis_hx5_d20`. 손 방향은 `left` / `right`.

## 다른 모델로 바꿔 끼우기

`onnx_path` 와 `tables_path` 를 주면 assets 기본값 대신 그걸 쓴다. 목적함수를
달리 학습한 모델을 나란히 놓고 비교할 때 쓴다.

```python
rt = UniRetargeter("left", "orca_hand",
                   onnx_path="assets/pinch_first.onnx",
                   tables_path="assets/pinch_first.npz")
```

## 지연 측정

```bash
python examples/run_retarget.py --hand left --robot orca_hand --threads 1
```

프레임당 밀리초와 환산 Hz 를 찍는다. 텔레옵 대역폭을 잡을 때 이 수를 쓴다.
