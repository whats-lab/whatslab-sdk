#!/usr/bin/env bash
# 실물 텔레옵(examples/quest_arm.py --robot) 에 필요한 드라이버를 설치한다.
#
#   pyAgxArm  : examples/requirements-robot.txt (python-can 만 의존). 패키지 extra 가 아니다.
#   orca_core : extra 에 못 넣는다 — 전 태그가 numpy>=2.2.6 을 선언해 numpy 1 환경(Isaac Sim 5.1 등)에서
#               pip 해석이 실패한다(ResolutionImpossible). 실제 코드는 numpy 1.26 에서도 동작하므로
#               --no-deps 로 넣고 진짜 런타임 deps 를 여기서 직접 준다(numpy 는 환경의 것을 쓴다).
#
#   uv 가 아니라 "$PY" -m pip 를 쓴다 — PY 로 고른 기존 인터프리터(conda·시스템·venv)에 넣는 스크립트이고,
#   orca_core 의 --no-deps 우회는 프로젝트 잠금(uv sync)으로 표현할 수 없다.
set -euo pipefail

# 호출한 디렉토리 기준 상대경로(PY·ORCA_SRC)는 저장소 루트로 옮기기 전에 절대경로로 바꾼다.
PY="${PY:-python}"
case "$PY" in */*) PY="$(cd "$(dirname "$PY")" && pwd)/$(basename "$PY")" ;; esac
ORCA_REF="${ORCA_REF:-v0.3.0}"
if [ -n "${ORCA_SRC:-}" ]; then
    mkdir -p "$ORCA_SRC"
    ORCA_SRC="$(cd "$ORCA_SRC" && pwd)"
fi
cd "$(dirname "$0")/.."  # -e "." 와 examples/ 경로가 어디서 실행해도 저장소 루트 기준이 되게
ORCA_SRC="${ORCA_SRC:-$(pwd)/../orca_core}"

echo "[1/3] whatslab-sdk[all,dev] + examples/requirements-robot.txt"
"$PY" -m pip install -e ".[all,dev]"
"$PY" -m pip install -r examples/requirements-robot.txt

# wheel 에는 models/ 가 안 들어간다(config.yaml·calibration.yaml 이 거기 있다).
# 그래서 git 설치가 아니라 소스 클론 + editable 로 넣는다.
echo "[2/3] orca_core ${ORCA_REF} → ${ORCA_SRC} (editable, --no-deps)"
if [ ! -d "$ORCA_SRC/.git" ]; then
    git clone https://github.com/orcahand/orca_core "$ORCA_SRC"
fi
git -C "$ORCA_SRC" fetch --tags --quiet
git -C "$ORCA_SRC" checkout --quiet "$ORCA_REF"
"$PY" -m pip install --no-deps -e "$ORCA_SRC"
# orca_core 가 선언한 범위 그대로 — numpy 만 빼고 환경의 것을 둔다.
"$PY" -m pip install \
    "dynamixel-sdk>=3.7.31,<4.0.0" \
    "pyserial>=3.5,<4.0" \
    "pyyaml>=6.0.2,<7.0.0" \
    "fastapi>=0.115.12,<0.116.0" \
    "uvicorn>=0.34.2,<0.35.0"

echo "[3/3] 확인"
"$PY" - <<'EOF'
import numpy, pyAgxArm, orca_core
from orca_core import OrcaHand, OrcaHandTouch
from orca_core.utils.utils import get_model_path
print(f"numpy      {numpy.__version__}")
print(f"pyAgxArm   {getattr(pyAgxArm, '__version__', 'ok')}")
print(f"orca_core  {getattr(orca_core, '__version__', 'ok')}")
for name in ("orcahand_touch_right", "orcahand_right"):
    print(f"model      {name} -> {get_model_path(model_name=name)}")
EOF
echo
echo "완료. 실물 텔레옵:"
echo "  $PY examples/quest_arm.py --rig rigs/nero_orca_right.yaml --viz --robot [--no-tactile]"
