#!/usr/bin/env bash
# 실물 텔레옵(examples/quest_arm.py --robot) 에 필요한 드라이버를 설치한다.
#
#   pyAgxArm  : pyproject 의 [robot] extra 로 들어간다 (python-can 만 의존).
#   orca_core : extra 에 못 넣는다 — 전 태그가 numpy>=2.2.6 을 선언하는데
#               이 저장소는 numpy<2 핀이라 pip 해석이 실패한다(ResolutionImpossible).
#               실제 코드는 numpy 1.26 에서 동작하므로 --no-deps 로 넣고
#               진짜 런타임 deps 를 여기서 직접 준다.
set -euo pipefail

PY="${PY:-python}"
ORCA_REF="${ORCA_REF:-v0.3.0}"
ORCA_SRC="${ORCA_SRC:-$(cd "$(dirname "$0")/.." && pwd)/../orca_core}"

echo "[1/3] whatslab-sdk[all,robot]"
"$PY" -m pip install -e ".[all,robot]"

# wheel 에는 models/ 가 안 들어간다(config.yaml·calibration.yaml 이 거기 있다).
# 그래서 git 설치가 아니라 소스 클론 + editable 로 넣는다.
echo "[2/3] orca_core ${ORCA_REF} → ${ORCA_SRC} (editable, --no-deps)"
if [ ! -d "$ORCA_SRC/.git" ]; then
    git clone https://github.com/orcahand/orca_core "$ORCA_SRC"
fi
git -C "$ORCA_SRC" fetch --tags --quiet
git -C "$ORCA_SRC" checkout --quiet "$ORCA_REF"
"$PY" -m pip install --no-deps -e "$ORCA_SRC"
# orca_core 가 선언한 범위 그대로 — numpy 만 이 저장소 핀(<2)을 따른다.
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
assert numpy.__version__.startswith("1."), "numpy<2 핀이 깨졌다"
for name in ("orcahand_touch_right", "orcahand_right"):
    print(f"model      {name} -> {get_model_path(model_name=name)}")
EOF
echo
echo "완료. 실물 텔레옵:"
echo "  $PY examples/quest_arm.py --rig rigs/nero_orca_right.yaml --viz --robot [--no-tactile]"
