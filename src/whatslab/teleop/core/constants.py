"""dex_retargeting 2단계 IK 파라미터 (atlas_hand_core/config.py 계승)."""

HUBER_DELTA     = 0.025
NORM_DELTA      = 0.01
IK_MAX_EVAL     = 100       # 반복 종료 (결정적 + 수렴). 단계별 nlopt maxeval
# VECTOR/POSITION_WEIGHT: 과거 시간예산(maxtime)을 두 스테이지에 배분하던 가중치.
# 현재는 maxeval(반복) 종료라 리타게팅에 영향 없음 — HandRetargeter 시그니처
# 호환(레거시 위치인자)용 기본값으로만 유지한다.
VECTOR_WEIGHT   = 1.0
POSITION_WEIGHT = 4.0
