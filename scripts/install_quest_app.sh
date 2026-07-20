#!/usr/bin/env bash
#
# PoseDataTracker (Quest 포즈 송신 앱)을 Meta Quest 에 adb 로 설치한다.
# whatslab.receiver.meta.QuestReceiver 가 이 앱이 보내는 OSC 를 수신한다.
#
# 사용:
#   scripts/install_quest_app.sh [경로.zip|경로.apk]
#     인자 생략 시: 스크립트 상위폴더 → ~/Downloads 순으로 PoseDataTracker*.{zip,apk} 탐색
#   -y : 확인 프롬프트 없이 설치
#
set -euo pipefail

ASSUME_YES=0
SRC=""
for a in "$@"; do
  case "$a" in
    -y|--yes) ASSUME_YES=1 ;;
    *) SRC="$a" ;;
  esac
done

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ── 1. 소스(zip/apk) 결정 ────────────────────────────────────────────────
if [[ -z "$SRC" ]]; then
  # 우선순위: 레포 내장(assets/quest) → 상위폴더 → ~/Downloads
  for d in "$SCRIPT_DIR/../assets/quest" "$SCRIPT_DIR/.." "$SCRIPT_DIR/../.." "$HOME/Downloads"; do
    f=$(ls "$d"/PoseDataTracker*.apk "$d"/PoseDataTracker*.zip 2>/dev/null | head -n1 || true)
    if [[ -n "$f" ]]; then SRC="$f"; break; fi
  done
fi
if [[ -z "$SRC" || ! -f "$SRC" ]]; then
  echo "[오류] 설치할 zip/apk 를 찾지 못했습니다. 경로를 인자로 주세요." >&2
  echo "  예: $0 ~/Downloads/'PoseDataTracker 1.0.3.zip'" >&2
  exit 1
fi

# ── 2. APK 추출 (zip 이면 임시 폴더에 풀기) ──────────────────────────────
TMP=""
cleanup() { [[ -n "$TMP" ]] && rm -rf "$TMP"; }
trap cleanup EXIT

case "$SRC" in
  *.apk) APK="$SRC" ;;
  *.zip)
    command -v unzip >/dev/null || { echo "[오류] unzip 필요: sudo apt install unzip" >&2; exit 1; }
    TMP="$(mktemp -d)"
    unzip -q "$SRC" -d "$TMP"
    APK="$(find "$TMP" -iname '*.apk' | head -n1)"
    [[ -n "$APK" ]] || { echo "[오류] zip 안에 .apk 가 없습니다: $SRC" >&2; exit 1; }
    ;;
  *) echo "[오류] .zip 또는 .apk 만 지원: $SRC" >&2; exit 1 ;;
esac
echo "[정보] APK: $APK ($(du -h "$APK" | cut -f1))"

# ── 3. adb / 기기 확인 ───────────────────────────────────────────────────
command -v adb >/dev/null || { echo "[오류] adb 없음: sudo apt install android-tools-adb" >&2; exit 1; }
adb start-server >/dev/null 2>&1 || true

mapfile -t DEVICES < <(adb devices | awk 'NR>1 && $2=="device"{print $1}')
UNAUTH=$(adb devices | awk 'NR>1 && $2=="unauthorized"{print $1}' | head -n1 || true)

if [[ -n "$UNAUTH" ]]; then
  echo "[오류] 기기가 unauthorized 입니다($UNAUTH)." >&2
  echo "  헤드셋을 착용하고 'USB 디버깅 허용' 팝업을 수락하세요." >&2
  exit 1
fi
if [[ ${#DEVICES[@]} -eq 0 ]]; then
  echo "[오류] 연결된 기기가 없습니다." >&2
  echo "  1) Quest 를 USB 로 연결  2) 개발자 모드+USB 디버깅 활성화  3) 헤드셋에서 허용 수락" >&2
  exit 1
fi

TARGET="${DEVICES[0]}"
if [[ ${#DEVICES[@]} -gt 1 ]]; then
  echo "[경고] 기기가 여러 대(${DEVICES[*]}). 첫 번째($TARGET) 에 설치합니다." >&2
fi
MODEL="$(adb -s "$TARGET" shell getprop ro.product.model 2>/dev/null | tr -d '\r' || true)"
echo "[정보] 대상 기기: $TARGET ${MODEL:+($MODEL)}"

# ── 4. 확인 후 설치 ──────────────────────────────────────────────────────
if [[ "$ASSUME_YES" -ne 1 ]]; then
  read -r -p "이 기기에 설치할까요? [y/N] " ans
  [[ "$ans" =~ ^[Yy]$ ]] || { echo "취소됨."; exit 0; }
fi

echo "[설치] adb install -r -g ..."
adb -s "$TARGET" install -r -g "$APK"

echo "[완료] 설치됨. Quest '앱 → 알 수 없는 소스' 에서 PoseDataTracker 실행 후,"
echo "       송신 대상 IP 를 이 PC 로 설정하면 QuestReceiver(OSC 9000)가 수신합니다."
