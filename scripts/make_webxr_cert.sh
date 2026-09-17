#!/usr/bin/env bash
#
# WebXR 무선 접속용 자체 서명 인증서를 만든다.
# whatslab.receiver.webxr.WebXRControllerReceiver(tls=True) 가 이 인증서를 읽는다.
#
# WebXR 은 보안 컨텍스트(HTTPS 또는 localhost)에서만 동작한다. 무선으로 쓰려면
# 헤드셋이 https://<이 PC 의 IP>:8443/ 으로 붙어야 하므로 인증서의 SAN 에
# 그 IP 가 들어 있어야 한다 — CN 만 있는 인증서는 요즘 브라우저가 거부한다.
#
# 사용:
#   scripts/make_webxr_cert.sh [-d 출력폴더] [-i 추가IP] [-n 일수] [-f]
#     -d : 기본 ${XDG_CACHE_HOME:-~/.cache}/whatslab/webxr
#     -i : SAN 에 IP 를 더 넣는다(여러 번 가능). 생략 시 이 머신의 전역 IPv4 자동 수집
#     -n : 유효기간(일), 기본 3650
#     -f : 기존 인증서가 있어도 다시 만든다
#
# 헤드셋에서는 최초 1회 인증서 경고를 수락해야 한다. 페이지와 WebSocket 이 같은
# 오리진(같은 포트)이라 한 번 수락하면 wss:// 도 같이 통과한다.
set -euo pipefail

OUT_DIR="${XDG_CACHE_HOME:-$HOME/.cache}/whatslab/webxr"
DAYS=3650
FORCE=0
EXTRA_IPS=()

while getopts ":d:i:n:fh" opt; do
  case "$opt" in
    d) OUT_DIR="$OPTARG" ;;
    i) EXTRA_IPS+=("$OPTARG") ;;
    n) DAYS="$OPTARG" ;;
    f) FORCE=1 ;;
    h) sed -n '2,24p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; exit 0 ;;
    \?) echo "알 수 없는 옵션: -$OPTARG" >&2; exit 2 ;;
    :) echo "-$OPTARG 는 값이 필요하다" >&2; exit 2 ;;
  esac
done

command -v openssl >/dev/null 2>&1 || {
  echo "openssl 이 없다 — 설치해야 한다 (apt install openssl)" >&2; exit 1; }

CERT="$OUT_DIR/cert.pem"
KEY="$OUT_DIR/key.pem"
STAMP="$OUT_DIR/san.txt"

# ── 1. SAN 구성: localhost + 이 머신의 전역 IPv4 + -i 로 받은 것 ────────────
mapfile -t AUTO_IPS < <(ip -4 -o addr show scope global 2>/dev/null \
                          | awk '{print $4}' | cut -d/ -f1)
IPS=("${AUTO_IPS[@]}" ${EXTRA_IPS[@]+"${EXTRA_IPS[@]}"})
if [[ ${#IPS[@]} -eq 0 ]]; then
  echo "전역 IPv4 를 찾지 못했다 — 네트워크에 붙어 있는지 확인하거나 -i 로 직접 줘라" >&2
  exit 1
fi

SAN="DNS:localhost,IP:127.0.0.1,IP:::1"
for ip in "${IPS[@]}"; do SAN="$SAN,IP:$ip"; done

# ── 2. 이미 같은 SAN 으로 만들어 뒀으면 재사용 ──────────────────────────────
# (매번 새로 만들면 헤드셋에서 인증서 경고를 다시 수락해야 한다)
if [[ $FORCE -eq 0 && -f "$CERT" && -f "$KEY" && -f "$STAMP" ]] \
   && [[ "$(cat "$STAMP")" == "$SAN" ]]; then
  echo "[cert] 기존 인증서 재사용: $CERT"
  openssl x509 -in "$CERT" -noout -subject -enddate -ext subjectAltName
  exit 0
fi

# ── 3. 생성 ────────────────────────────────────────────────────────────────
mkdir -p "$OUT_DIR"
openssl req -x509 -newkey rsa:2048 -nodes \
  -keyout "$KEY" -out "$CERT" \
  -days "$DAYS" -subj "/CN=whatslab-webxr" \
  -addext "subjectAltName=$SAN" 2>/dev/null
chmod 600 "$KEY"
printf '%s' "$SAN" > "$STAMP"

echo "[cert] 생성 완료"
echo "  cert : $CERT"
echo "  key  : $KEY"
openssl x509 -in "$CERT" -noout -subject -enddate -ext subjectAltName

# 기본 경로가 실제로 쓰는 src IP 를 우선한다 (docker 브리지 172.17.x 를 피하려고)
PRIMARY="$(ip route get 1.1.1.1 2>/dev/null | grep -oP 'src \K[\d.]+' || true)"
if [[ -z "$PRIMARY" ]]; then
  PRIMARY="$(printf '%s\n' "${IPS[@]}" | grep -m1 -E '^(192\.168|10)\.' || printf '%s' "${IPS[0]}")"
fi
echo
echo "헤드셋 브라우저에서 접속:  https://$PRIMARY:8443/"
echo "  (안 붙으면 다른 후보: ${IPS[*]})"
echo "  1) 인증서 경고를 수락한다 (최초 1회)"
echo "  2) 'VR 시작' 을 누른다"
