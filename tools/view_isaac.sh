#!/usr/bin/env bash
# Isaac Sim GUI 를 EC2 에서 띄우고 WebRTC 로 스트리밍한다.
#
#   bash tools/view_isaac.sh out/store_stocked.usda
#
# 노트북에서는 "Isaac Sim WebRTC Streaming Client" 를 받아 이 인스턴스의 공인 IP 로 접속한다.
# 보안그룹에 내 IP 에서 TCP 49100, UDP 47998 (안전하게는 TCP/UDP 47995-48012, 49000-49007, 49100)
# 인바운드가 열려 있어야 한다. 접속을 끊어도 앱은 살아 있다 (quitOnSessionEnded=false).
set -euo pipefail
USD="${1:-out/store_stocked.usda}"
USD_ABS="$(readlink -f "$USD")"
source ~/.isaac_cache_env
PUBLIC_IP="$(curl -s -m 3 http://169.254.169.254/latest/meta-data/public-ipv4 || true)"
echo "USD: $USD_ABS"
echo "스트리밍 주소: ${PUBLIC_IP:-<공인 IP 조회 실패>}  (TCP 49100 시그널링, UDP 47998 미디어)"

# 시작 후 스테이지를 연다. --exec 는 Kit 이 부팅을 마친 뒤 스크립트를 돌린다.
OPEN_SCRIPT="$(mktemp --suffix=.py)"
cat > "$OPEN_SCRIPT" <<PY
import omni.usd
omni.usd.get_context().open_stage(r"$USD_ABS")
print("[view_isaac] 스테이지 열림: $USD_ABS")
PY

exec ~/isaac6-venv/bin/isaacsim isaacsim.exp.full.streaming --no-window \
  --/exts/omni.kit.livestream.app/primaryStream.publicIp="$PUBLIC_IP" \
  --/exts/omni.services.livestream.session/quitOnSessionEnded=false \
  --exec "$OPEN_SCRIPT"
