#!/bin/bash
# 파지 배치 — 시나리오 주문 전부를 순간이동 모드로 돌려 성공률을 낸다.
#
#   bash tools/batch_picks.sh suction s007 s003     # 흡착 (scenario_s007.json, scenario_s003.json)
#   bash tools/batch_picks.sh parallel 007 003      # 평행 그리퍼
#
# 결과: out/<gripper>_<seed>/drive_*.json, 요약은 표준출력. 주문 하나에 3~4분.
set -u
cd "$(dirname "$0")/.."
source ~/.isaac_cache_env
GRIPPER=${1:-suction}; shift || true
SEEDS=${*:-s007 s003}
[ "$GRIPPER" = "suction" ] && FLAG="--suction" || FLAG=""
for seed in $SEEDS; do
  out="out/${GRIPPER}_${seed}"; rm -rf "$out"; mkdir -p "$out"
  for o in 0 1 2 3 4; do
    timeout 1200 ~/isaac6-venv/bin/python -m tools.drive_isaac "out/scenario_${seed}.json" \
      --order $o --arm $FLAG --teleport --out "$out" > "$out/log_$o.txt" 2>&1
    echo "=== $GRIPPER $seed $o $(grep -oE '파지 성공 [0-9]+/[0-9]+' "$out/log_$o.txt")"
  done
done
echo "=== DONE $(date +%T)"
.venv/bin/python - "$GRIPPER" $SEEDS <<'PY'
import json, sys, glob, collections
g, seeds = sys.argv[1], sys.argv[2:]
rows = []
for sd in seeds:
    for f in sorted(glob.glob(f"out/{g}_{sd}/drive_*.json")):
        d = json.load(open(f))
        for p in d["picks"]:
            gr = p["grasp"]
            rows.append((p["product"].split("_", 1)[1], gr["phase"], bool(gr.get("success"))))
n = len(rows); ok = sum(r[2] for r in rows)
print(f"\n{g}: {n}회 중 성공 {ok} ({ok / max(1, n) * 100:.0f} %)")
print("실패 단계:", dict(collections.Counter(r[1] for r in rows if not r[2])))
PY
