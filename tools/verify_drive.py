"""Isaac 주행 결과(drive_isaac 의 JSON)가 계획대로 됐는지 본다.

    python -m tools.verify_drive out/drive/drive_007_ORD_02.json

본다:
  - 끝까지 갔다 (스텝 한도에 안 걸림), 정차 수 = 주문 품목 수
  - 정차마다 위치 오차 ≤ 5 cm, yaw 오차 ≤ 3°
  - 충돌 프레임 0, 최소 간격 ≥ 0 (본체가 진열대·기둥·벽 AABB 안에 들어간 적 없음)
  - 주행 거리가 계획의 0.95~1.25 배 (경로를 크게 벗어나지 않았다)
  - 도크 복귀 오차 ≤ 5 cm
  - 궤적이 모두 매장 바닥 안
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from scene.constants import SHELF, STORE
from tools.verify_shelf import Check

POS_TOL, YAW_TOL = 0.05, 3.0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("json", nargs="?", default="out/drive/drive_007_ORD_02.json")
    args = ap.parse_args()
    d = json.loads(Path(args.json).read_text())
    sc = json.loads(Path(d["scenario"]).read_text())
    order = next(o for o in sc["orders"] if o["id"] == d["order"])
    c = Check()
    print(f"\n{args.json}  ({d['order']}, {d['robot']})\n")

    c.true("끝까지 주행함 (스텝 한도 안)", d["completed"], f"시뮬 {d['sim_time_s']} s, 벽시계 {d['wall_time_s']} s")
    c.eq("정차 수 = 주문 품목 수", len(d["picks"]), len(order["lines"]), tol=0)
    worst_pos = max((p["pos_err_m"] for p in d["picks"]), default=0)
    worst_yaw = max((abs(p["yaw_err_deg"]) for p in d["picks"]), default=0)
    c.true(f"정차 위치 오차 ≤ {POS_TOL * 100:.0f} cm", worst_pos <= POS_TOL, f"최대 {worst_pos * 100:.1f} cm")
    c.true(f"정차 yaw 오차 ≤ {YAW_TOL:.0f}°", worst_yaw <= YAW_TOL, f"최대 {worst_yaw:.1f}°")
    c.true("정차가 주문 품목과 1:1", sorted(p["line"] for p in d["picks"]) == list(range(len(order["lines"]))), "")
    c.true("충돌 프레임 0", d["collision_frames"] == 0, f"{d['collision_frames']}프레임")
    c.true("최소 간격 ≥ 0 (장애물 안에 들어간 적 없음)", d["min_clearance_m"] >= 0, f"{d['min_clearance_m'] * 100:.1f} cm @ {d['min_clearance_at']}")
    ratio = d["driven_length_m"] / d["planned_length_m"]
    c.true("주행 거리 / 계획 거리 0.95~1.25", 0.95 <= ratio <= 1.25, f"{d['driven_length_m']} / {d['planned_length_m']} = {ratio:.3f}")
    c.true(f"도크 복귀 오차 ≤ {POS_TOL * 100:.0f} cm", d["dock_return_err_m"] <= POS_TOL, f"{d['dock_return_err_m'] * 100:.1f} cm")
    lx, ly = STORE.footprint(SHELF)
    outside = [t for t in d["trace"] if not (0 <= t[1] <= lx and 0 <= t[2] <= ly)]
    c.true("궤적이 매장 바닥 안", not outside, f"{len(d['trace'])}점" if not outside else f"{len(outside)}점 밖")
    return 1 if c.report() else 0


if __name__ == "__main__":
    sys.exit(main())
