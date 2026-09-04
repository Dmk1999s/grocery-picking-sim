"""시나리오 평면도 — 매장 위에 주문 경로·정차·파지 대상을 얹는다.

    python -m tools.plan_scenario out/scenario_007.json --out out/scenario_007_plan.png
    python -m tools.plan_scenario out/scenario_007.json --order 2      # 한 주문만
    python -m tools.plan_scenario out/scenario_007.json --order 2 --drive out/drive/drive_007_ORD_02.json   # + 실제 궤적

바탕은 plan_store 와 같은 USD 기반 평면도다. 그 위에
  경로       주문마다 색. 도크(★) → 경유점 → 정차(●, 번호 = 주문 안 방문 순서) → 도크
  파지 대상  정차점에서 진열대 안 상품 중심으로 짧은 선 (팔이 들어가는 방향)
  넘어짐 ×  오배치 ▲  꺼진 등(회색 실선)   — 시나리오 흔들림이 어디 있는지
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from pxr import Usd, UsdGeom

from scene.constants import ROBOT
from tools.plan_store import draw_store

ORDER_COLORS = ["#d62728", "#1f77b4", "#2ca02c", "#9467bd", "#ff7f0e", "#17becf", "#8c564b"]


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("json", nargs="?", default="out/scenario_007.json")
    ap.add_argument("--out", default=None, help="기본 <json 이름>_plan.png")
    ap.add_argument("--order", type=int, default=None, help="이 주문만 그린다 (0부터)")
    ap.add_argument("--drive", default=None, help="drive_isaac 결과 JSON. 실제 주행 궤적을 검정 선으로 겹친다")
    args = ap.parse_args()

    sc = json.loads(Path(args.json).read_text())
    stage = Usd.Stage.Open(sc["usd"])
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])

    fig, ax = plt.subplots(figsize=(7.5, 7.5 * 17.6 / 25.6 + 1.2))
    lx, ly = draw_store(ax, stage, cache, robot_marker=False,
                        light_scale={k: v["scale"] for k, v in sc["lights"].items()})

    # 흔들림 표시: 넘어짐 · 오배치 (맨 앞 상품 기준, USD 에서 읽는다)
    fallen, misplaced = [], []
    for prim in stage.Traverse():
        if not prim.GetPath().name.startswith("Item_") or prim.GetAttribute("stock:facing").Get() != 0:
            continue
        r = cache.ComputeWorldBound(prim).ComputeAlignedRange()
        c = (r.GetMin() + r.GetMax()) / 2
        if prim.GetAttribute("stock:state").Get() == "fallen":
            fallen.append((c[0], c[1]))
        if prim.GetAttribute("stock:misplaced").Get():
            misplaced.append((c[0], c[1]))
    if fallen:
        ax.scatter(*zip(*fallen), marker="x", s=14, c="#c00", lw=0.8, zorder=8, label=f"넘어짐 {len(fallen)}")
    if misplaced:
        ax.scatter(*zip(*misplaced), marker="^", s=10, c="#7b2cbf", lw=0, zorder=8, label=f"오배치 {len(misplaced)}")

    # 경로
    orders = sc["orders"] if args.order is None else [sc["orders"][args.order]]
    for oi, o in enumerate(orders):
        col = ORDER_COLORS[(args.order if args.order is not None else oi) % len(ORDER_COLORS)]
        wp = o["route"]["waypoints"]
        ax.plot([w["x"] for w in wp], [w["y"] for w in wp], color=col, lw=1.4, alpha=0.85, zorder=9,
                label=f"{o['id']}  {o['route']['length_m']:.0f} m")
        picks = [w for w in wp if w["kind"] == "pick"]
        for n, w in enumerate(picks, 1):
            line = o["lines"][w["line"]]
            # 정차 footprint
            rw, rl = ROBOT.base_w, ROBOT.base_l
            if abs(w["yaw_deg"]) % 180 == 90:
                rw, rl = rl, rw
            ax.add_patch(Rectangle((w["x"] - rl / 2, w["y"] - rw / 2), rl, rw, fc=col, ec="k", lw=0.4, alpha=0.5, zorder=9))
            cx, cy, _ = line["item_center"]
            ax.plot([w["x"], cx], [w["y"], cy], color=col, lw=0.8, zorder=10)
            ax.plot(cx, cy, "o", ms=3, color=col, mec="k", mew=0.4, zorder=11)
            ax.text(w["x"], w["y"], str(n), ha="center", va="center", fontsize=6, color="w", weight="bold", zorder=12)
    if args.drive:
        dr = json.loads(Path(args.drive).read_text())
        tr = dr["trace"]
        ax.plot([t[1] for t in tr], [t[2] for t in tr], color="k", lw=0.9, ls=(0, (2, 1.2)), zorder=11,
                label=f"실제 주행 {dr['order']}  {dr['driven_length_m']:.0f} m / {dr['sim_time_s']:.0f} s  최소 간격 {dr['min_clearance_m'] * 100:.0f} cm")
        for pk in dr["picks"]:
            ax.plot(pk["actual"][0], pk["actual"][1], "k+", ms=7, mew=1.2, zorder=13)
        if dr.get("loc_trace"):
            lt = dr["loc_trace"]; L = dr.get("localization") or {}
            ax.plot([t[3] for t in lt], [t[4] for t in lt], color="#d62728", lw=0.8, alpha=0.8, zorder=12,
                    label=f"추정 위치 (라이다+PF)  RMS {L.get('rms_pos_m', 0) * 100:.1f} cm, 최대 {L.get('max_pos_m', 0) * 100:.0f} cm")
    d = sc["dock"]
    ax.plot(d["x"], d["y"], marker="*", ms=12, color="#222", zorder=12, label="도크")

    s = sc["stats"]
    ax.set_title(
        f"seed {sc['seed']}  상품 {s['items']}  빈 자리 {100 - sc['params']['fill'] * 100:.0f} %  "
        f"yaw 흔들림 {s['jittered']}  넘어짐 {s['fallen']}  오배치 {s['misplaced']}  꺼진 등 {s['lights_off']}",
        fontsize=9,
    )
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.08), ncol=4, fontsize=7, frameon=False)
    fig.tight_layout()
    out = Path(args.out) if args.out else Path(args.json).with_name(Path(args.json).stem + "_plan.png")
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=140)
    print(f"저장: {out}")


if __name__ == "__main__":
    main()
