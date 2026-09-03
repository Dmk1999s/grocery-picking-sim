"""매장 USD 를 위에서 내려다본 평면도 PNG 로 뽑는다.

    python -m tools.plan_store out/store.usda --out out/store_plan.png

USD 안의 실제 형상 AABB 를 그린다 (constants 가 아니라). 실측한 마트의
도면·사진과 나란히 놓고 비교하는 용도다. matplotlib 이 필요하다.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager

# 한글 라벨용. 없으면 네모로 나오지만 도면 자체는 멀쩡하다.
# Ubuntu: sudo apt install fonts-nanum / macOS: AppleGothic 기본 내장
_KR = {"NanumGothic", "AppleGothic", "Malgun Gothic", "Noto Sans CJK KR"}
_have = _KR & {f.name for f in font_manager.fontManager.ttflist}
if _have:
    plt.rcParams["font.family"] = sorted(_have)[0]
plt.rcParams["axes.unicode_minus"] = False
from matplotlib.patches import Rectangle
from pxr import Usd, UsdGeom, UsdLux

from scene.constants import ROBOT, SHELF, STORE
from scene.store import corridors, placements
from tools.verify_shelf import world_bbox

COLORS = {"wall": "#4a6fa5", "gondola": "#c97b3a", "endcap": "#b23a48"}


def draw_store(ax, stage: Usd.Stage, cache: UsdGeom.BBoxCache, *, robot_marker: bool = True, light_scale: dict | None = None) -> tuple[float, float]:
    """매장 평면 요소를 ax 에 그린다. plan_scenario 가 같은 바탕 위에 경로를 얹는다.

    light_scale: 조명 프림 경로 → 세기 배율. 주면 꺼진 등(0) 은 회색 실선으로 그린다.
    """
    lx, ly = STORE.footprint(SHELF)
    ax.set_aspect("equal")

    # 타일 격자
    t = STORE.tile
    for x in range(int(lx / t) + 1):
        ax.axvline(x * t, color="#ddd", lw=0.4, zorder=0)
    for y in range(int(ly / t) + 1):
        ax.axhline(y * t, color="#ddd", lw=0.4, zorder=0)

    # 통로
    for cor in corridors():
        (x0, x1), (y0, y1) = cor["x"], cor["y"]
        ax.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, fc="#eef6e8", ec="none", zorder=1))
        ax.text((x0 + x1) / 2, (y0 + y1) / 2, cor["name"], ha="center", va="center", fontsize=7, color="#5a7d4a", zorder=6)

    # 진열대 (USD 에서 읽은 AABB)
    for p in placements():
        prim = stage.GetPrimAtPath(p["path"])
        lo, hi = world_bbox(cache, prim)
        ax.add_patch(Rectangle((lo[0], lo[1]), hi[0] - lo[0], hi[1] - lo[1], fc=COLORS[p["kind"]], ec="k", lw=0.4, zorder=3))
        # 앞면 표시: 레일 쪽에 흰 선
        if p["kind"] == "endcap":
            y = lo[1] if p["side"] == "Front" else hi[1]
            ax.plot([lo[0], hi[0]], [y, y], color="w", lw=1.5, zorder=4)
        else:
            x = hi[0] if p["side"] == "L" else lo[0]
            ax.plot([x, x], [lo[1], hi[1]], color="w", lw=1.5, zorder=4)

    # 기둥 · 벽
    for prim in stage.GetPrimAtPath("/World/Store/Columns").GetChildren():
        lo, hi = world_bbox(cache, prim)
        ax.add_patch(Rectangle((lo[0], lo[1]), hi[0] - lo[0], hi[1] - lo[1], fc="#333", zorder=5))
    for prim in stage.GetPrimAtPath("/World/Store/Walls").GetChildren():
        lo, hi = world_bbox(cache, prim)
        ax.add_patch(Rectangle((lo[0], lo[1]), hi[0] - lo[0], hi[1] - lo[1], fc="#888", zorder=5))

    # 조명 (점선). 시나리오에서 꺼진 등은 회색 실선
    for prim in stage.GetPrimAtPath("/World/Store/Lights").GetChildren():
        lo, hi = world_bbox(cache, prim)
        off = light_scale is not None and light_scale.get(str(prim.GetPath()), 1.0) == 0
        ax.add_patch(Rectangle((lo[0], lo[1]), hi[0] - lo[0], hi[1] - lo[1], fc="none",
                               ec="#999" if off else "#e6b800", ls="-" if off else "--", lw=1.2 if off else 0.8, zorder=6))

    # AMR 크기 비교용
    if robot_marker:
        r = ROBOT
        ax.add_patch(Rectangle((STORE.aisle_x_range(0, SHELF)[0] + (STORE.aisle_width - r.base_w) / 2, ly / 2 - r.base_l / 2),
                               r.base_w, r.base_l, fc="#6c6", ec="k", lw=0.6, zorder=7))

    ax.set_xlim(-STORE.wall_t - 0.2, lx + STORE.wall_t + 0.2)
    ax.set_ylim(-STORE.wall_t - 0.2, ly + STORE.wall_t + 0.2)
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
    return lx, ly


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("usd", nargs="?", default="out/store.usda")
    ap.add_argument("--out", default="out/store_plan.png")
    args = ap.parse_args()

    stage = Usd.Stage.Open(args.usd)
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    lx, ly = STORE.footprint(SHELF)

    fig, ax = plt.subplots(figsize=(6, 6 * ly / lx + 1))
    draw_store(ax, stage, cache)
    ax.set_title(f"{Path(args.usd).name}  {lx:.1f} × {ly:.1f} m  (타일 {STORE.tile * 100:.0f} cm)", fontsize=9)
    fig.tight_layout()
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.out, dpi=130)
    print(f"저장: {args.out}")


if __name__ == "__main__":
    main()
