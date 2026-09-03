"""팔 어깨 높이·도달 반경에 따라 맨 앞 상품 중 몇 %에 닿는지 본다.

    python -m tools.reach_study out/store_stocked.usda --out docs/img/reach_study.png

정차 자세는 scenario.pick_pose 와 같다 (진열대 앞면에서 standoff, 왼쪽 마운트).
상품 좌표는 USD 에서 되읽는다. 결과는 마운트 높이(x) × 도달 반경(선) 그래프와
단별 표. 팔·리프트 사양을 이 숫자로 정한다 — 감으로 정하지 않기 위한 도구.
"""

from __future__ import annotations

import argparse
from dataclasses import replace
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from pxr import Usd, UsdGeom

from scene.constants import ROBOT, SHELF, STORE
from scene.scenario import pick_pose
from scene.store import placements
from tools.plan_store import _have  # 한글 폰트 설정 부수효과

HEIGHTS = np.round(np.arange(0.30, 1.21, 0.05), 2)
REACHES = [0.60, 0.85, 1.00, 1.30]          # [표준] UR3e 0.5 / UR5e 0.85 / Franka 0.855 / UR10e 1.3 급


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("usd", nargs="?", default="out/store_stocked.usda")
    ap.add_argument("--out", default="out/reach_study.png")
    args = ap.parse_args()

    stage = Usd.Stage.Open(args.usd)
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    units = {u["path"]: u for u in placements(STORE, SHELF)}
    centers, levels = [], []
    for path, unit in units.items():
        stock = stage.GetPrimAtPath(f"{path}/Stock")
        if not stock:
            continue
        for prim in stock.GetChildren():
            if prim.GetAttribute("stock:facing").Get() != 0:
                continue
            r = cache.ComputeWorldBound(prim).ComputeAlignedRange()
            lo, hi = r.GetMin(), r.GetMax()
            centers.append((unit, tuple((lo[i] + hi[i]) / 2 for i in range(3))))
            levels.append(prim.GetAttribute("stock:level").Get())
    levels = np.array(levels)
    n = len(centers)
    print(f"맨 앞 상품 {n}개, 단 0~{levels.max()}")

    # 마운트→상품 거리는 높이에만 의존하므로 한 번 계산해 두고 반경만 바꾼다
    dist = np.zeros((len(HEIGHTS), n))
    for hi_, h in enumerate(HEIGHTS):
        rb = replace(ROBOT, arm_base_dz=float(h) - ROBOT.arm_shoulder_dz)   # x 축 = 어깨 높이
        for j, (unit, c) in enumerate(centers):
            dist[hi_, j] = pick_pose(unit, c, rb)["reach_m"]

    fig, ax = plt.subplots(figsize=(7, 4))
    for r in REACHES:
        frac = (dist <= r).mean(axis=1) * 100
        ax.plot(HEIGHTS, frac, marker="o", ms=3, label=f"도달 {r:.2f} m")
        best = int(np.argmax(frac))
        print(f"  도달 {r:.2f} m: 최고 {frac[best]:.0f} % @ 마운트 {HEIGHTS[best]:.2f} m   (어깨 1.00 m 에서 {frac[HEIGHTS.tolist().index(1.0)]:.0f} %)")
    ax.axvline(ROBOT.arm_mount_z(), color="#888", ls="--", lw=0.8)
    ax.text(ROBOT.arm_mount_z() + 0.01, 3, f"현재 어깨 {ROBOT.arm_mount_z():.2f} m\n(Franka on Carter 상판)", fontsize=8, color="#666")
    ax.set_xlabel("팔 어깨(도달 구 중심) 높이 [m]"); ax.set_ylabel("닿는 맨 앞 상품 [%]")
    ax.set_ylim(0, 102); ax.grid(alpha=0.3); ax.legend(fontsize=8)
    ax.set_title(f"정차 standoff {ROBOT.pick_standoff} m, 진열대 {SHELF.n_levels}단 (벽면 {STORE.wall_unit_levels}단)", fontsize=9)
    fig.tight_layout()
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.out, dpi=130)
    print(f"저장: {args.out}")

    # 단별 표: 도달 0.85 에서 마운트 몇 개 값
    print("\n단별 도달률 % (도달 0.85 m)")
    hs = [0.35, 0.60, 0.80, 1.00]
    print("  단   " + "".join(f"  h={h:.2f}" for h in hs))
    for lv in range(levels.max() + 1):
        m = levels == lv
        row = [(dist[HEIGHTS.tolist().index(h)][m] <= 0.85).mean() * 100 for h in hs]
        print(f"  {lv:<3} " + "".join(f"  {v:6.0f}" for v in row) + f"   ({m.sum()}개)")


if __name__ == "__main__":
    main()
