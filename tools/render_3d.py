"""USD 장면을 3D 로 그려 PNG / 회전 GIF 로 저장한다.

    python -m tools.render_3d out/shelf.usda --gif docs/img/shelf_3d.gif
    python -m tools.render_3d out/store.usda --gif docs/img/store_3d.gif --png docs/img/store_3d.png

Isaac Sim 없이 형상만 확인하는 용도다. USD 의 Cube 프림을 월드 AABB 로 그린다
(배치 회전이 전부 90° 배수라 AABB 가 실제 상자와 같다). 실제 렌더는 Isaac 에서.
matplotlib 이 필요하다.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from pxr import Usd, UsdGeom

from tools.verify_shelf import world_bbox

# 프림 이름으로 색을 정한다 — 구조가 한눈에 구분되게
COLORS = [
    ("Post_", "#555a60"), ("Back", "#c9c4b8"), ("Kick", "#3a3a3a"),
    ("Rail_", "#d94b4b"), ("Level_00", "#8a7a66"), ("Level_", "#e0d6c4"),
    ("Column", "#333333"), ("Wall", "#d8d8d8"), ("Ceiling", "#f4f4f4"),
]


def color_of(name: str) -> str:
    for key, col in COLORS:
        if name.startswith(key):
            return col
    return "#bbbbbb"


def box_faces(lo, hi):
    x0, y0, z0 = lo
    x1, y1, z1 = hi
    v = [(x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0),
         (x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)]
    idx = [(0, 1, 2, 3), (4, 5, 6, 7), (0, 1, 5, 4), (2, 3, 7, 6), (1, 2, 6, 5), (0, 3, 7, 4)]
    return [[v[i] for i in f] for f in idx]


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("usd")
    ap.add_argument("--png", default=None)
    ap.add_argument("--gif", default=None)
    ap.add_argument("--frames", type=int, default=36)
    ap.add_argument("--elev", type=float, default=28.0)
    ap.add_argument("--azim", type=float, default=-60.0, help="PNG 시점·GIF 시작 각도")
    ap.add_argument("--skip", default="Ceiling,/Walls/", help="가리는 프림은 뺀다 (경로 부분 문자열, 콤마 구분)")
    args = ap.parse_args()
    skip = tuple(s for s in args.skip.split(",") if s)

    stage = Usd.Stage.Open(args.usd)
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])

    fig = plt.figure(figsize=(7, 6))
    ax = fig.add_subplot(111, projection="3d")
    lo_all = [1e9] * 3
    hi_all = [-1e9] * 3
    faces, colors = [], []
    # 모든 면을 한 컬렉션에 넣어야 상자끼리 앞뒤가 제대로 정렬된다
    for prim in Usd.PrimRange(stage.GetPseudoRoot()):
        if not prim.IsA(UsdGeom.Cube):
            continue
        if any(k in str(prim.GetPath()) for k in skip):
            continue
        lo, hi = world_bbox(cache, prim)
        lo_all = [min(a, b) for a, b in zip(lo_all, lo)]
        hi_all = [max(a, b) for a, b in zip(hi_all, hi)]
        fs = box_faces(lo, hi)
        faces += fs
        colors += [color_of(prim.GetName())] * len(fs)

    # 바닥은 메시라 따로: 있으면 연한 판으로
    floor = stage.GetPrimAtPath("/World/Store/Floor")
    if floor:
        lo, hi = world_bbox(cache, floor)
        faces.append([(lo[0], lo[1], 0), (hi[0], lo[1], 0), (hi[0], hi[1], 0), (lo[0], hi[1], 0)])
        colors.append("#eeeae2")
        lo_all = [min(a, b) for a, b in zip(lo_all, lo)]
        hi_all = [max(a, b) for a, b in zip(hi_all, hi)]

    ax.add_collection3d(Poly3DCollection(faces, facecolors=colors, edgecolors="#00000033", linewidths=0.3, zsort="average"))

    # 축 범위 = 실제 형상 범위, 상자 비율 = 실제 치수 비율. 왜곡 없이 그린다.
    ext = [max(h - l, 0.1) for l, h in zip(lo_all, hi_all)]
    ax.set_xlim(lo_all[0], hi_all[0])
    ax.set_ylim(lo_all[1], hi_all[1])
    ax.set_zlim(0, hi_all[2])
    ax.set_box_aspect(ext, zoom=1.25)
    ax.set_axis_off()
    fig.tight_layout(pad=0)

    if args.png:
        ax.view_init(elev=args.elev, azim=args.azim)
        Path(args.png).parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(args.png, dpi=110)
        print(f"저장: {args.png}")

    if args.gif:
        def update(i):
            ax.view_init(elev=args.elev, azim=args.azim + 360 * i / args.frames)
            return ()
        anim = FuncAnimation(fig, update, frames=args.frames, blit=False)
        Path(args.gif).parent.mkdir(parents=True, exist_ok=True)
        anim.save(args.gif, writer=PillowWriter(fps=12), dpi=70)
        print(f"저장: {args.gif}")


if __name__ == "__main__":
    main()
