"""생성된 매장 USD 를 되읽어 constants 와 대조하고, AMR 통과 가능성을 본다.

    python -m tools.verify_store out/store.usda

두 종류를 본다:
  구조   벽·바닥·천장·기둥·조명·진열대가 constants 대로 놓였는가
  통행   AMR 이 부통로를 지나고 주통로에서 돌 수 있는가 — 기둥·엔드캡 포함

통행 검사는 USD 안의 실제 형상 AABB 로 한다. constants 만 보고 계산하면
배치 코드의 실수(회전 방향, 오프셋)를 못 잡는다.
"""

from __future__ import annotations

import argparse
import sys
from itertools import combinations

from pxr import Usd, UsdGeom, UsdLux, UsdPhysics

from scene.constants import ROBOT, SHELF, STORE
from scene.store import corridors, placements
from tools.verify_shelf import Check, world_bbox

TOL = 1e-3  # 배치는 mm 면 충분


def overlap_2d(a, b, tol=TOL) -> bool:
    """두 AABB(min,max) 가 xy 평면에서 tol 보다 깊게 겹치는가."""
    return (
        a[0][0] < b[1][0] - tol and b[0][0] < a[1][0] - tol
        and a[0][1] < b[1][1] - tol and b[0][1] < a[1][1] - tol
    )


def free_width(band_lo: float, band_hi: float, axis: int, obstacles: list, other_lo: float, other_hi: float) -> float:
    """통로 폭 방향 `axis` 에서, 통로 구간 안(other 축)에 걸친 장애물을 피해
    지나갈 수 있는 최대 연속 폭. 장애물 한쪽으로 비켜 간다고 본다."""
    width = band_hi - band_lo
    o = 1 - axis
    for lo, hi in obstacles:
        if hi[o] <= other_lo + TOL or lo[o] >= other_hi - TOL:
            continue                          # 통로 길이 방향으로 안 걸침
        if hi[axis] <= band_lo + TOL or lo[axis] >= band_hi - TOL:
            continue                          # 통로 폭 방향으로 안 걸침
        width = min(width, max(lo[axis] - band_lo, band_hi - hi[axis]))
    return width


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("usd", nargs="?", default="out/store.usda")
    args = ap.parse_args()

    stage = Usd.Stage.Open(args.usd)
    if stage is None:
        print(f"열 수 없음: {args.usd}")
        return 2

    st, sh, rb = STORE, SHELF, ROBOT
    lx, ly = st.footprint(sh)
    c = Check()
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    print(f"\n{args.usd}\n")

    # 스테이지 규약 -----------------------------------------
    c.true("up axis = Z", UsdGeom.GetStageUpAxis(stage) == UsdGeom.Tokens.z)
    c.eq("metersPerUnit", UsdGeom.GetStageMetersPerUnit(stage), 1.0)

    # 건물 ---------------------------------------------------
    floor = stage.GetPrimAtPath("/World/Store/Floor")
    c.true("바닥 존재", bool(floor))
    if floor:
        lo, hi = world_bbox(cache, floor)
        c.eq("바닥 윗면 z=0", hi[2], 0.0)
        c.eq("바닥 크기 X", hi[0] - lo[0], lx)
        c.eq("바닥 크기 Y", hi[1] - lo[1], ly)
        c.true("바닥에 CollisionAPI", floor.HasAPI(UsdPhysics.CollisionAPI))
        c.true("바닥에 st (타일 텍스처 좌표)", UsdGeom.PrimvarsAPI(floor).HasPrimvar("st"))

    ceil = stage.GetPrimAtPath("/World/Store/Ceiling")
    c.true("천장 존재", bool(ceil))
    if ceil:
        lo, hi = world_bbox(cache, ceil)
        c.eq("천장 밑면 높이", lo[2], st.ceiling_h)

    walls = [p for p in stage.GetPrimAtPath("/World/Store/Walls").GetChildren() if p.IsA(UsdGeom.Cube)]
    c.eq("벽 4면", len(walls), 4)
    if walls:
        wlo = [min(world_bbox(cache, w)[0][i] for w in walls) for i in range(3)]
        whi = [max(world_bbox(cache, w)[1][i] for w in walls) for i in range(3)]
        c.eq("벽이 바닥을 둘러쌈 (x min)", wlo[0], -st.wall_t)
        c.eq("벽이 바닥을 둘러쌈 (x max)", whi[0], lx + st.wall_t)
        c.eq("벽이 바닥을 둘러쌈 (y min)", wlo[1], -st.wall_t)
        c.eq("벽이 바닥을 둘러쌈 (y max)", whi[1], ly + st.wall_t)
        c.eq("벽 높이 = 천장", whi[2], st.ceiling_h)

    cols = [p for p in stage.GetPrimAtPath("/World/Store/Columns").GetChildren() if p.IsA(UsdGeom.Cube)]
    c.eq("기둥 개수", len(cols), len(st.columns))
    col_boxes = [world_bbox(cache, p) for p in cols]
    for i, (lo, hi) in enumerate(col_boxes):
        c.eq(f"Column_{i:02d} 단면", hi[0] - lo[0], st.column_size)
        c.eq(f"Column_{i:02d} 바닥~천장", hi[2] - lo[2], st.ceiling_h)

    lights = [p for p in stage.GetPrimAtPath("/World/Store/Lights").GetChildren() if p.IsA(UsdLux.RectLight)]
    cors = corridors(st, sh)
    c.eq("조명 = 통로 수", len(lights), len(cors))
    for i, (p, cor) in enumerate(zip(lights, cors)):
        lo, hi = world_bbox(cache, p)
        (x0, x1), (y0, y1) = cor["x"], cor["y"]
        c.true(
            f"Light_{i:02d} 가 {cor['name']} 위 천장 근처",
            abs((lo[0] + hi[0]) / 2 - (x0 + x1) / 2) < TOL
            and abs((lo[1] + hi[1]) / 2 - (y0 + y1) / 2) < TOL
            and st.ceiling_h - 0.2 < lo[2] <= st.ceiling_h,
            f"z={lo[2]:.2f}",
        )

    # 진열대 배치 -------------------------------------------
    want = placements(st, sh)
    units = []
    for p in want:
        prim = stage.GetPrimAtPath(p["path"])
        if not prim:
            c.true(f"{p['path']} 존재", False)
            continue
        units.append((p, prim, world_bbox(cache, prim)))
    c.eq("진열대 대수", len(units), len(want))

    bad_floor = [p["path"] for p, _, (lo, hi) in units if abs(lo[2]) > TOL]
    c.true("모든 진열대가 바닥에 닿음", not bad_floor, f"{len(bad_floor)}대 떠 있음")

    bad_in = [
        p["path"] for p, _, (lo, hi) in units
        if lo[0] < -TOL or hi[0] > lx + TOL or lo[1] < -TOL or hi[1] > ly + TOL
    ]
    c.true("모든 진열대가 벽 안에 있음", not bad_in, f"{len(bad_in)}대 벽 침범" if bad_in else "")

    bad_h = [
        p["path"] for p, _, (lo, hi) in units
        if abs((hi[2] - lo[2]) - p["spec"].height) > TOL
    ]
    c.true("진열대 높이가 종류별 spec 과 같음", not bad_h, f"{bad_h[:2]}" if bad_h else "벽면 2.10 / 곤돌라 1.80")

    overlaps = [
        (a[0]["path"], b[0]["path"])
        for a, b in combinations(units, 2)
        if overlap_2d(a[2], b[2])
    ]
    c.true("진열대끼리 겹치지 않음", not overlaps, f"{overlaps[:2]}" if overlaps else f"{len(units)}대 쌍 검사")

    hit_col = [
        (p["path"], i)
        for p, _, bb in units
        for i, cb in enumerate(col_boxes)
        if overlap_2d(bb, cb)
    ]
    c.true("진열대가 기둥을 파고들지 않음", not hit_col, f"{hit_col[:2]}" if hit_col else "")

    # 면이 통로를 보는가: 벽면/곤돌라는 앞면 x, 엔드캡은 앞면 y 가 통로 경계
    # 앞면 = 가격표 레일 앞면이라 rail_t 만큼 통로로 나온다.
    r = sh.rail_t
    bad_face = []
    for p, _, (lo, hi) in units:
        if p["kind"] == "endcap":
            if p["side"] == "Front":
                ok = abs(lo[1] - (st.main_aisle_width - r)) < TOL
            else:
                ok = abs(hi[1] - (ly - st.main_aisle_width + r)) < TOL
        else:
            x_lo, x_hi = st.aisle_x_range(p["aisle"], sh)
            ok = abs(hi[0] - (x_lo + r)) < TOL if p["side"] == "L" else abs(lo[0] - (x_hi - r)) < TOL
        if not ok:
            bad_face.append(p["path"])
    c.true("모든 진열대 앞면이 자기 통로 경계에 있음", not bad_face, f"{bad_face[:3]}" if bad_face else "레일 두께만큼 통로로 나옴")

    # 벽면 진열대 등이 벽에 붙는가
    bad_wall = []
    for p, _, (lo, hi) in units:
        if p["kind"] != "wall":
            continue
        if p["side"] == "L" and abs(lo[0]) > TOL:
            bad_wall.append(p["path"])
        if p["side"] == "R" and abs(hi[0] - lx) > TOL:
            bad_wall.append(p["path"])
    c.true("벽면 진열대 뒷면이 벽에 닿음", not bad_wall, f"{bad_wall[:2]}" if bad_wall else "")

    # 통행 --------------------------------------------------
    # 장애물 = 진열대 전체 + 기둥. 통로 정의 밖의 형상만 걸리는 게 정상이다.
    obstacles = [bb for _, _, bb in units] + col_boxes
    need_go, need_turn = rb.corridor_width(), rb.turn_diameter()
    for cor in cors:
        (x0, x1), (y0, y1) = cor["x"], cor["y"]
        if cor["axis"] == "y":
            fw = free_width(x0, x1, 0, obstacles, y0, y1)
        else:
            fw = free_width(y0, y1, 1, obstacles, x0, x1)
        c.true(
            f"{cor['name']}: 직진 여유폭 ≥ {need_go:.2f}",
            fw >= need_go - TOL,
            f"{fw:.2f} m (통로 {(x1 - x0) if cor['axis'] == 'y' else (y1 - y0):.2f}, 레일·기둥 반영)",
        )
        if cor["axis"] == "y":
            # 부통로 안에서 돌 수 있으면 막다른 상황에서 후진 없이 빠져나온다.
            c.true(
                f"{cor['name']}: 제자리 회전 ≥ {need_turn:.2f}",
                fw >= need_turn - TOL,
                f"{fw:.2f} m",
            )

    # 부통로 입구(주통로와 만나는 사각 구간)에서 돌 수 있는가 — 기둥·엔드캡 포함.
    # 장애물을 x 로 비키든 y 로 비키든 한쪽만 되면 된다. 남는 사각형의 짧은 변이
    # 회전 지름 이상이어야 한다.
    for a in range(st.n_aisles):
        x0, x1 = st.aisle_x_range(a, sh)
        for tag, (y0, y1) in zip(("앞", "뒤"), st.main_aisle_y_ranges(sh)):
            fx = free_width(x0, x1, 0, obstacles, y0, y1)   # x 로 비킴, y 는 전폭
            fy = free_width(y0, y1, 1, obstacles, x0, x1)   # y 로 비킴, x 는 전폭
            square = max(min(fx, y1 - y0), min(x1 - x0, fy))
            c.true(
                f"부통로 {a} {tag} 입구에서 회전 가능",
                square >= need_turn - TOL,
                f"자유 정사각 {square:.2f} m (x비킴 {fx:.2f}, y비킴 {fy:.2f}), 필요 {need_turn:.2f}",
            )

    return 1 if c.report() else 0


if __name__ == "__main__":
    sys.exit(main())
