"""생성된 진열대 USD 를 되읽어 constants 와 대조한다.

생성기가 의도대로 돌았는지 눈이 아니라 숫자로 본다. parking-patrol 의
expected_map 대조와 같은 역할이다.

    python -m tools.verify_shelf out/shelf.usda
"""

from __future__ import annotations

import argparse
import sys

from pxr import Usd, UsdGeom, UsdPhysics

from scene.constants import SHELF
from scene.shelf import slot_positions

TOL = 1e-4  # 0.1 mm


class Check:
    def __init__(self) -> None:
        self.rows: list[tuple[bool, str, str]] = []

    def eq(self, name: str, got: float, want: float, tol: float = TOL) -> None:
        ok = abs(got - want) <= tol
        self.rows.append((ok, name, f"{got:.5f} (기대 {want:.5f})"))

    def true(self, name: str, cond: bool, detail: str = "") -> None:
        self.rows.append((bool(cond), name, detail))

    def report(self) -> int:
        failed = 0
        for ok, name, detail in self.rows:
            mark = "PASS" if ok else "FAIL"
            if not ok:
                failed += 1
            print(f"  [{mark}] {name:<42} {detail}")
        print()
        n = len(self.rows)
        if failed:
            print(f"{n}개 중 {failed}개 실패")
        else:
            print(f"{n}개 전부 통과")
        return failed


def world_bbox(cache: UsdGeom.BBoxCache, prim: Usd.Prim):
    """프림의 월드 좌표 AABB (min, max) 를 돌려준다."""
    r = cache.ComputeWorldBound(prim).ComputeAlignedRange()
    return r.GetMin(), r.GetMax()


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("usd", nargs="?", default="out/shelf.usda")
    args = ap.parse_args()

    stage = Usd.Stage.Open(args.usd)
    if stage is None:
        print(f"열 수 없음: {args.usd}")
        return 2

    s = SHELF
    c = Check()

    # 스테이지 규약 -----------------------------------------
    print(f"\n{args.usd}\n")
    c.true(
        "up axis = Z",
        UsdGeom.GetStageUpAxis(stage) == UsdGeom.Tokens.z,
        str(UsdGeom.GetStageUpAxis(stage)),
    )
    c.eq("metersPerUnit", UsdGeom.GetStageMetersPerUnit(stage), 1.0)

    root = stage.GetPrimAtPath("/World/Shelf_00")
    c.true("루트 프림 존재", bool(root), str(root.GetPath()))
    if not root:
        return c.report() or 1

    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])

    # 전체 외형 ---------------------------------------------
    lo, hi = world_bbox(cache, root)
    c.eq("전체 폭 (Y)", hi[1] - lo[1], s.width, 1e-3)
    c.eq("전체 깊이 (X)", hi[0] - lo[0], s.depth, 1e-3)
    c.eq("전체 높이 (Z)", hi[2] - lo[2], s.height, 1e-3)
    c.eq("바닥이 z=0 에 닿음", lo[2], 0.0, 1e-3)
    c.eq("앞면이 x=0 에 있음", lo[0], 0.0, 1e-3)

    # 선반판 높이 -------------------------------------------
    want_z = s.level_heights()
    for i, wz in enumerate(want_z):
        p = stage.GetPrimAtPath(f"/World/Shelf_00/Level_{i:02d}")
        if not p:
            c.true(f"Level_{i:02d} 존재", False)
            continue
        lo_i, hi_i = world_bbox(cache, p)
        c.eq(f"Level_{i:02d} 윗면 z", hi_i[2], wz, 1e-3)
        c.eq(f"Level_{i:02d} 두께", hi_i[2] - lo_i[2], s.board_t, 1e-3)

    # 선반판이 측판 안쪽에 있는가 ---------------------------
    inner_half = s.width / 2 - s.side_t
    p0 = stage.GetPrimAtPath("/World/Shelf_00/Level_00")
    lo0, hi0 = world_bbox(cache, p0)
    c.true(
        "선반판이 측판을 파고들지 않음",
        lo0[1] >= -inner_half - TOL and hi0[1] <= inner_half + TOL,
        f"y ∈ [{lo0[1]:.3f}, {hi0[1]:.3f}], 허용 ±{inner_half:.3f}",
    )

    # 콜라이더 ----------------------------------------------
    geoms = [
        p
        for p in Usd.PrimRange(root)
        if p.IsA(UsdGeom.Cube)
    ]
    n_col = sum(1 for p in geoms if p.HasAPI(UsdPhysics.CollisionAPI))
    c.true(
        "모든 형상에 CollisionAPI",
        n_col == len(geoms) and len(geoms) > 0,
        f"{n_col}/{len(geoms)}",
    )
    c.eq("형상 개수 (측판2+뒷판1+선반N)", len(geoms), 3 + s.n_levels)

    # 슬롯이 선반 내부에 들어오는가 -------------------------
    slots = slot_positions(s)
    c.eq("슬롯 개수", len(slots), s.n_levels * s.slots_per_level)

    bad = []
    for sl in slots:
        half_w = sl["max_w"] / 2 + s.slot_margin_y
        if abs(sl["y"]) + half_w > inner_half + TOL:
            bad.append(sl)
        if sl["x_front"] + sl["max_d"] > s.depth - s.back_t + TOL:
            bad.append(sl)
        if sl["z"] + sl["max_h"] > s.height + TOL:
            bad.append(sl)
    c.true("모든 슬롯이 선반 내부", not bad, f"벗어난 슬롯 {len(bad)}개")

    # 상품이 실제로 들어갈 만한 크기인가 ---------------------
    # YCB 최대 물체(크래커 박스류)가 대략 16×6×21 cm 다.
    c.true(
        "슬롯 폭 ≥ 10 cm",
        s.slot_width() >= 0.10,
        f"{s.slot_width() * 100:.1f} cm",
    )
    # 단 여유는 "상품이 들어가는가"로 따진다. 대표 상품 높이로 검사한다.
    # (YCB 기준 대략: 수프 캔 10cm, 크래커 박스 21cm, 500ml 병 23cm)
    REF_ITEM_H = {"수프캔": 0.10, "크래커박스": 0.21, "500ml병": 0.23}
    cl = s.level_clearances()
    c.true(
        "모든 단에 최소 상품(10cm)이 들어감",
        min(cl) >= REF_ITEM_H["수프캔"] - TOL,
        "[" + ", ".join(f"{x * 100:.0f}" for x in cl) + "] cm",
    )
    for name, h in REF_ITEM_H.items():
        fits = [i for i, x in enumerate(cl) if x >= h - TOL]
        c.true(
            f"{name}({h * 100:.0f}cm) 가 들어가는 단",
            len(fits) == len(cl),
            f"{len(fits)}/{len(cl)}단"
            + ("" if len(fits) == len(cl) else f"  ← 단 {sorted(set(range(len(cl))) - set(fits))} 불가"),
        )

    return 1 if c.report() else 0


if __name__ == "__main__":
    sys.exit(main())
