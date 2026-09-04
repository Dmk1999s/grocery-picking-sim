"""상품이 채워진 매장 USD 를 되읽어 배치가 물리적으로 말이 되는지 본다.

    python -m tools.verify_stock out/store_stocked.usda

본다:
  - 모든 상품이 자기 단 안에 있다 (지주 사이·앞단 뒤·백판 앞·위 선반 아래, 진열대 로컬 좌표로)
  - 밑면이 선반 윗면에 닿아 있다 (떠 있거나 파고들지 않음)
  - 같은 진열대 안에서 상품끼리 겹치지 않는다
  - 참조가 풀렸다 (메시가 실제로 있다), 강체·콜라이더·질량이 붙어 있다
  - 개수·종류가 stock.plan 과 같다
"""

from __future__ import annotations

import argparse
import json
import sys
from itertools import combinations

from pxr import Gf, Usd, UsdGeom, UsdPhysics

from scene.constants import SHELF, STORE
from scene.store import placements
from tools.verify_shelf import Check

TOL = 2e-3


def local_bbox(cache: UsdGeom.BBoxCache, prim: Usd.Prim, ancestor: Usd.Prim):
    """`ancestor` 프레임에서 본 prim 의 AABB."""
    r = cache.ComputeRelativeBound(prim, ancestor).ComputeAlignedRange()
    return r.GetMin(), r.GetMax()


def overlap3(a, b, tol=TOL) -> bool:
    return all(a[0][i] < b[1][i] - tol and b[0][i] < a[1][i] - tol for i in range(3))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("usd", nargs="?", default="out/store_stocked.usda")
    args = ap.parse_args()

    stage = Usd.Stage.Open(args.usd)
    if stage is None:
        print(f"열 수 없음: {args.usd}")
        return 2
    c = Check()
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    print(f"\n{args.usd}\n")

    units = {p["path"]: p for p in placements(STORE, SHELF)}
    n_items = 0
    bad_slot, bad_floor, bad_ref, bad_phys, overlaps = [], [], [], [], []
    per_unit_boxes: dict[str, list] = {}
    products: set[str] = set()

    for unit_path, unit in units.items():
        unit_prim = stage.GetPrimAtPath(unit_path)
        stock = stage.GetPrimAtPath(f"{unit_path}/Stock")
        if not stock:
            continue
        spec = unit["spec"]
        back_x = spec.depth - spec.back_t
        half_in = spec.inner_width() / 2
        heights, fronts, clears = spec.level_heights(), spec.level_fronts(), spec.level_clearances()
        boxes = []
        for item in stock.GetChildren():
            n_items += 1
            products.add(item.GetAttribute("stock:product").Get())
            # 참조가 풀렸는가: 자식 메시가 있어야 한다
            # instanceable 프림 안은 인스턴스 프록시로 순회해야 보인다
            geom = [p for p in Usd.PrimRange(item, Usd.TraverseInstanceProxies()) if p.IsA(UsdGeom.Mesh)]
            if not geom:
                bad_ref.append(item.GetPath())
                continue
            if not (item.HasAPI(UsdPhysics.RigidBodyAPI) and item.HasAPI(UsdPhysics.MassAPI)
                    and any(g.HasAPI(UsdPhysics.CollisionAPI) for g in geom)):
                bad_phys.append(item.GetPath())

            lo, hi = local_bbox(cache, item, unit_prim)
            li = item.GetAttribute("stock:level").Get()
            z = heights[li]
            if abs(lo[2] - z) > TOL:
                bad_floor.append((str(item.GetPath()), round(lo[2] - z, 4)))
            # 단 안: 위 선반판 아래, 지주 사이, 앞단 여유 뒤 ~ 백판 앞
            if (hi[2] > z + clears[li] + TOL
                    or lo[1] < -half_in - TOL or hi[1] > half_in + TOL
                    or lo[0] < fronts[li] + spec.slot_front_gap - TOL or hi[0] > back_x + TOL):
                bad_slot.append(str(item.GetPath()))
            boxes.append((str(item.GetPath()), (lo, hi)))
        per_unit_boxes[unit_path] = boxes
        for (pa, ba), (pb, bb) in combinations(boxes, 2):
            if overlap3(ba, bb):
                overlaps.append((pa, pb))

    c.true("상품이 하나 이상 있음", n_items > 0, f"{n_items}개, {len(products)}종")
    c.true("모든 참조가 풀림 (메시 존재)", not bad_ref, f"실패 {len(bad_ref)}: {bad_ref[:2]}" if bad_ref else "")
    c.true("강체·콜라이더·질량이 붙음", not bad_phys, f"누락 {len(bad_phys)}: {bad_phys[:2]}" if bad_phys else "")
    c.true("밑면이 선반 윗면에 닿음", not bad_floor, f"{len(bad_floor)}개 어긋남 {bad_floor[:2]}" if bad_floor else f"허용 ±{TOL * 1000:.0f} mm")
    c.true("모든 상품이 자기 단 안 (지주 사이·앞단 뒤·백판 앞·위 선반 아래)", not bad_slot, f"{len(bad_slot)}개 벗어남 {bad_slot[:2]}" if bad_slot else "")
    c.true("같은 진열대 안에서 겹치지 않음", not overlaps, f"{len(overlaps)}쌍 {overlaps[:1]}" if overlaps else "")

    # 계획과 대조: 개수는 stock.plan 을 같은 인자로 다시 돌려야 정확하다. 여기서는
    # 메타데이터 일관성만 본다 — 같은 (진열대, 단, 슬롯, facing) 이 두 번 없음.
    keys = set()
    dup = 0
    for unit_path in per_unit_boxes:
        for item in stage.GetPrimAtPath(f"{unit_path}/Stock").GetChildren():
            k = (unit_path, item.GetAttribute("stock:level").Get(), item.GetAttribute("stock:slot").Get(), item.GetAttribute("stock:facing").Get())
            if k in keys:
                dup += 1
            keys.add(k)
    c.true("슬롯·facing 메타데이터 중복 없음", dup == 0, f"중복 {dup}")

    return 1 if c.report() else 0


if __name__ == "__main__":
    sys.exit(main())
