"""상품 배치 — 진열대 슬롯에 YCB 에셋을 채운다.

    python -m scene.store --out out/store.usda                     # 먼저 매장
    python -m scene.stock --store out/store.usda --out out/store_stocked.usda
    python -m scene.stock --seed 7 --fill 0.7                      # 시나리오: 빈 자리 30 %

store.usda 는 건드리지 않는다. 새 레이어가 store.usda 를 서브레이어로 깔고,
각 진열대 프림 아래 `Stock/Item_NN` 을 추가한다. 상품은 진열대 로컬 좌표에
놓이므로 진열대가 어디에 어떻게 돌아가 있든 따라간다.

배치 규칙 (실제 진열 관행):
  - 같은 상품은 옆으로 이어 붙인다 (페이싱). 한 단의 슬롯 몇 개가 한 상품 블록
  - 앞에서 뒤로 여러 개 세운다 (depth). 슬롯 깊이에 들어가는 만큼, --depth 상한
  - 통로마다 품목군이 다르다 (플래노그램). 실측 전에는 PLANOGRAM 의 잠정 배정
  - seed 를 주면 상품 선택·빈 자리·yaw 가 흔들린다. seed 없으면 결정적 (트윈)

에셋은 tools/ycb_catalog.py 가 만든 assets/ycb/usd/*.usd 를 참조(reference)
한다. 인스턴싱을 켜서 같은 상품 수천 개가 메시 하나를 공유한다.
"""

from __future__ import annotations

import argparse
import json
import random
from pathlib import Path

from pxr import Gf, Sdf, Usd, UsdGeom

from scene.constants import SHELF, STORE, ShelfSpec
from scene.shelf import slot_positions
from scene.store import placements

CATALOG = Path("assets/ycb/catalog.json")
FACING_GAP = 0.01        # [설계] 앞뒤 상품 사이 틈
SIDE_MARGIN = 0.005      # [설계] 슬롯 폭 안에서 좌우 여유

# 부통로 → 품목군. 실제 매장은 통로마다 카테고리가 정해져 있다. [잠정]
# 벽면 진열대는 그 통로의 품목군을 따르고, 엔드캡은 행사 상품(식품)이다.
PLANOGRAM: dict[int, list[str]] = {
    0: ["식품"], 1: ["식품"], 2: ["식품"], 3: ["식품"],
    4: ["생활용품"], 5: ["생활용품", "주방"], 6: ["주방"], 7: ["문구", "완구", "스포츠"],
}
BLOCK_SLOTS = 2          # [설계] 같은 상품이 옆으로 이어지는 슬롯 수


def load_catalog(path: Path = CATALOG) -> list[dict]:
    rows = json.loads(path.read_text())
    return [r for r in rows if r.get("category")]


def orient(item: dict, slot: dict) -> tuple[float, float, float, float] | None:
    """상품을 슬롯에 넣을 수 있는 방향을 고른다. (yaw_deg, depth, width, height) 또는 None.

    상품 로컬 x 가 선반 깊이(X) 방향으로 가는 것이 yaw 0. 90° 돌리면 x↔y.
    폭이 남는 쪽(슬롯 폭에 여유가 큰 쪽)을 택한다.
    """
    dx, dy, dz = item["dims"]
    if dz > slot["max_h"]:
        return None
    options = []
    for yaw, d, w in ((0.0, dx, dy), (90.0, dy, dx)):
        if w + 2 * SIDE_MARGIN <= slot["max_w"] and d <= slot["max_d"]:
            options.append((slot["max_w"] - w, yaw, d, w))
    if not options:
        return None
    _, yaw, d, w = max(options)
    return yaw, d, w, dz


def plan(
    catalog: list[dict],
    *,
    seed: int | None = None,
    fill: float = 1.0,
    depth: int = 3,
    aisles: set[int] | None = None,
) -> list[dict]:
    """배치 계획. 각 항목: unit(진열대 배치 dict), slot, product, yaw, x, y, z, k(앞에서부터 몇 번째)."""
    rng = random.Random(seed)
    by_cat: dict[str, list[dict]] = {}
    for r in catalog:
        by_cat.setdefault(r["category"], []).append(r)

    items: list[dict] = []
    for unit in placements(STORE, SHELF):
        aisle = unit["aisle"] if unit["aisle"] is not None else 0
        if aisles is not None and unit["aisle"] is not None and aisle not in aisles:
            continue
        cats = ["식품"] if unit["kind"] == "endcap" else PLANOGRAM.get(aisle, ["식품"])
        pool = [r for c in cats for r in by_cat.get(c, [])]
        if not pool:
            continue
        spec: ShelfSpec = unit["spec"]
        slots = slot_positions(spec)

        # 단마다 상품 블록을 이어 붙인다. seed 없으면 카탈로그 순서, 있으면 섞는다.
        order = pool[:] if seed is None else rng.sample(pool, len(pool))
        cursor = 0
        for li in range(spec.n_levels):
            level_slots = [s for s in slots if s["level"] == li]
            si = 0
            while si < len(level_slots):
                product = order[cursor % len(order)]
                cursor += 1
                for s in level_slots[si : si + BLOCK_SLOTS]:
                    if seed is not None and rng.random() > fill:
                        continue
                    o = orient(product, s)
                    if o is None:
                        continue
                    yaw, d, w, h = o
                    n = min(depth, int((s["max_d"] + FACING_GAP) // (d + FACING_GAP)))
                    for k in range(n):
                        items.append(
                            {
                                "unit": unit,
                                "slot": s,
                                "product": product,
                                "yaw": yaw,
                                "x": s["x_front"] + d / 2 + k * (d + FACING_GAP),
                                "y": s["y"],
                                "z": s["z"],
                                "k": k,
                                "dims": (d, w, h),
                            }
                        )
                si += BLOCK_SLOTS
    return items


def build_stock(out: Path, store_usd: Path, items: list[dict], *, asset_root: Path = Path("assets/ycb/usd")) -> Usd.Stage:
    """store.usda 를 서브레이어로 깔고 그 위에 상품 프림을 얹는 새 레이어를 만든다."""
    out.unlink(missing_ok=True)
    stage = Usd.Stage.CreateNew(str(out))
    store_rel = Path(__import__("os").path.relpath(store_usd.resolve(), out.parent.resolve()))
    stage.GetRootLayer().subLayerPaths = [str(store_rel)]
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    stage.SetDefaultPrim(stage.GetPrimAtPath("/World"))

    counters: dict[str, int] = {}
    for it in items:
        unit_path = it["unit"]["path"]
        n = counters.get(unit_path, 0)
        counters[unit_path] = n + 1
        path = f"{unit_path}/Stock/Item_{n:03d}"
        xf = UsdGeom.Xform.Define(stage, path)
        prim = xf.GetPrim()
        asset = asset_root.resolve() / f"{it['product']['name']}.usd"
        asset_rel = Path(__import__("os").path.relpath(asset, out.parent.resolve()))
        prim.GetReferences().AddReference(str(asset_rel))
        prim.SetInstanceable(True)
        xf.AddTranslateOp().Set(Gf.Vec3d(it["x"], it["y"], it["z"]))
        # xformOp 순서 [translate, rotateZ, rotateY] = 점에 rotateY(넘어짐) → rotateZ(yaw) → 이동
        if it["yaw"]:
            xf.AddRotateZOp().Set(it["yaw"])
        if it.get("pitch"):
            xf.AddRotateYOp().Set(it["pitch"])
        # 검증·로봇 작업 지시용 메타데이터 (정답 라벨)
        prim.CreateAttribute("stock:product", Sdf.ValueTypeNames.String).Set(it["product"]["name"])
        prim.CreateAttribute("stock:level", Sdf.ValueTypeNames.Int).Set(it["slot"]["level"])
        prim.CreateAttribute("stock:slot", Sdf.ValueTypeNames.Int).Set(it["slot"]["index"])
        prim.CreateAttribute("stock:facing", Sdf.ValueTypeNames.Int).Set(it["k"])
        prim.CreateAttribute("stock:state", Sdf.ValueTypeNames.String).Set(it.get("state", "upright"))
        prim.CreateAttribute("stock:misplaced", Sdf.ValueTypeNames.Bool).Set(bool(it.get("misplaced", False)))
    stage.GetRootLayer().Save()
    return stage


def describe(items: list[dict]) -> str:
    units = {it["unit"]["path"] for it in items}
    products: dict[str, int] = {}
    for it in items:
        products[it["product"]["name"]] = products.get(it["product"]["name"], 0) + 1
    top = sorted(products.items(), key=lambda kv: -kv[1])[:6]
    lines = [
        f"  상품 {len(items)}개  /  진열대 {len(units)}대  /  종류 {len(products)}",
        "  많이 놓인 것: " + ", ".join(f"{n.split('_', 1)[1]} {c}" for n, c in top),
    ]
    return "\n".join(lines)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--store", default="out/store.usda")
    ap.add_argument("--out", default="out/store_stocked.usda")
    ap.add_argument("--catalog", default=str(CATALOG))
    ap.add_argument("--seed", type=int, default=None, help="없으면 결정적(트윈), 있으면 시나리오")
    ap.add_argument("--fill", type=float, default=1.0, help="seed 모드에서 슬롯이 채워질 확률")
    ap.add_argument("--depth", type=int, default=3, help="앞뒤로 세우는 최대 개수")
    ap.add_argument("--aisles", default=None, help="채울 부통로만 (예: 0,1). 기본 전부")
    args = ap.parse_args()

    catalog = load_catalog(Path(args.catalog))
    aisles = {int(a) for a in args.aisles.split(",")} if args.aisles else None
    items = plan(catalog, seed=args.seed, fill=args.fill, depth=args.depth, aisles=aisles)

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    build_stock(out, Path(args.store), items, asset_root=Path(args.catalog).parent / "usd")
    print(f"저장: {out}  (서브레이어 {args.store})")
    print(describe(items))


if __name__ == "__main__":
    main()
