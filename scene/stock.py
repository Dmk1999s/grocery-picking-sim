"""상품 배치 — 진열대 슬롯에 YCB 에셋을 채운다.

    python -m scene.store --out out/store.usda                     # 먼저 매장
    python -m scene.stock --store out/store.usda --out out/store_stocked.usda
    python -m scene.stock --seed 7 --fill 0.7                      # 시나리오: 빈 자리 30 %

store.usda 는 건드리지 않는다. 새 레이어가 store.usda 를 서브레이어로 깔고,
각 진열대 프림 아래 `Stock/Item_NN` 을 추가한다. 상품은 진열대 로컬 좌표에
놓이므로 진열대가 어디에 어떻게 돌아가 있든 따라간다.

배치 규칙 (실제 진열 관행 — 빼곡하게):
  - 같은 상품은 옆으로 2~5 페이싱 붙여 세운다. 옆 상품과 0.5~1.5 cm. 라벨(넓은 면)이 통로를 본다
  - 앞에서 뒤로 선반 끝까지 세운다 (최대 6). 앞뒤 틈 1 cm
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

from pxr import Gf, Sdf, Usd, UsdGeom, UsdSemantics

from scene.constants import SHELF, STORE, ShelfSpec
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


def load_catalog(path: Path = CATALOG) -> list[dict]:
    rows = json.loads(path.read_text())
    return [r for r in rows if r.get("category")]


def orient(item: dict, slot: dict) -> tuple[float, float, float, float] | None:
    """상품을 자리에 넣을 수 있는 방향을 고른다. (yaw_deg, depth, width, height) 또는 None.

    상품 로컬 x 가 선반 깊이(X) 방향으로 가는 것이 yaw 0. 90° 돌리면 x↔y.
    실제 진열처럼 **넓은 면(라벨)이 통로를 보게** — 통로 방향 폭이 큰 쪽을 택하되 깊이가 들어가야 한다.
    """
    dx, dy, dz = item["dims"]
    if dz > slot["max_h"]:
        return None
    options = []
    for yaw, d, w in ((0.0, dx, dy), (90.0, dy, dx)):
        if w + 2 * SIDE_MARGIN <= slot["max_w"] and d <= slot["max_d"]:
            options.append((w, yaw, d, w))
    if not options:
        return None
    _, yaw, d, w = max(options)
    return yaw, d, w, dz


# 빼곡한 진열 [설계] — 실제 마트: 같은 상품이 여러 페이싱 붙어 서고, 옆 상품과 거의 붙고, 뒤로는 선반 끝까지
FACINGS = (2, 5)         # 같은 상품이 옆으로 이어지는 페이싱 수 (seed 없으면 3)
SIDE_GAP = (0.005, 0.015)   # 옆 상품과의 틈 (seed 없으면 0.01)
EDGE_GAP = 0.01          # 지주 옆 여유
DEPTH_MAX = 6            # 앞뒤 최대 개수 (선반 깊이가 허용하는 만큼)


def plan(
    catalog: list[dict],
    *,
    seed: int | None = None,
    fill: float = 1.0,
    depth: int = DEPTH_MAX,
    aisles: set[int] | None = None,
) -> list[dict]:
    """배치 계획. 각 항목: unit, slot(열 자리), product, yaw, x, y, z, k(앞에서부터 몇 번째), dims.

    단마다 왼쪽 지주부터 오른쪽으로 상품 블록을 이어 붙인다. 블록 = 같은 상품 FACINGS 개 열(column),
    열마다 뒤로 depth 개. 열의 '자리'(slot dict)는 시나리오 흔들림·검증이 쓰는 폭·깊이·높이 상한이다.
    fill < 1 이면 열 단위로 빈 자리가 난다 (실제 매장의 팔린 자리).
    """
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
        inner_w = spec.inner_width()
        back_x = spec.depth - spec.back_t
        fronts, clear = spec.level_fronts(), spec.level_clearances()
        order = pool[:] if seed is None else rng.sample(pool, len(pool))
        cursor = 0
        for li, z in enumerate(spec.level_heights()):
            x_front = fronts[li] + spec.slot_front_gap
            max_d = back_x - x_front
            y = -inner_w / 2 + EDGE_GAP
            col = 0
            guard = 0
            while y < inner_w / 2 - EDGE_GAP - 0.03 and guard < 200:
                guard += 1
                product = order[cursor % len(order)]
                cursor += 1
                probe = {"max_w": inner_w / 2 - EDGE_GAP - y, "max_d": max_d, "max_h": clear[li]}
                o = orient(product, probe)
                if o is None:
                    continue
                yaw, d, w, h = o
                n_face = 3 if seed is None else rng.randint(*FACINGS)
                gap = 0.01 if seed is None else rng.uniform(*SIDE_GAP)
                for f in range(n_face):
                    if y + w > inner_w / 2 - EDGE_GAP:
                        break
                    slot = {"level": li, "index": col, "x_front": x_front, "y": y + w / 2, "z": z,
                            "max_w": w + 2 * SIDE_MARGIN, "max_d": max_d, "max_h": clear[li]}
                    col += 1
                    y += w + gap
                    if seed is not None and rng.random() > fill:
                        continue
                    n = min(depth, int((max_d + FACING_GAP) // (d + FACING_GAP)))
                    for k in range(n):
                        items.append({"unit": unit, "slot": slot, "product": product, "yaw": yaw,
                                      "x": x_front + d / 2 + k * (d + FACING_GAP), "y": slot["y"], "z": z, "k": k, "dims": (d, w, h)})
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
        prim.CreateAttribute("stock:yaw", Sdf.ValueTypeNames.Float).Set(float(it["yaw"]))
        prim.CreateAttribute("stock:state", Sdf.ValueTypeNames.String).Set(it.get("state", "upright"))
        prim.CreateAttribute("stock:misplaced", Sdf.ValueTypeNames.Bool).Set(bool(it.get("misplaced", False)))
        # 시맨틱 라벨 (Isaac Replicator 가 인스턴스 분할·2D 박스에 쓴다): class = 상품명
        UsdSemantics.LabelsAPI.Apply(prim, "class").CreateLabelsAttr([it["product"]["name"]])
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
    ap.add_argument("--depth", type=int, default=DEPTH_MAX, help="앞뒤로 세우는 최대 개수")
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
