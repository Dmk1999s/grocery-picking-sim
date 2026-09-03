"""시나리오 JSON 이 USD 와 맞고, 로봇 작업이 실제로 가능한지 본다.

    python -m tools.verify_scenario out/scenario_007.json
    python -m tools.verify_scenario out/scenario_007.json --no-repro   # 재생성 비교 생략

본다:
  - 주문 품목이 가리키는 프림이 USD 에 있고 stock:* 라벨이 JSON 과 같다. 맨 앞(facing 0) 상품이다
  - 파지 대상 좌표가 USD 월드 AABB 중심과 같다 (constants 가 아니라 USD 로)
  - 정차 자세: 본체 + 안전여유가 진열대·기둥·벽 어느 것과도 겹치지 않고, 왼쪽이 진열대를 보며,
    본체 측면과 진열대 앞면 사이가 pick_standoff (레일 두께만큼 줄어든다)
  - 팔 도달: 마운트 → 상품 중심 ≤ arm_reach
  - 경유점: 구간이 축에 나란하고, 통로폭(본체+여유) 띠가 장애물과 겹치지 않으며, 길이 합이 JSON 과 같다
  - 흔들림 라벨: 오배치 상품은 그 진열대 플래노그램에 없는 품목군, 넘어짐은 rotateY -90, 개수가 통계와 같다
  - 조명: USD intensity == JSON, 꺼진 개수, 배율 범위
  - 재현성: 같은 seed 로 다시 생성한 JSON 이 완전히 같다
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import tempfile
from pathlib import Path

from pxr import Usd, UsdGeom, UsdLux

from scene.constants import ROBOT, SHELF, STORE
from scene.stock import PLANOGRAM, load_catalog
from scene.store import placements
from tools.verify_shelf import Check, world_bbox

TOL = 2e-3


def rect_overlap(a, b, tol=TOL) -> bool:
    return a[0] < b[2] - tol and b[0] < a[2] - tol and a[1] < b[3] - tol and b[1] < a[3] - tol


def footprint(x, y, yaw_deg, l, w, inflate=0.0):
    """본체 사각형의 AABB (x0, y0, x1, y1). yaw 가 90° 배수면 정확, 아니면 보수적."""
    a = math.radians(yaw_deg)
    hl, hw = l / 2 + inflate, w / 2 + inflate
    pts = [(x + dx * math.cos(a) - dy * math.sin(a), y + dx * math.sin(a) + dy * math.cos(a))
           for dx in (-hl, hl) for dy in (-hw, hw)]
    xs, ys = [p[0] for p in pts], [p[1] for p in pts]
    return min(xs), min(ys), max(xs), max(ys)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("json", nargs="?", default="out/scenario_007.json")
    ap.add_argument("--no-repro", action="store_true")
    args = ap.parse_args()

    sc = json.loads(Path(args.json).read_text())
    stage = Usd.Stage.Open(sc["usd"])
    if stage is None:
        print(f"열 수 없음: {sc['usd']}")
        return 2
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    c = Check()
    rb = ROBOT
    print(f"\n{args.json}  ↔  {sc['usd']}\n")

    # 장애물 (USD 월드 AABB, xy)
    units = {u["path"]: u for u in placements(STORE, SHELF)}
    obstacles: dict[str, tuple] = {}
    for path in units:
        lo, hi = world_bbox(cache, stage.GetPrimAtPath(path))
        obstacles[path] = (lo[0], lo[1], hi[0], hi[1])
    for scope in ("/World/Store/Columns", "/World/Store/Walls"):
        for prim in stage.GetPrimAtPath(scope).GetChildren():
            lo, hi = world_bbox(cache, stage.GetPrimAtPath(str(prim.GetPath())))
            obstacles[str(prim.GetPath())] = (lo[0], lo[1], hi[0], hi[1])

    # ── 상품 라벨 · 개수
    items = [p for p in stage.Traverse() if p.GetPath().name.startswith("Item_") and p.GetParent().GetPath().name == "Stock"]
    c.eq("상품 개수 = JSON stats.items", len(items), sc["stats"]["items"], tol=0)
    fallen = [p for p in items if p.GetAttribute("stock:state").Get() == "fallen"]
    mis_cols = {(str(p.GetParent().GetParent().GetPath()), p.GetAttribute("stock:level").Get(), p.GetAttribute("stock:slot").Get())
                for p in items if p.GetAttribute("stock:misplaced").Get()}
    c.eq("넘어진 상품 수 = stats.fallen", len(fallen), sc["stats"]["fallen"], tol=0)
    c.eq("오배치 슬롯열 수 = stats.misplaced", len(mis_cols), sc["stats"]["misplaced"], tol=0)
    bad_pitch = [p for p in fallen if not any(o.GetOpName() == "xformOp:rotateY" and abs(o.Get() + 90) < 1e-6
                                            for o in UsdGeom.Xformable(p).GetOrderedXformOps())]
    c.true("넘어진 상품은 rotateY -90 (앞으로 눕음)", not bad_pitch, f"{len(bad_pitch)}개 아님" if bad_pitch else f"{len(fallen)}개")
    cat_of = {r["name"]: r["category"] for r in load_catalog(Path(sc["store"]).parent.parent / "assets/ycb/catalog.json")}
    bad_mis = []
    for p in items:
        if not p.GetAttribute("stock:misplaced").Get():
            continue
        u = units[str(p.GetParent().GetParent().GetPath())]
        planned = ["식품"] if u["kind"] == "endcap" else PLANOGRAM.get(u["aisle"], ["식품"])
        if cat_of[p.GetAttribute("stock:product").Get()] in planned:
            bad_mis.append(str(p.GetPath()))
    c.true("오배치 상품은 그 통로 플래노그램에 없는 품목군", not bad_mis, f"{len(bad_mis)}개 위반" if bad_mis else "")

    # ── 조명
    bad_light, n_off = [], 0
    lo_s, hi_s = sc["params"]["light_scale"]
    for path, v in sc["lights"].items():
        got = UsdLux.RectLight(stage.GetPrimAtPath(path)).GetIntensityAttr().Get()
        if abs(got - v["intensity"]) > 0.5 or not (v["scale"] == 0 or lo_s - 1e-9 <= v["scale"] <= hi_s + 1e-9):
            bad_light.append(path)
        n_off += v["scale"] == 0
    c.true("조명 intensity = JSON, 배율이 범위 안", not bad_light, f"{len(sc['lights'])}개, 꺼짐 {n_off}")
    c.eq("꺼진 등 수 = stats.lights_off", n_off, sc["stats"]["lights_off"], tol=0)

    # ── 주문
    bad_prim, bad_front, bad_center, bad_reach, bad_face, bad_stop, bad_standoff = [], [], [], [], [], [], []
    bad_axis, bad_sweep, bad_len = [], [], []
    n_lines = n_seg = 0
    standoffs = []
    for o in sc["orders"]:
        for line in o["lines"]:
            n_lines += 1
            prim = stage.GetPrimAtPath(line["prim"])
            if not prim or prim.GetAttribute("stock:product").Get() != line["product"] \
                    or prim.GetAttribute("stock:level").Get() != line["level"] or prim.GetAttribute("stock:slot").Get() != line["slot"] \
                    or prim.GetAttribute("stock:state").Get() != line["state"] or bool(prim.GetAttribute("stock:misplaced").Get()) != line["misplaced"]:
                bad_prim.append(line["prim"])
                continue
            if prim.GetAttribute("stock:facing").Get() != 0:
                bad_front.append(line["prim"])
            lo, hi = world_bbox(cache, prim)
            center = [(lo[i] + hi[i]) / 2 for i in range(3)]
            if max(abs(a - b) for a, b in zip(center, line["item_center"])) > TOL:
                bad_center.append(line["prim"])
            st = line["stop"]
            reach = math.dist(st["arm_mount"], center)
            if reach > rb.arm_reach + TOL or abs(reach - st["reach_m"]) > TOL:
                bad_reach.append((line["prim"], round(reach, 3)))
            # 왼쪽(+Y 로봇) = approach_dir
            a = math.radians(st["yaw_deg"])
            left = (-math.sin(a), math.cos(a))
            if math.dist(left, line["approach_dir"][:2]) > 1e-6:
                bad_face.append(line["prim"])
            # 정차 footprint + 안전여유 vs 장애물
            fp = footprint(st["x"], st["y"], st["yaw_deg"], rb.base_l, rb.base_w, rb.safety_margin)
            hits = [k for k, ob in obstacles.items() if rect_overlap(fp, ob)]
            if hits:
                bad_stop.append((line["prim"], hits[:1]))
            # 본체 측면 ↔ 대상 진열대 앞면 간격 (레일이 튀어나온 만큼 줄어든다)
            fp0 = footprint(st["x"], st["y"], st["yaw_deg"], rb.base_l, rb.base_w)
            ob = obstacles[line["unit"]]
            nx, ny = line["approach_dir"][:2]
            if nx:
                gap = (ob[0] - fp0[2]) if nx > 0 else (fp0[0] - ob[2])
            else:
                gap = (ob[1] - fp0[3]) if ny > 0 else (fp0[1] - ob[3])
            standoffs.append(gap)
            if not (rb.pick_standoff - SHELF.rail_t - TOL <= gap <= rb.pick_standoff + TOL):
                bad_standoff.append((line["prim"], round(gap, 3)))
        # 경로
        wp = o["route"]["waypoints"]
        total = 0.0
        half = rb.corridor_width() / 2
        for a_, b_ in zip(wp, wp[1:]):
            n_seg += 1
            dx, dy = b_["x"] - a_["x"], b_["y"] - a_["y"]
            total += math.hypot(dx, dy)
            if abs(dx) > TOL and abs(dy) > TOL:
                bad_axis.append((o["id"], (a_["x"], a_["y"]), (b_["x"], b_["y"])))
                continue
            band = (min(a_["x"], b_["x"]) - half, min(a_["y"], b_["y"]) - half, max(a_["x"], b_["x"]) + half, max(a_["y"], b_["y"]) + half)
            hits = [k for k, ob in obstacles.items() if rect_overlap(band, ob)]
            if hits:
                bad_sweep.append((o["id"], (a_["x"], a_["y"]), (b_["x"], b_["y"]), hits[:1]))
        if abs(total - o["route"]["length_m"]) > 5e-3:
            bad_len.append((o["id"], round(total, 3), o["route"]["length_m"]))
        picks = [w for w in wp if w["kind"] == "pick"]
        if len(picks) != len(o["lines"]) or wp[0]["kind"] != "dock" or wp[-1]["kind"] != "dock":
            bad_len.append((o["id"], "구조"))

    c.true("주문 품목 프림이 있고 stock:* 라벨이 JSON 과 같음", not bad_prim, f"{n_lines}품목" if not bad_prim else f"{bad_prim[:2]}")
    c.true("주문 품목은 맨 앞 상품 (facing 0)", not bad_front, f"{bad_front[:2]}" if bad_front else "")
    c.true("파지 대상 좌표 = USD 월드 AABB 중심", not bad_center, f"{bad_center[:2]}" if bad_center else f"허용 ±{TOL * 1000:.0f} mm")
    c.true("팔 도달: 마운트 → 상품 중심 ≤ arm_reach", not bad_reach, f"{bad_reach[:2]}" if bad_reach else f"reach {rb.arm_reach} m")
    c.true("정차 시 왼쪽(팔)이 진열대를 봄", not bad_face, f"{bad_face[:2]}" if bad_face else "")
    c.true("정차 본체+안전여유가 장애물과 안 겹침", not bad_stop, f"{bad_stop[:2]}" if bad_stop else f"여유 {rb.safety_margin} m, 장애물 {len(obstacles)}개")
    c.true("본체 측면 ↔ 진열대 앞면 = pick_standoff", not bad_standoff,
           f"{bad_standoff[:2]}" if bad_standoff else f"{min(standoffs):.3f}~{max(standoffs):.3f} m (레일 {SHELF.rail_t} m 제외)")
    c.true("경로 구간이 축에 나란함", not bad_axis, f"{bad_axis[:1]}" if bad_axis else f"{n_seg}구간")
    c.true("경로 띠(본체+여유)가 장애물과 안 겹침", not bad_sweep, f"{bad_sweep[:1]}" if bad_sweep else f"띠 폭 {rb.corridor_width():.2f} m")
    c.true("경로 길이·구조가 JSON 과 같음", not bad_len, f"{bad_len[:2]}" if bad_len else "")

    # ── 재현성
    if not args.no_repro:
        from scene.scenario import generate
        from scene.constants import ScenarioSpec
        with tempfile.TemporaryDirectory() as td:
            spec = ScenarioSpec(**{k: (tuple(v) if isinstance(v, list) else v) for k, v in sc["params"].items() if k in ScenarioSpec.__dataclass_fields__})
            again = generate(sc["seed"], out_usd=Path(td) / "again.usda", store_usd=Path(sc["store"]),
                             spec=spec, n_orders=sc["params"]["n_orders"], lines=sc["params"]["lines_per_order"], depth=sc["params"]["depth"])
        strip = lambda d: {k: v for k, v in d.items() if k not in ("usd",)}
        same = json.dumps(strip(again), sort_keys=True) == json.dumps(strip(sc), sort_keys=True)
        c.true("같은 seed 로 재생성하면 JSON 이 같음", same, f"seed {sc['seed']}")

    return 1 if c.report() else 0


if __name__ == "__main__":
    sys.exit(main())
