"""시나리오 생성 — seed 하나로 '어느 날의 매장'과 '피킹 주문'을 재현한다.

    python -m scene.scenario --seed 7                     # out/scenario_007.usda + .json
    python -m scene.scenario --seed 7 --orders 6 --lines 4

트윈(store_stocked.usda)은 플래노그램대로 꽉 찬 이상적인 매장이다. 실제 매장은
그렇지 않다 — 빈 자리, 살짝 돌아간 상품, 앞으로 넘어진 상품, 다른 통로 상품이
잘못 놓인 자리, 통로마다 다른 조명, 꺼진 등. 학습·평가에 쓰려면 이 흔들림을
재현 가능하게 만들어야 한다. seed 하나가 아래 전부를 결정한다:

  매장 상태  stock.plan(seed) 위에 → 빈 자리 · yaw 흔들림 · 넘어짐 · 오배치 · 조명
  피킹 주문  주문 N 건 × 품목 M 개. 각 품목은 매장 안 실제 프림 하나를 가리킨다
  로봇 작업  품목마다 정차 자세(x, y, yaw) 와 파지 대상(월드 좌표·진입 방향),
             주문마다 도크 → 정차들 → 도크 경유점과 경로 길이

출력:
  out/scenario_007.usda   store.usda 를 서브레이어로 깐 상품·조명 레이어 (stock.py 와 같은 구조)
  out/scenario_007.json   주문·정차·경유점·파지 대상 + 흔들림 통계. 로봇 쪽은 이것만 읽는다

정답(ground truth)은 USD 안 `stock:*` 속성이다. 잘못 놓인 상품도 stock:product 는
실제 상품 이름이라, "플래노그램은 A 라는데 실제로는 B" 를 검출기가 맞혀야 한다.

좌표는 전부 매장 프레임(m). 정차 자세의 yaw 는 +X 에서 +Y 쪽으로 도는 각(°).
로봇 프레임은 +X 전방, +Y 왼쪽. 팔은 왼쪽 마운트라 정차하면 왼쪽이 진열대를 본다.
"""

from __future__ import annotations

import argparse
import json
import math
import random
from pathlib import Path

import numpy as np
from pxr import Gf, Usd, UsdGeom, UsdLux

from scene.constants import ROBOT, SCENARIO, SHELF, STORE, RobotSpec, ScenarioSpec
from scene.shelf import slot_positions
from scene.stock import CATALOG, FACING_GAP, PLANOGRAM, SIDE_MARGIN, build_stock, load_catalog, orient
from scene.stock import plan as stock_plan
from scene.store import corridors, placements

FALLEN_PITCH = -90.0     # rotateY. 상품 윗면이 -X(통로) 쪽으로 눕는다 = 앞으로 넘어짐


# ─────────────────────────────────────────────────────────────
# 상품 메시 — 회전한 뒤의 실제 AABB 를 알아야 선반에 정확히 놓인다
# ─────────────────────────────────────────────────────────────
class MeshPoints:
    """에셋 USD 의 AABB 를 한 번 읽어 두고, 회전 후 AABB 를 돌려준다.

    정점 전체를 돌려 정확한 AABB 를 구하면 안 된다. USD BBoxCache 는 인스턴스
    프림의 바운드를 extent 상자의 8 꼭짓점을 회전해서 잡으므로, 검증기(verify_stock)
    와 같은 값을 내려면 여기서도 상자 꼭짓점을 돌려야 한다. yaw 흔들림에서는
    과대평가지만 그만큼 이웃과 더 띄우는 쪽이라 안전하고, 90° 넘어짐은 상자
    회전이 정확해서 밑면이 선반에 그대로 닿는다.
    """

    def __init__(self, asset_root: Path):
        self.root = asset_root
        self._box: dict[str, np.ndarray] = {}

    def corners(self, name: str) -> np.ndarray:
        if name not in self._box:
            stage = Usd.Stage.Open(str(self.root / f"{name}.usd"))
            cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
            r = cache.ComputeWorldBound(stage.GetDefaultPrim()).ComputeAlignedRange()
            lo, hi = np.asarray(r.GetMin()), np.asarray(r.GetMax())
            self._box[name] = np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
        return self._box[name]

    def aabb(self, name: str, yaw_deg: float, pitch_deg: float) -> tuple[np.ndarray, np.ndarray]:
        """xformOp [translate, rotateZ, rotateY] 규약대로 상자를 Ry → Rz 로 돌린 AABB (이동 전)."""
        b = math.radians(pitch_deg)
        ry = np.array([[math.cos(b), 0, math.sin(b)], [0, 1, 0], [-math.sin(b), 0, math.cos(b)]])
        a = math.radians(yaw_deg)
        rz = np.array([[math.cos(a), -math.sin(a), 0], [math.sin(a), math.cos(a), 0], [0, 0, 1]])
        q = self.corners(name) @ ry.T @ rz.T
        return q.min(axis=0), q.max(axis=0)


def place_column(
    mesh: MeshPoints, slot: dict, back_x: float, entries: list[tuple[dict, float, float]]
) -> list[dict] | None:
    """한 슬롯의 앞뒤 열을 (상품, yaw, pitch) 순서대로 놓는다. 안 들어가면 None.

    각 상품의 회전 후 AABB 를 슬롯 폭·높이와 대조하고, 앞에서 뒤로 이어 붙이며
    백판 앞까지 들어가는지 본다. 돌려주는 항목은 stock.plan 과 같은 꼴 + pitch.
    """
    out = []
    cursor = slot["x_front"]
    for k, (product, yaw, pitch) in enumerate(entries):
        lo, hi = mesh.aabb(product["name"], yaw, pitch)
        d, w, h = hi - lo
        if w + 2 * SIDE_MARGIN > slot["max_w"] or h > slot["max_h"]:
            return None
        if cursor + d > back_x:
            return None
        out.append(
            {
                "slot": slot,
                "product": product,
                "yaw": yaw,
                "pitch": pitch,
                "x": cursor - lo[0],
                "y": slot["y"] - (lo[1] + hi[1]) / 2,
                "z": slot["z"] - lo[2],
                "k": k,
                "dims": (float(d), float(w), float(h)),
            }
        )
        cursor += d + FACING_GAP
    return out


# ─────────────────────────────────────────────────────────────
# 1. 매장 상태 — stock.plan 위에 흔들림
# ─────────────────────────────────────────────────────────────
def perturb(
    items: list[dict], catalog: list[dict], mesh: MeshPoints, rng: random.Random, spec: ScenarioSpec
) -> tuple[list[dict], dict]:
    """슬롯(앞뒤 열) 단위로 오배치 → 넘어짐 → yaw 흔들림을 적용한다.

    셋은 서로 배타적이지 않다. 안 들어가는 경우(회전한 상품이 슬롯보다 크거나
    뒤 상품과 겹침)에는 그 흔들림만 포기하고 원래대로 둔다 — 시나리오가 물리적
    으로 말이 안 되는 것보다 흔들림이 조금 덜 들어가는 쪽이 낫다.
    """
    columns: dict[tuple, list[dict]] = {}
    for it in items:
        columns.setdefault((it["unit"]["path"], it["slot"]["level"], it["slot"]["index"]), []).append(it)

    stats = {"columns": len(columns), "misplaced": 0, "fallen": 0, "jittered": 0, "dropped_facings": 0}
    out: list[dict] = []
    for key, col in columns.items():
        col.sort(key=lambda it: it["k"])
        unit, slot = col[0]["unit"], col[0]["slot"]
        back_x = unit["spec"].depth - unit["spec"].back_t
        aisle = unit["aisle"] if unit["aisle"] is not None else 0
        planned_cats = ["식품"] if unit["kind"] == "endcap" else PLANOGRAM.get(aisle, ["식품"])

        product = col[0]["product"]
        misplaced = False
        if rng.random() < spec.p_misplaced:
            others = [r for r in catalog if r["category"] not in planned_cats and orient(r, slot) is not None]
            if others:
                product = rng.choice(others)
                misplaced = True
        base_yaw = orient(product, slot)[0] if misplaced else col[0]["yaw"]
        n = len(col)

        entries = [(product, base_yaw, 0.0) for _ in range(n)]
        fallen = False
        if rng.random() < spec.p_fallen:
            entries[0] = (product, base_yaw + rng.uniform(-spec.jitter_deg, spec.jitter_deg), FALLEN_PITCH)
            fallen = True
        jitter = rng.random() < spec.p_jitter
        if jitter:
            entries = [(p, y + (rng.uniform(-spec.jitter_deg, spec.jitter_deg) if pitch == 0 else 0.0), pitch)
                       for p, y, pitch in entries]

        if not (misplaced or fallen or jitter):
            out.extend(col)
            continue

        # 넘어진 상품은 깊이를 더 먹는다. 뒤에서부터 빼며 들어갈 때까지 시도
        placed = None
        trial = entries
        while trial and placed is None:
            placed = place_column(mesh, slot, back_x, trial)
            if placed is None and fallen:
                trial = trial[:-1]
            else:
                break
        if placed is None and fallen:      # 넘어짐 포기, 나머지 흔들림으로 재시도
            fallen = False
            trial = [(p, y, 0.0) for p, y, _ in entries]
            placed = place_column(mesh, slot, back_x, trial)
        if placed is None and jitter:      # yaw 흔들림도 포기
            jitter = False
            trial = [(product, base_yaw, 0.0) for _ in range(n)]
            placed = place_column(mesh, slot, back_x, trial)
        if placed is None:                 # 오배치도 포기 → 원본
            out.extend(col)
            continue

        stats["misplaced"] += misplaced
        stats["fallen"] += fallen
        stats["jittered"] += jitter
        stats["dropped_facings"] += n - len(placed)
        for it in placed:
            it["unit"] = unit
            it["misplaced"] = misplaced
            it["state"] = "fallen" if it["pitch"] else "upright"
            if not it["pitch"]:
                it.pop("pitch")
            out.append(it)
    return out, stats


def light_overrides(rng: random.Random, spec: ScenarioSpec) -> dict[str, dict]:
    """통로 조명마다 세기 배율. 꺼진 등은 0."""
    out = {}
    for i, cor in enumerate(corridors(STORE, SHELF)):
        scale = 0.0 if rng.random() < spec.p_light_off else rng.uniform(*spec.light_scale)
        out[f"/World/Store/Lights/Light_{i:02d}"] = {
            "corridor": cor["name"],
            "scale": round(scale, 3),
            "intensity": round(STORE.light_intensity * scale, 1),
        }
    return out


def apply_lights(stage: Usd.Stage, lights: dict[str, dict]) -> None:
    for path, v in lights.items():
        prim = stage.OverridePrim(path)
        UsdLux.RectLight(prim).CreateIntensityAttr().Set(v["intensity"])


# ─────────────────────────────────────────────────────────────
# 2. 로봇 정차 자세 · 도달 — 진열대 앞면 기준
# ─────────────────────────────────────────────────────────────
def unit_normal(unit: dict) -> tuple[float, float]:
    """진열대 안쪽(+X 로컬) 방향의 월드 단위벡터. 팔이 들어가는 방향."""
    a = math.radians(unit["rotate"])
    return (round(math.cos(a)), round(math.sin(a)))


def pick_pose(unit: dict, item_center, robot: RobotSpec = ROBOT) -> dict:
    """상품 하나 앞의 정차 자세와 팔 마운트 위치, 도달 거리.

    진열대 앞면에서 pick_standoff + 본체 반폭만큼 떨어져, 상품의 통로 방향
    좌표에 맞춰 선다. 왼쪽(+Y 로봇)이 진열대를 보도록 heading 을 정한다.
    """
    nx, ny = unit_normal(unit)                    # 통로 → 진열대 (안쪽)
    fx, fy, _ = unit["translate"]                 # 앞면 위의 한 점
    cx, cy, cz = item_center
    # 상품 중심을 앞면 위로 투영: 앞면 법선 성분만 앞면 값으로 바꾼다
    px = fx if nx else cx
    py = fy if ny else cy
    off = robot.pick_standoff + robot.base_w / 2
    left = (nx, ny)
    fwd = (left[1], -left[0])                     # heading = left 를 -90° 돌린 것
    # 팔 베이스가 본체 중심에서 (arm_base_dx 앞, arm_mount_dy 왼쪽) 에 있으므로, 어깨가 상품 앞에 오도록 본체를 뒤로 물린다
    rx = px - nx * off - fwd[0] * robot.arm_base_dx
    ry = py - ny * off - fwd[1] * robot.arm_base_dx
    yaw = math.degrees(math.atan2(fwd[1], fwd[0]))
    mount = (rx + fwd[0] * robot.arm_base_dx + nx * robot.arm_mount_dy,
             ry + fwd[1] * robot.arm_base_dx + ny * robot.arm_mount_dy, robot.arm_mount_z())
    reach = math.dist(mount, (cx, cy, cz))
    return {
        "x": round(rx, 4), "y": round(ry, 4), "yaw_deg": round(yaw, 1),
        "arm_mount": [round(v, 4) for v in mount],
        "reach_m": round(reach, 4),
        "reachable": reach <= robot.arm_reach,
    }


# ─────────────────────────────────────────────────────────────
# 3. 경로 — 통로 중심선 그래프 위 경유점
# ─────────────────────────────────────────────────────────────
class Router:
    """부통로 중심선(세로) 과 주통로 주행선(가로) 만 다닌다.

    주통로 주행선은 기둥이 없는 가장 넓은 띠의 가운데다 (앞 주통로에는 기둥이
    서 있어 폭 중심으로 가면 기둥에 걸린다). 두 자세 사이 경로는 후보(앞 주통로
    경유 / 뒤 주통로 경유 / 같은 통로 직행) 중 가장 짧은 것이다.
    """

    def __init__(self, store=STORE, shelf=SHELF, robot: RobotSpec = ROBOT):
        self.lx, self.ly = store.footprint(shelf)
        self.aisle_x = [sum(store.aisle_x_range(a, shelf)) / 2 for a in range(store.n_aisles)]
        self.aisle_xr = [store.aisle_x_range(a, shelf) for a in range(store.n_aisles)]
        self.main_y = [self._free_line(yr, store, shelf, robot) for yr in store.main_aisle_y_ranges(shelf)]

    @staticmethod
    def _free_line(yr, store, shelf, robot) -> float:
        half = store.column_size / 2
        blocks = sorted((y - half, y + half) for _, y in store.columns(shelf) if yr[0] < y < yr[1])
        edges = [yr[0]] + [e for b in blocks for e in b] + [yr[1]]
        bands = [(edges[i], edges[i + 1]) for i in range(0, len(edges), 2)]
        lo, hi = max(bands, key=lambda b: b[1] - b[0])
        return (lo + hi) / 2

    def aisle_of(self, x: float, y: float) -> int | None:
        for a, (x0, x1) in enumerate(self.aisle_xr):
            if x0 <= x <= x1 and self.main_y[0] < y < self.main_y[1]:
                return a
        return None

    def main_of(self, y: float) -> int:
        return 0 if abs(y - self.main_y[0]) < abs(y - self.main_y[1]) else 1

    def route(self, p: tuple[float, float], q: tuple[float, float]) -> list[tuple[float, float]]:
        """p → q 경유점 (p, q 포함). 통로 중심선 위로 올라가 그래프를 따라간다."""
        ap, aq = self.aisle_of(*p), self.aisle_of(*q)
        cands = []
        if ap is not None and aq is not None:
            if ap == aq:
                cands.append([p, (self.aisle_x[ap], p[1]), (self.aisle_x[ap], q[1]), q])
            for my in self.main_y:
                cands.append([p, (self.aisle_x[ap], p[1]), (self.aisle_x[ap], my), (self.aisle_x[aq], my), (self.aisle_x[aq], q[1]), q])
        elif ap is not None:                      # q 는 주통로
            my = self.main_y[self.main_of(q[1])]
            cands.append([p, (self.aisle_x[ap], p[1]), (self.aisle_x[ap], my), (q[0], my), q])
        elif aq is not None:                      # p 는 주통로
            my = self.main_y[self.main_of(p[1])]
            cands.append([p, (p[0], my), (self.aisle_x[aq], my), (self.aisle_x[aq], q[1]), q])
        else:
            mp, mq = self.main_of(p[1]), self.main_of(q[1])
            if mp == mq:
                cands.append([p, (p[0], self.main_y[mp]), (q[0], self.main_y[mq]), q])
            else:
                for ax in self.aisle_x:           # 어느 부통로로 건너갈지
                    cands.append([p, (p[0], self.main_y[mp]), (ax, self.main_y[mp]), (ax, self.main_y[mq]), (q[0], self.main_y[mq]), q])
        best = min(cands, key=self.length)
        return dedupe(best)

    @staticmethod
    def length(pts) -> float:
        return sum(math.dist(a, b) for a, b in zip(pts, pts[1:]))


def dedupe(pts):
    out = [pts[0]]
    for p in pts[1:]:
        if math.dist(p, out[-1]) > 1e-6:
            out.append(p)
    return out


def order_route(router: Router, dock: tuple[float, float], stops: list[dict]) -> tuple[list[dict], list[dict], float]:
    """도크 → 정차들 → 도크. 방문 순서는 경로 길이 기준 탐욕 최근접 (최적은 아니다).

    돌려주는 것: (방문 순서의 stops, 경유점 목록, 총 길이).
    """
    remaining = stops[:]
    cur = dock
    ordered, waypoints = [], [{"x": dock[0], "y": dock[1], "kind": "dock", "line": None}]
    total = 0.0
    while remaining:
        nxt = min(remaining, key=lambda s: router.length(router.route(cur, (s["stop"]["x"], s["stop"]["y"]))))
        remaining.remove(nxt)
        seg = router.route(cur, (nxt["stop"]["x"], nxt["stop"]["y"]))
        total += router.length(seg)
        waypoints += [{"x": round(x, 4), "y": round(y, 4), "kind": "via", "line": None} for x, y in seg[1:-1]]
        waypoints.append({"x": nxt["stop"]["x"], "y": nxt["stop"]["y"], "kind": "pick", "line": nxt["line"], "yaw_deg": nxt["stop"]["yaw_deg"]})
        ordered.append(nxt)
        cur = seg[-1]
    seg = router.route(cur, dock)
    total += router.length(seg)
    waypoints += [{"x": round(x, 4), "y": round(y, 4), "kind": "via", "line": None} for x, y in seg[1:-1]]
    waypoints.append({"x": dock[0], "y": dock[1], "kind": "dock", "line": None})
    # 경유점 heading = 다음 점 방향. 마지막은 직전 방향
    for i, w in enumerate(waypoints):
        if "yaw_deg" in w:
            continue
        a, b = (waypoints[i], waypoints[i + 1]) if i + 1 < len(waypoints) else (waypoints[i - 1], waypoints[i])
        w["yaw_deg"] = round(math.degrees(math.atan2(b["y"] - a["y"], b["x"] - a["x"])), 1)
    return ordered, waypoints, round(total, 3)


# ─────────────────────────────────────────────────────────────
# 4. 전체
# ─────────────────────────────────────────────────────────────
def generate(
    seed: int,
    *,
    out_usd: Path,
    store_usd: Path,
    catalog_path: Path = CATALOG,
    spec: ScenarioSpec = SCENARIO,
    n_orders: int | None = None,
    lines: int | None = None,
    depth: int = 3,
) -> dict:
    n_orders = spec.n_orders if n_orders is None else n_orders
    lines = spec.lines_per_order if lines is None else lines
    rng = random.Random(seed)
    catalog = load_catalog(catalog_path)
    mesh = MeshPoints(catalog_path.parent / "usd")

    # 매장 상태 — stock 의 seed 흐름과 독립인 rng 를 써서 fill 만 바뀌어도 나머지가 안 흔들린다
    items = stock_plan(catalog, seed=seed, fill=spec.fill, depth=depth)
    items, pstats = perturb(items, catalog, mesh, random.Random(seed * 7919 + 1), spec)
    lights = light_overrides(random.Random(seed * 7919 + 2), spec)

    out_usd.parent.mkdir(parents=True, exist_ok=True)
    stage = build_stock(out_usd, store_usd, items, asset_root=catalog_path.parent / "usd")
    apply_lights(stage, lights)
    stage.GetRootLayer().Save()

    # 파지 후보 = 맨 앞 상품. 월드 좌표는 constants 가 아니라 방금 쓴 USD 에서 되읽는다
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    units = {u["path"]: u for u in placements(STORE, SHELF)}
    candidates = []
    for unit_path in sorted({it["unit"]["path"] for it in items}):
        stock = stage.GetPrimAtPath(f"{unit_path}/Stock")
        for prim in stock.GetChildren():
            if prim.GetAttribute("stock:facing").Get() != 0:
                continue
            r = cache.ComputeWorldBound(prim).ComputeAlignedRange()
            lo, hi = r.GetMin(), r.GetMax()
            center = tuple((lo[i] + hi[i]) / 2 for i in range(3))
            unit = units[unit_path]
            pose = pick_pose(unit, center)
            # 파지: 손가락이 통로 방향으로 닫힌다 (앞 상품 뒤에는 다음 상품이 붙어 있어 깊이 방향으로는 못 잡는다).
            # 통로 방향 폭 = 진열대 로컬 y 폭. 진열대 회전이 90° 배수라 월드 AABB 에서 바로 읽는다
            along = 1 if unit_normal(unit)[0] else 0
            width_along = hi[along] - lo[along]
            pose["graspable"] = width_along <= ROBOT.graspable_width()
            pose["grasp_width_m"] = round(width_along, 4)
            candidates.append(
                {
                    "prim": str(prim.GetPath()),
                    "product": prim.GetAttribute("stock:product").Get(),
                    "unit": unit_path,
                    "kind": unit["kind"],
                    "aisle": unit["aisle"],
                    "side": unit["side"],
                    "level": prim.GetAttribute("stock:level").Get(),
                    "slot": prim.GetAttribute("stock:slot").Get(),
                    "state": prim.GetAttribute("stock:state").Get(),
                    "misplaced": bool(prim.GetAttribute("stock:misplaced").Get()),
                    "item_center": [round(v, 4) for v in center],
                    "item_aabb": [[round(v, 4) for v in lo], [round(v, 4) for v in hi]],
                    "approach_dir": list(unit_normal(unit)) + [0],
                    "stop": pose,
                }
            )
    reachable = [c for c in candidates if c["stop"]["reachable"]]
    pickable = [c for c in reachable if c["stop"]["graspable"]]
    by_product: dict[str, list[dict]] = {}
    for c in pickable:
        by_product.setdefault(c["product"], []).append(c)

    # 주문 — 도달 가능하고 그리퍼에 들어가는 상품 중에서. 한 주문 안에서는 상품이 겹치지 않는다
    router = Router()
    dock = (router.aisle_x[0], router.main_y[0])
    orng = random.Random(seed * 7919 + 3)
    orders = []
    for oi in range(n_orders):
        names = orng.sample(sorted(by_product), min(lines, len(by_product)))
        picked = []
        for li, name in enumerate(names):
            c = dict(orng.choice(by_product[name]))
            c["line"] = li
            picked.append(c)
        ordered, waypoints, length = order_route(router, dock, picked)
        orders.append(
            {
                "id": f"ORD_{oi:02d}",
                "lines": picked,
                "visit_order": [c["line"] for c in ordered],
                "route": {"waypoints": waypoints, "length_m": length},
            }
        )

    unreach_by_level: dict[int, int] = {}
    for c in candidates:
        if not c["stop"]["reachable"]:
            unreach_by_level[c["level"]] = unreach_by_level.get(c["level"], 0) + 1

    return {
        "seed": seed,
        "usd": str(out_usd),
        "store": str(store_usd),
        "params": {**spec.__dict__, "n_orders": n_orders, "lines_per_order": lines, "depth": depth},
        "robot": {**ROBOT.__dict__, "arm_side": "left"},
        "stats": {
            "items": len(items),
            "products": len({it["product"]["name"] for it in items}),
            **pstats,
            "front_items": len(candidates),
            "reachable_front_items": len(reachable),
            "graspable_front_items": sum(1 for c in candidates if c["stop"]["graspable"]),
            "pickable_front_items": len(pickable),
            "arm_mount_z": round(ROBOT.arm_mount_z(), 3),
            "unreachable_by_level": dict(sorted(unreach_by_level.items())),
            "lights_off": sum(1 for v in lights.values() if v["scale"] == 0),
        },
        "lights": lights,
        "dock": {"x": round(dock[0], 4), "y": round(dock[1], 4), "yaw_deg": 0.0},
        "main_lines_y": [round(v, 4) for v in router.main_y],
        "orders": orders,
    }


def describe(sc: dict) -> str:
    s = sc["stats"]
    lines = [
        f"  상품 {s['items']}개 / {s['products']}종   슬롯열 {s['columns']}  →  yaw 흔들림 {s['jittered']}  넘어짐 {s['fallen']}  오배치 {s['misplaced']}  (facing 탈락 {s['dropped_facings']})",
        f"  조명 {len(sc['lights'])}개 중 꺼짐 {s['lights_off']}",
        f"  맨 앞 상품 {s['front_items']}개 중 팔 도달 {s['reachable_front_items']}개 (어깨 {s['arm_mount_z']} m, 미도달 단별 {s['unreachable_by_level']}),"
        f" 그리퍼 폭 안 {s['graspable_front_items']}개, 둘 다 {s['pickable_front_items']}개",
    ]
    for o in sc["orders"]:
        names = ", ".join(l["product"].split("_", 1)[1] for l in o["lines"])
        where = ", ".join(f"{'EC' if l['aisle'] is None else 'A' + str(l['aisle'])}/{l['side']}/L{l['level']}" for l in o["lines"])
        lines.append(f"  {o['id']}  {len(o['lines'])}품목  경로 {o['route']['length_m']:.1f} m   [{names}]  @ {where}")
    return "\n".join(lines)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seed", type=int, required=True)
    ap.add_argument("--store", default="out/store.usda")
    ap.add_argument("--out", default=None, help="기본 out/scenario_<seed:03d>.usda (json 은 같은 이름)")
    ap.add_argument("--catalog", default=str(CATALOG))
    ap.add_argument("--orders", type=int, default=None)
    ap.add_argument("--lines", type=int, default=None)
    ap.add_argument("--fill", type=float, default=None)
    args = ap.parse_args()

    out_usd = Path(args.out) if args.out else Path(f"out/scenario_{args.seed:03d}.usda")
    spec = SCENARIO if args.fill is None else ScenarioSpec(**{**SCENARIO.__dict__, "fill": args.fill})
    sc = generate(args.seed, out_usd=out_usd, store_usd=Path(args.store), catalog_path=Path(args.catalog),
                  spec=spec, n_orders=args.orders, lines=args.lines)
    out_json = out_usd.with_suffix(".json")
    out_json.write_text(json.dumps(sc, ensure_ascii=False, indent=1))
    print(f"저장: {out_usd}  +  {out_json}")
    print(describe(sc))


if __name__ == "__main__":
    main()
