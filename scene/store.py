"""매장 생성기 — 순수 USD.

    python -m scene.store --out out/store.usda

constants.StoreSpec 의 그림대로 한 구역을 만든다:
  바닥(타일 텍스처) · 천장 · 벽 4면 · 기둥 · 통로별 라인 조명
  벽면 진열대 · 양면 곤돌라 · 엔드캡     ← 전부 shelf.build_shelf 로

배치 좌표는 placements() 가 constants 에서만 계산한다. verify_store 도
같은 함수를 써서 USD 와 대조하므로, 배치 규칙은 여기 한 곳에만 있다.

프림 이름은 로봇이 작업을 지칭하는 단위와 같다:
  /World/Shelves/Aisle_00/L/Unit_02   부통로 0 의 왼쪽(-X) 면, 세 번째 진열대
  /World/Shelves/EndCap/Gondola_00_Front  첫 양면 곤돌라의 앞 주통로쪽 엔드캡
"""

from __future__ import annotations

import argparse
import math
import struct
import zlib
from pathlib import Path

from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade

from scene.constants import ROBOT, SHELF, STORE, RobotSpec, ShelfSpec, StoreSpec
from scene.shelf import box, build_shelf, new_stage


# ─────────────────────────────────────────────────────────────
# 배치 계산 — 순수 함수
# ─────────────────────────────────────────────────────────────
def placements(store: StoreSpec = STORE, shelf: ShelfSpec = SHELF) -> list[dict]:
    """진열대 배치 목록. 각 항목: path, spec, translate, rotate, kind, aisle, side.

    회전 규약 (선반 로컬 +X = 안쪽):
      rotate   0 → 안쪽이 world +X  (부통로의 오른쪽 R 면, 통로를 -X 로 본다)
      rotate 180 → 안쪽이 world -X  (부통로의 왼쪽 L 면)
      rotate +90 → 안쪽이 world +Y  (앞 주통로를 보는 엔드캡)
      rotate -90 → 안쪽이 world -Y  (뒤 주통로를 보는 엔드캡)
    """
    lx, ly = store.footprint(shelf)
    wall_spec = store.wall_unit_spec(shelf)
    cap_spec = store.endcap_spec(shelf)
    out: list[dict] = []

    # 벽면 진열대: 주통로 사이 전체 길이를 최대한 채우고 가운데 정렬
    span_lo, span_hi = store.main_aisle_width, ly - store.main_aisle_width
    n_wall = int((span_hi - span_lo) / wall_spec.width + 1e-9)
    wall_y0 = span_lo + ((span_hi - span_lo) - n_wall * wall_spec.width) / 2

    run_y0, _ = store.run_y_range(shelf)

    for a in range(store.n_aisles):
        x_lo, x_hi = store.aisle_x_range(a, shelf)
        for side, x_face, rot in (("L", x_lo, 180.0), ("R", x_hi, 0.0)):
            is_wall = (side == "L" and a == 0) or (side == "R" and a == store.n_aisles - 1)
            spec = wall_spec if is_wall else shelf
            n = n_wall if is_wall else store.shelves_per_run
            y0 = wall_y0 if is_wall else run_y0
            for j in range(n):
                out.append(
                    {
                        "path": f"/World/Shelves/Aisle_{a:02d}/{side}/Unit_{j:02d}",
                        "kind": "wall" if is_wall else "gondola",
                        "aisle": a,
                        "side": side,
                        "index": j,
                        "spec": spec,
                        "translate": (x_face, y0 + spec.width * (j + 0.5), 0.0),
                        "rotate": rot,
                    }
                )

    # 엔드캡: 양면 곤돌라(부통로 사이) 열 양 끝
    for g in range(store.n_aisles - 1):
        xc = store.aisle_x_range(g, shelf)[1] + shelf.depth   # 곤돌라 중심선
        for tag, y_face, rot in (
            ("Front", store.main_aisle_width, 90.0),
            ("Back", ly - store.main_aisle_width, -90.0),
        ):
            out.append(
                {
                    "path": f"/World/Shelves/EndCap/Gondola_{g:02d}_{tag}",
                    "kind": "endcap",
                    "aisle": None,
                    "side": tag,
                    "index": g,
                    "spec": cap_spec,
                    "translate": (xc, y_face, 0.0),
                    "rotate": rot,
                }
            )
    return out


def corridors(store: StoreSpec = STORE, shelf: ShelfSpec = SHELF) -> list[dict]:
    """AMR 이 다닐 구간. name, (x_lo, x_hi), (y_lo, y_hi), axis(진행 방향)."""
    lx, ly = store.footprint(shelf)
    out = []
    y_lo, y_hi = store.run_y_range(shelf)
    cap_d = store.endcap_spec(shelf).depth
    for a in range(store.n_aisles):
        out.append(
            {
                "name": f"부통로 {a}",
                "x": store.aisle_x_range(a, shelf),
                "y": (y_lo - cap_d, y_hi + cap_d),   # 엔드캡 옆까지 통로다
                "axis": "y",
            }
        )
    for tag, yr in zip(("앞", "뒤"), store.main_aisle_y_ranges(shelf)):
        out.append({"name": f"{tag} 주통로", "x": (0.0, lx), "y": yr, "axis": "x"})
    return out


# ─────────────────────────────────────────────────────────────
# 바닥 타일 텍스처 — 실측의 자(尺)를 화면에서도 셀 수 있게
# ─────────────────────────────────────────────────────────────
def write_tile_png(path: Path, px: int = 256, grout: int = 6) -> None:
    """타일 한 장 = 이미지 한 장. 가장자리에 줄눈. 외부 라이브러리 없이 쓴다."""
    tile = (214, 210, 202)
    seam = (120, 116, 110)
    rows = bytearray()
    for y in range(px):
        rows.append(0)  # filter: none
        for x in range(px):
            edge = x < grout or y < grout or x >= px - grout or y >= px - grout
            rows.extend(seam if edge else tile)

    def chunk(tag: bytes, data: bytes) -> bytes:
        return (
            struct.pack(">I", len(data))
            + tag
            + data
            + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)
        )

    png = b"\x89PNG\r\n\x1a\n"
    png += chunk(b"IHDR", struct.pack(">IIBBBBB", px, px, 8, 2, 0, 0, 0))
    png += chunk(b"IDAT", zlib.compress(bytes(rows), 9))
    png += chunk(b"IEND", b"")
    path.write_bytes(png)


def _floor(stage: Usd.Stage, path: str, lx: float, ly: float, tile: float, tex_rel: str) -> None:
    """바닥 = st 를 가진 사각 메시 + 타일 텍스처. 콜라이더."""
    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr([Gf.Vec3f(0, 0, 0), Gf.Vec3f(lx, 0, 0), Gf.Vec3f(lx, ly, 0), Gf.Vec3f(0, ly, 0)])
    mesh.CreateFaceVertexCountsAttr([4])
    mesh.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    mesh.CreateNormalsAttr([Gf.Vec3f(0, 0, 1)] * 4)
    mesh.CreateExtentAttr([Gf.Vec3f(0, 0, 0), Gf.Vec3f(lx, ly, 0)])
    pv = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar(
        "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex
    )
    # st 를 타일 단위로 두면 텍스처 한 장 = 타일 한 장이 된다
    pv.Set([Gf.Vec2f(0, 0), Gf.Vec2f(lx / tile, 0), Gf.Vec2f(lx / tile, ly / tile), Gf.Vec2f(0, ly / tile)])
    UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())

    mat = UsdShade.Material.Define(stage, "/World/Store/Looks/FloorTile")
    surf = UsdShade.Shader.Define(stage, "/World/Store/Looks/FloorTile/Surface")
    surf.CreateIdAttr("UsdPreviewSurface")
    surf.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.35)
    reader = UsdShade.Shader.Define(stage, "/World/Store/Looks/FloorTile/StReader")
    reader.CreateIdAttr("UsdPrimvarReader_float2")
    reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
    tex = UsdShade.Shader.Define(stage, "/World/Store/Looks/FloorTile/Texture")
    tex.CreateIdAttr("UsdUVTexture")
    tex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(tex_rel)
    tex.CreateInput("wrapS", Sdf.ValueTypeNames.Token).Set("repeat")
    tex.CreateInput("wrapT", Sdf.ValueTypeNames.Token).Set("repeat")
    tex.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(
        reader.CreateOutput("result", Sdf.ValueTypeNames.Float2)
    )
    surf.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(
        tex.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)
    )
    mat.CreateSurfaceOutput().ConnectToSource(surf.CreateOutput("surface", Sdf.ValueTypeNames.Token))
    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(mat)


# ─────────────────────────────────────────────────────────────
# 매장 생성
# ─────────────────────────────────────────────────────────────
def build_store(
    stage: Usd.Stage,
    store: StoreSpec = STORE,
    shelf: ShelfSpec = SHELF,
    *,
    tile_tex_rel: str = "./floor_tile.png",
) -> Usd.Prim:
    lx, ly = store.footprint(shelf)
    t, h = store.wall_t, store.ceiling_h

    root = UsdGeom.Xform.Define(stage, "/World/Store")
    UsdGeom.Scope.Define(stage, "/World/Store/Looks")
    _floor(stage, "/World/Store/Floor", lx, ly, store.tile, tile_tex_rel)

    # 천장: 조명 반사면. 물리는 필요 없다.
    box(stage, "/World/Store/Ceiling", size=(lx + 2 * t, ly + 2 * t, t), center=(lx / 2, ly / 2, h + t / 2), collision=False)

    # 벽 4면: 바닥 사각형 바깥에 붙는다. 모서리는 동서 벽이 채운다.
    UsdGeom.Scope.Define(stage, "/World/Store/Walls")
    box(stage, "/World/Store/Walls/West", size=(t, ly + 2 * t, h), center=(-t / 2, ly / 2, h / 2))
    box(stage, "/World/Store/Walls/East", size=(t, ly + 2 * t, h), center=(lx + t / 2, ly / 2, h / 2))
    box(stage, "/World/Store/Walls/South", size=(lx, t, h), center=(lx / 2, -t / 2, h / 2))
    box(stage, "/World/Store/Walls/North", size=(lx, t, h), center=(lx / 2, ly + t / 2, h / 2))

    # 기둥
    UsdGeom.Scope.Define(stage, "/World/Store/Columns")
    c = store.column_size
    for i, (x, y) in enumerate(store.columns):
        box(stage, f"/World/Store/Columns/Column_{i:02d}", size=(c, c, h), center=(x, y, h / 2))

    # 조명: 통로마다 천장에 라인 하나, 아래(-Z)를 비춘다
    UsdGeom.Scope.Define(stage, "/World/Store/Lights")
    for i, cor in enumerate(corridors(store, shelf)):
        (x0, x1), (y0, y1) = cor["x"], cor["y"]
        light = UsdLux.RectLight.Define(stage, f"/World/Store/Lights/Light_{i:02d}")
        UsdGeom.Xformable(light).AddTranslateOp().Set(Gf.Vec3d((x0 + x1) / 2, (y0 + y1) / 2, h - 0.05))
        if cor["axis"] == "y":
            light.CreateWidthAttr(store.light_w)
            light.CreateHeightAttr(y1 - y0)
        else:
            light.CreateWidthAttr(x1 - x0)
            light.CreateHeightAttr(store.light_w)
        light.CreateIntensityAttr(store.light_intensity)
        light.CreateEnableColorTemperatureAttr(True)
        light.CreateColorTemperatureAttr(store.light_temp_k)

    # 진열대
    UsdGeom.Scope.Define(stage, "/World/Shelves")
    for p in placements(store, shelf):
        build_shelf(stage, p["path"], p["spec"], translate=p["translate"], rotate_z_deg=p["rotate"])

    prim = root.GetPrim()
    prim.CreateAttribute("store:footprint", Sdf.ValueTypeNames.Float2).Set(Gf.Vec2f(lx, ly))
    prim.CreateAttribute("store:aisleWidth", Sdf.ValueTypeNames.Float).Set(store.aisle_width)
    prim.CreateAttribute("store:mainAisleWidth", Sdf.ValueTypeNames.Float).Set(store.main_aisle_width)
    prim.CreateAttribute("store:tile", Sdf.ValueTypeNames.Float).Set(store.tile)
    return prim


# ─────────────────────────────────────────────────────────────
def describe(store: StoreSpec = STORE, shelf: ShelfSpec = SHELF, robot: RobotSpec = ROBOT) -> str:
    lx, ly = store.footprint(shelf)
    ps = placements(store, shelf)
    kinds = {k: sum(1 for p in ps if p["kind"] == k) for k in ("wall", "gondola", "endcap")}
    lines = [
        f"  바닥      {lx:.2f} × {ly:.2f} m  (타일 {store.tile * 100:.0f} cm → {lx / store.tile:.1f} × {ly / store.tile:.1f} 장)",
        f"  천장      {store.ceiling_h:.2f} m,  기둥 {len(store.columns)}개 ({store.column_size * 100:.0f} cm 각)",
        f"  진열대    벽면 {kinds['wall']} + 곤돌라 면 {kinds['gondola']} + 엔드캡 {kinds['endcap']} = {len(ps)}대",
    ]
    for cor in corridors(store, shelf):
        (x0, x1), (y0, y1) = cor["x"], cor["y"]
        w = (x1 - x0) if cor["axis"] == "y" else (y1 - y0)
        lines.append(f"  {cor['name']:<8} 폭 {w:.2f} m   x∈[{x0:.2f},{x1:.2f}] y∈[{y0:.2f},{y1:.2f}]")
    lines.append(
        f"  AMR       직진 {robot.corridor_width():.2f} m / 회전 {robot.turn_diameter():.2f} m 필요"
        f"  (본체 {robot.base_w:.2f}×{robot.base_l:.2f}, 여유 {robot.safety_margin:.2f})"
    )
    reach = (store.aisle_width - robot.base_w) / 2 + shelf.level_fronts()[1] + shelf.slot_front_gap
    lines.append(
        f"  팔 도달   통로 중앙에서 상단 선반 상품 앞면까지 {reach:.2f} m"
        f"  (통로 반폭 {store.aisle_width / 2:.2f} − 본체 반폭 {robot.base_w / 2:.2f} + 선반 들어감 {shelf.level_fronts()[1]:.2f} + 앞 여유 {shelf.slot_front_gap:.2f})"
    )
    return "\n".join(lines)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--out", default="out/store.usda")
    args = ap.parse_args()

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.unlink(missing_ok=True)

    tex = out.parent / "floor_tile.png"
    write_tile_png(tex)

    stage = new_stage(out)
    build_store(stage, tile_tex_rel=f"./{tex.name}")
    stage.GetRootLayer().Save()

    print(f"저장: {out}  (+ {tex.name})")
    print(describe())


if __name__ == "__main__":
    main()
