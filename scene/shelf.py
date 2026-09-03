"""진열대(곤돌라 한 면) 생성기 — 순수 USD.

Isaac Sim 없이 노트북에서 돈다 (`pip install usd-core`).
Isaac 은 결과 .usda 를 읽기만 한다.

    python -m scene.shelf --out out/shelf.usda

형상은 전부 constants.ShelfSpec 에서 나온다. 이 파일에 치수를 적지 말 것.

구조 (constants.py 의 그림 참고):
  Frame/Post_L, Post_R   뒤쪽 지주 두 개. 측면은 뚫려 있다.
  Frame/Back             백판
  Frame/Kick             걸레받이 (바닥 데크 아래, 앞면이 들어가 있음)
  Level_00               바닥 데크 (깊다)
  Level_01..             상단 선반 (얕다, 앞면이 데크보다 안쪽)
  Rail_NN                각 단 앞단의 가격표 레일
"""

from __future__ import annotations

import argparse
from pathlib import Path

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

from scene.constants import METERS_PER_UNIT, SHELF, ShelfSpec


# ─────────────────────────────────────────────────────────────
# USD 헬퍼
# ─────────────────────────────────────────────────────────────
def new_stage(path: str | Path) -> Usd.Stage:
    """Z-up · 미터 단위 스테이지. Isaac Sim 규약에 맞춘다."""
    stage = Usd.Stage.CreateNew(str(path))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, METERS_PER_UNIT)
    UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(stage.GetPrimAtPath("/World"))
    return stage


def box(
    stage: Usd.Stage,
    path: str,
    size: tuple[float, float, float],
    center: tuple[float, float, float],
    *,
    collision: bool = True,
) -> UsdGeom.Cube:
    """축정렬 직육면체 하나. 단위 큐브를 스케일해서 만든다.

    정적 콜라이더로만 쓴다 — RigidBodyAPI 를 붙이지 않으므로 움직이지 않는다.
    """
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    cube.CreateExtentAttr([Gf.Vec3f(-0.5, -0.5, -0.5), Gf.Vec3f(0.5, 0.5, 0.5)])

    xf = UsdGeom.Xformable(cube)
    xf.AddTranslateOp().Set(Gf.Vec3d(*center))
    xf.AddScaleOp().Set(Gf.Vec3f(*size))

    if collision:
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    return cube


def material(stage: Usd.Stage, path: str, color: tuple[float, float, float], *, roughness: float = 0.5, metallic: float = 0.0) -> UsdShade.Material:
    """단색 UsdPreviewSurface. 이미 있으면 그대로 돌려준다 (매장에서 137대가 공유)."""
    prim = stage.GetPrimAtPath(path)
    if prim:
        return UsdShade.Material(prim)
    mat = UsdShade.Material.Define(stage, path)
    sh = UsdShade.Shader.Define(stage, f"{path}/Surface")
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)
    sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metallic)
    mat.CreateSurfaceOutput().ConnectToSource(sh.CreateOutput("surface", Sdf.ValueTypeNames.Token))
    return mat


def bind(prim: Usd.Prim, mat: UsdShade.Material) -> None:
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(mat)


# 곤돌라 색: 국내 마트 진열대는 대개 흰색/아이보리 분체도장 강판, 레일은 흰색 또는 유색
def shelf_materials(stage: Usd.Stage) -> dict[str, UsdShade.Material]:
    return {
        "steel": material(stage, "/World/Looks/ShelfSteel", (0.86, 0.86, 0.84), roughness=0.45, metallic=0.2),
        "post": material(stage, "/World/Looks/ShelfPost", (0.55, 0.56, 0.58), roughness=0.5, metallic=0.4),
        "back": material(stage, "/World/Looks/ShelfBack", (0.90, 0.90, 0.88), roughness=0.7),
        "kick": material(stage, "/World/Looks/ShelfKick", (0.25, 0.25, 0.26), roughness=0.8),
        "rail": material(stage, "/World/Looks/PriceRail", (0.95, 0.95, 0.95), roughness=0.3),
    }


# ─────────────────────────────────────────────────────────────
# 슬롯 — 상품이 놓일 자리
# ─────────────────────────────────────────────────────────────
def slot_positions(spec: ShelfSpec = SHELF) -> list[dict]:
    """상품 배치용 슬롯 목록.

    stock.py 가 이걸 받아서 YCB/GSO 에셋을 채운다. 각 슬롯은 자기 자리에
    들어갈 수 있는 상품의 최대 치수를 같이 들고 있어서, 안 맞는 에셋을
    배치 단계에서 걸러낼 수 있다.

    좌표는 선반 로컬 프레임 (원점 = 바닥 데크 앞면 중앙, +X 안쪽).
    상단 선반은 데크보다 얕으므로 x_front 가 단마다 다르다.
    """
    inner_w = spec.inner_width()
    pitch = inner_w / spec.slots_per_level
    back_x = spec.depth - spec.back_t
    clearances = spec.level_clearances()   # 단마다 다르다
    fronts = spec.level_fronts()

    slots: list[dict] = []
    for li, z in enumerate(spec.level_heights()):
        x_front = fronts[li] + spec.slot_front_gap
        for si in range(spec.slots_per_level):
            slots.append(
                {
                    "level": li,
                    "index": si,
                    # 상품 바닥 앞면 중심이 놓일 자리
                    "x_front": x_front,
                    "y": -inner_w / 2 + pitch * (si + 0.5),
                    "z": z,                          # 선반 윗면
                    # 이 슬롯에 들어갈 수 있는 상품 크기 상한
                    "max_w": spec.slot_width(),
                    "max_d": back_x - x_front,
                    "max_h": clearances[li],
                }
            )
    return slots


# ─────────────────────────────────────────────────────────────
# 진열대 생성
# ─────────────────────────────────────────────────────────────
def build_shelf(
    stage: Usd.Stage,
    prim_path: str,
    spec: ShelfSpec = SHELF,
    *,
    translate: tuple[float, float, float] = (0.0, 0.0, 0.0),
    rotate_z_deg: float = 0.0,
) -> Usd.Prim:
    """진열대 한 대를 만들고 루트 Xform 을 돌려준다.

    translate/rotate_z_deg 로 매장 안에 배치한다 (store.py 가 쓴다).
    """
    root = UsdGeom.Xform.Define(stage, prim_path)
    rx = UsdGeom.Xformable(root)
    rx.AddTranslateOp().Set(Gf.Vec3d(*translate))
    if rotate_z_deg:
        rx.AddRotateZOp().Set(rotate_z_deg)

    w, d, h = spec.width, spec.depth, spec.height
    inner_w = spec.inner_width()
    back_x = d - spec.back_t           # 백판 앞면의 x
    mats = shelf_materials(stage)

    # 프레임 -------------------------------------------------
    frame = f"{prim_path}/Frame"
    UsdGeom.Scope.Define(stage, frame)

    # 지주 두 개: 백판 바로 앞, 좌우 끝. 측면은 그 외에 아무것도 없다.
    for tag, sign in (("L", -1.0), ("R", +1.0)):
        post = box(
            stage,
            f"{frame}/Post_{tag}",
            size=(spec.post_d, spec.post_w, h),
            center=(back_x - spec.post_d / 2, sign * (w / 2 - spec.post_w / 2), h / 2),
        )
        bind(post.GetPrim(), mats["post"])

    # 백판: 맨 안쪽, 지주 사이
    back = box(
        stage,
        f"{frame}/Back",
        size=(spec.back_t, inner_w, h),
        center=(d - spec.back_t / 2, 0.0, h / 2),
    )
    bind(back.GetPrim(), mats["back"])

    # 걸레받이: 바닥 데크 아래, 앞면이 데크보다 들어가 있다
    kick_h = spec.bottom_z - spec.deck_t
    kick_d = back_x - spec.kick_setback
    kick = box(
        stage,
        f"{frame}/Kick",
        size=(kick_d, inner_w, kick_h),
        center=(spec.kick_setback + kick_d / 2, 0.0, kick_h / 2),
    )
    bind(kick.GetPrim(), mats["kick"])

    # 단 -----------------------------------------------------
    # 윗면이 level_heights() 높이에 오도록 중심을 두께의 절반만큼 내린다.
    heights = spec.level_heights()
    fronts = spec.level_fronts()
    thick = spec.level_thickness()
    for li, z_top in enumerate(heights):
        depth_i = back_x - fronts[li]
        lvl = box(
            stage,
            f"{prim_path}/Level_{li:02d}",
            size=(depth_i, inner_w, thick[li]),
            center=(fronts[li] + depth_i / 2, 0.0, z_top - thick[li] / 2),
        )
        bind(lvl.GetPrim(), mats["steel"])
        # 가격표 레일: 앞단에 걸려 윗면과 같은 높이에서 아래로 내려온다
        rail = box(
            stage,
            f"{prim_path}/Rail_{li:02d}",
            size=(spec.rail_t, inner_w, spec.rail_h),
            center=(fronts[li] - spec.rail_t / 2, 0.0, z_top - spec.rail_h / 2),
        )
        bind(rail.GetPrim(), mats["rail"])

    # 슬롯을 메타데이터로 남겨둔다 — stock.py 가 USD 만 보고도 채울 수 있게.
    prim = root.GetPrim()
    prim.CreateAttribute("shelf:nLevels", Sdf.ValueTypeNames.Int).Set(spec.n_levels)
    prim.CreateAttribute("shelf:slotsPerLevel", Sdf.ValueTypeNames.Int).Set(
        spec.slots_per_level
    )
    prim.CreateAttribute("shelf:levelHeights", Sdf.ValueTypeNames.FloatArray).Set(
        [float(z) for z in heights]
    )
    prim.CreateAttribute("shelf:levelFronts", Sdf.ValueTypeNames.FloatArray).Set(
        [float(x) for x in fronts]
    )
    return prim


def describe(spec: ShelfSpec = SHELF, indent: str = "  ") -> str:
    hs = spec.level_heights()
    cl = ", ".join(f"{c * 100:.0f}" for c in spec.level_clearances())
    lines = [
        f"{indent}외형      {spec.width:.2f} × {spec.depth:.2f} × {spec.height:.2f} m (W×D×H)",
        f"{indent}상단 선반 깊이 {spec.shelf_depth:.2f} m  (데크보다 {(spec.level_fronts()[1]) * 100:.0f} cm 안쪽)",
        f"{indent}{spec.n_levels}단  높이 {[f'{z:.3f}' for z in hs]}  (홀 피치 {spec.hole_pitch * 1000:.0f} mm 스냅)",
        f"{indent}단별 여유 [{cl}] cm  ← 상품 높이 상한 (최상단이 다르다)",
        f"{indent}슬롯 {len(slot_positions(spec))}개  폭 {spec.slot_width() * 100:.1f} cm",
    ]
    return "\n".join(lines)


# ─────────────────────────────────────────────────────────────
def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--out", default="out/shelf.usda")
    args = ap.parse_args()

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.unlink(missing_ok=True)

    stage = new_stage(out)
    build_shelf(stage, "/World/Shelf_00")
    stage.GetRootLayer().Save()

    print(f"저장: {out}")
    print(describe())


if __name__ == "__main__":
    main()
