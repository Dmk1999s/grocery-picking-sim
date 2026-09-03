"""진열대 생성기 — 순수 USD.

Isaac Sim 없이 노트북에서 돈다 (`pip install usd-core`).
Isaac 은 결과 .usda 를 읽기만 한다.

    python -m scene.shelf --out out/shelf.usda

형상은 전부 constants.ShelfSpec 에서 나온다. 이 파일에 치수를 적지 말 것.
"""

from __future__ import annotations

import argparse
from pathlib import Path

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

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


def _box(
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


# ─────────────────────────────────────────────────────────────
# 슬롯 — 상품이 놓일 자리
# ─────────────────────────────────────────────────────────────
def slot_positions(spec: ShelfSpec = SHELF) -> list[dict]:
    """상품 배치용 슬롯 목록.

    stock.py 가 이걸 받아서 YCB/GSO 에셋을 채운다. 각 슬롯은 자기 자리에
    들어갈 수 있는 상품의 최대 치수를 같이 들고 있어서, 안 맞는 에셋을
    배치 단계에서 걸러낼 수 있다.

    좌표는 선반 로컬 프레임 (원점 = 바닥면 통로쪽 앞면 중앙, +X 안쪽).
    """
    inner_w = spec.width - 2 * spec.side_t
    pitch = inner_w / spec.slots_per_level
    usable_d = spec.depth - spec.back_t - spec.slot_front_gap
    clearances = spec.level_clearances()   # 단마다 다르다

    slots: list[dict] = []
    for li, z in enumerate(spec.level_heights()):
        for si in range(spec.slots_per_level):
            slots.append(
                {
                    "level": li,
                    "index": si,
                    # 상품 바닥 중심이 놓일 자리
                    "x_front": spec.slot_front_gap,  # 상품 앞면의 x
                    "y": -inner_w / 2 + pitch * (si + 0.5),
                    "z": z,                          # 선반판 윗면
                    # 이 슬롯에 들어갈 수 있는 상품 크기 상한
                    "max_w": spec.slot_width(),
                    "max_d": usable_d,
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
    inner_w = w - 2 * spec.side_t

    # 프레임 -------------------------------------------------
    frame = f"{prim_path}/Frame"
    UsdGeom.Scope.Define(stage, frame)

    # 측판 두 장: 깊이 전체 × 두께 × 높이 전체
    for tag, sign in (("L", -1.0), ("R", +1.0)):
        _box(
            stage,
            f"{frame}/Side_{tag}",
            size=(d, spec.side_t, h),
            center=(d / 2, sign * (w / 2 - spec.side_t / 2), h / 2),
        )

    # 뒷판: 맨 안쪽
    _box(
        stage,
        f"{frame}/Back",
        size=(spec.back_t, inner_w, h),
        center=(d - spec.back_t / 2, 0.0, h / 2),
    )

    # 선반판 -------------------------------------------------
    # 윗면이 level_heights() 높이에 오도록 중심을 판 두께의 절반만큼 내린다.
    board_d = d - spec.back_t
    for li, z_top in enumerate(spec.level_heights()):
        _box(
            stage,
            f"{prim_path}/Level_{li:02d}",
            size=(board_d, inner_w, spec.board_t),
            center=(board_d / 2, 0.0, z_top - spec.board_t / 2),
        )

    # 슬롯을 메타데이터로 남겨둔다 — stock.py 가 USD 만 보고도 채울 수 있게.
    prim = root.GetPrim()
    prim.CreateAttribute("shelf:nLevels", Sdf.ValueTypeNames.Int).Set(spec.n_levels)
    prim.CreateAttribute("shelf:slotsPerLevel", Sdf.ValueTypeNames.Int).Set(
        spec.slots_per_level
    )
    prim.CreateAttribute("shelf:levelHeights", Sdf.ValueTypeNames.FloatArray).Set(
        [float(z) for z in spec.level_heights()]
    )
    return prim


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

    s = SHELF
    print(f"저장: {out}")
    print(f"  외형      {s.width:.2f} × {s.depth:.2f} × {s.height:.2f} m (W×D×H)")
    print(f"  선반 {s.n_levels}단  높이 {[f'{z:.2f}' for z in s.level_heights()]}")
    cl = ", ".join(f"{c * 100:.0f}" for c in s.level_clearances())
    print(f"  단별 여유 [{cl}] cm  ← 상품 높이 상한 (최상단이 다르다)")
    print(f"  슬롯 {len(slot_positions())}개  폭 {s.slot_width() * 100:.1f} cm")


if __name__ == "__main__":
    main()
