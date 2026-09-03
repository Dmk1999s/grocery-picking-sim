"""YCB 구글 스캔 메시를 받아 USD 로 바꾸고 치수 카탈로그를 만든다.

    aws s3 sync --no-sign-request --exclude "*" --include "*_google_16k.tgz" \\
        s3://ycb-benchmarks/data/google/ assets/ycb/raw/
    python -m tools.ycb_catalog                      # 전부
    python -m tools.ycb_catalog --only 003_cracker_box

산출물
  assets/ycb/models/<id>/google_16k/   압축 해제 (gitignore)
  assets/ycb/usd/<id>.usd + <id>.png   USD 에셋(크레이트 바이너리): 메시 + 텍스처 + 물리 (gitignore)
  assets/ycb/catalog.json              이름·치수·질량·분류. 커밋한다 — stock.py 는 이것만 본다

USD 에셋 규약
  루트 Xform = 물체 밑면 중심 (스캔 원점이 제각각이라 여기서 맞춘다). Z-up, 미터.
  RigidBodyAPI + MassAPI(질량) + convexHull 콜라이더. 선반 위에 그냥 놓으면 된다.
"""

from __future__ import annotations

import argparse
import json
import re
import shutil
import tarfile
from pathlib import Path

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade, Vt

from scene.constants import METERS_PER_UNIT

RAW = Path("assets/ycb/raw")
MODELS = Path("assets/ycb/models")
USD_DIR = Path("assets/ycb/usd")
CATALOG = Path("assets/ycb/catalog.json")

# 질량(g): Calli et al., "The YCB Object and Model Set" (arXiv:1502.03143) Table I.
# 논문 표에 없는 네 개(021, 053, 055, 077)는 나중에 추가된 물체라 YCB 웹 목록 값이다.
# 마트에서 팔 만한 물건에 분류를 붙였다. 분류가 None 이면 매장에 놓지 않는다.
#   식품 / 생활용품 / 주방 / 문구 / 스포츠 / 완구
YCB_INFO: dict[str, tuple[float | None, str | None]] = {
    "001_chips_can": (205, "식품"),
    "002_master_chef_can": (414, "식품"),
    "003_cracker_box": (453, "식품"),
    "004_sugar_box": (514, "식품"),
    "005_tomato_soup_can": (349, "식품"),
    "006_mustard_bottle": (431, "식품"),
    "007_tuna_fish_can": (171, "식품"),
    "008_pudding_box": (187, "식품"),
    "009_gelatin_box": (97, "식품"),
    "010_potted_meat_can": (370, "식품"),
    "011_banana": (66, "식품"),
    "012_strawberry": (18, "식품"),
    "013_apple": (68, "식품"),
    "014_lemon": (29, "식품"),
    "015_peach": (33, "식품"),
    "016_pear": (49, "식품"),
    "017_orange": (47, "식품"),
    "018_plum": (25, "식품"),
    "019_pitcher_base": (178, "주방"),
    "021_bleach_cleanser": (1131, "생활용품"),   # 웹 목록
    "022_windex_bottle": (1022, "생활용품"),
    "023_wine_glass": (133, "주방"),
    "024_bowl": (147, "주방"),
    "025_mug": (118, "주방"),
    "026_sponge": (6.2, "생활용품"),
    "029_plate": (279, "주방"),
    "030_fork": (34, "주방"),
    "031_spoon": (30, "주방"),
    "032_knife": (31, "주방"),
    "033_spatula": (105, "주방"),
    "037_scissors": (82, "문구"),
    "040_large_marker": (16, "문구"),
    "041_small_marker": (8.3, "문구"),
    "053_mini_soccer_ball": (123, "스포츠"),     # 웹 목록
    "054_softball": (191, "스포츠"),
    "055_baseball": (148, "스포츠"),             # 웹 목록
    "056_tennis_ball": (58, "스포츠"),
    "057_racquetball": (41, "스포츠"),
    "058_golf_ball": (46, "스포츠"),
    "062_dice": (5.2, "완구"),
    "077_rubiks_cube": (94, "완구"),             # 웹 목록
}


def parse_obj(path: Path):
    """textured.obj → (points, uvs, face_counts, face_vert_idx, face_uv_idx)."""
    pts, uvs, counts, vidx, tidx = [], [], [], [], []
    with path.open() as f:
        for line in f:
            if line.startswith("v "):
                _, x, y, z = line.split()[:4]
                pts.append((float(x), float(y), float(z)))
            elif line.startswith("vt "):
                _, u, v = line.split()[:3]
                uvs.append((float(u), float(v)))
            elif line.startswith("f "):
                verts = line.split()[1:]
                counts.append(len(verts))
                for v in verts:
                    parts = v.split("/")
                    vidx.append(int(parts[0]) - 1)
                    tidx.append(int(parts[1]) - 1 if len(parts) > 1 and parts[1] else -1)
    return pts, uvs, counts, vidx, tidx


def write_usd(name: str, src_dir: Path) -> dict:
    pts, uvs, counts, vidx, tidx = parse_obj(src_dir / "textured.obj")
    xs, ys, zs = zip(*pts)
    lo = (min(xs), min(ys), min(zs))
    hi = (max(xs), max(ys), max(zs))
    # 원점을 밑면 중심으로
    off = ((lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, lo[2])
    pts = [(x - off[0], y - off[1], z - off[2]) for x, y, z in pts]
    dims = (hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2])

    USD_DIR.mkdir(parents=True, exist_ok=True)
    usd_path = USD_DIR / f"{name}.usd"     # 크레이트: 16k 면 메시가 텍스트면 6 MB, 바이너리면 1 MB
    tex_src = src_dir / "texture_map.png"
    tex_dst = USD_DIR / f"{name}.png"
    if tex_src.exists():
        shutil.copyfile(tex_src, tex_dst)

    stage = Usd.Stage.CreateNew(str(usd_path))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, METERS_PER_UNIT)
    prim_name = "ycb_" + re.sub(r"[^A-Za-z0-9_]", "_", name)   # 프림 이름: 숫자 시작·하이픈 불가
    root = UsdGeom.Xform.Define(stage, f"/{prim_name}")
    stage.SetDefaultPrim(root.GetPrim())
    Usd.ModelAPI(root.GetPrim()).SetKind("component")

    mesh = UsdGeom.Mesh.Define(stage, f"/{prim_name}/geom")
    mesh.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*p) for p in pts]))
    mesh.CreateFaceVertexCountsAttr(Vt.IntArray(counts))
    mesh.CreateFaceVertexIndicesAttr(Vt.IntArray(vidx))
    mesh.CreateExtentAttr([Gf.Vec3f(-dims[0] / 2, -dims[1] / 2, 0), Gf.Vec3f(dims[0] / 2, dims[1] / 2, dims[2])])
    mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    if uvs and all(t >= 0 for t in tidx):
        pv = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.faceVarying)
        pv.Set(Vt.Vec2fArray([Gf.Vec2f(*uv) for uv in uvs]))
        pv.SetIndices(Vt.IntArray(tidx))

    # 머티리얼: 스캔 텍스처
    if tex_src.exists():
        mat = UsdShade.Material.Define(stage, f"/{prim_name}/Looks/Scan")
        sh = UsdShade.Shader.Define(stage, f"/{prim_name}/Looks/Scan/Surface")
        sh.CreateIdAttr("UsdPreviewSurface")
        sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.6)
        rd = UsdShade.Shader.Define(stage, f"/{prim_name}/Looks/Scan/StReader")
        rd.CreateIdAttr("UsdPrimvarReader_float2")
        rd.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
        tx = UsdShade.Shader.Define(stage, f"/{prim_name}/Looks/Scan/Texture")
        tx.CreateIdAttr("UsdUVTexture")
        tx.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(f"./{tex_dst.name}")
        tx.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(rd.CreateOutput("result", Sdf.ValueTypeNames.Float2))
        sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(tx.CreateOutput("rgb", Sdf.ValueTypeNames.Float3))
        mat.CreateSurfaceOutput().ConnectToSource(sh.CreateOutput("surface", Sdf.ValueTypeNames.Token))
        UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(mat)

    # 물리: 강체 + 볼록껍질 콜라이더 + 질량
    mass_g, category = YCB_INFO.get(name, (None, None))
    UsdPhysics.RigidBodyAPI.Apply(root.GetPrim())
    UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
    UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr(UsdPhysics.Tokens.convexHull)
    if mass_g:
        UsdPhysics.MassAPI.Apply(root.GetPrim()).CreateMassAttr(mass_g / 1000.0)

    stage.GetRootLayer().Save()
    return {
        "name": name,
        "usd": str(usd_path),
        "dims": [round(d, 4) for d in dims],        # x, y, z 크기 (m). z 가 높이
        "mass_kg": (mass_g / 1000.0) if mass_g else None,
        "category": category,
        "n_faces": len(counts),
    }


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--only", default=None, help="이름 하나만")
    args = ap.parse_args()

    tgzs = sorted(RAW.glob("*_google_16k.tgz"))
    if args.only:
        tgzs = [t for t in tgzs if t.name.startswith(args.only)]
    if not tgzs:
        print(f"{RAW} 에 tgz 가 없다. 모듈 docstring 의 aws s3 sync 를 먼저.")
        return

    rows = []
    for tgz in tgzs:
        name = tgz.name.replace("_google_16k.tgz", "")
        # tgz 안 폴더 이름이 물체마다 조금씩 다르다 (예: 076_timer → 076_timer/google_16k 가 아님).
        # 풀어놓고 textured.obj 를 찾는다.
        dst = MODELS / name
        objs = list(dst.rglob("textured.obj"))
        if not objs:
            dst.mkdir(parents=True, exist_ok=True)
            with tarfile.open(tgz) as tf:
                tf.extractall(dst)
            objs = list(dst.rglob("textured.obj"))
        if not objs:
            print(f"  {name:<28} textured.obj 없음 — 건너뜀")
            continue
        row = write_usd(name, objs[0].parent)
        rows.append(row)
        d = row["dims"]
        print(f"  {name:<28} {d[0] * 100:5.1f} × {d[1] * 100:5.1f} × {d[2] * 100:5.1f} cm  {row['mass_kg'] or '-':>6}  {row['category'] or '-'}")

    CATALOG.parent.mkdir(parents=True, exist_ok=True)
    CATALOG.write_text(json.dumps(rows, ensure_ascii=False, indent=1))
    print(f"\n{len(rows)}개 → {CATALOG}")


if __name__ == "__main__":
    main()
