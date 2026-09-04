"""검출기 학습용 합성 데이터 — 정차 자세의 헤드 카메라 시점에서 RGB + 2D 박스를 찍는다.

    source ~/.isaac_cache_env
    ~/isaac6-venv/bin/python -m tools.dataset_isaac out/scenario_007.usda --n 300 --out out/dataset/train
    ~/isaac6-venv/bin/python -m tools.dataset_isaac out/scenario_003.usda --n 100 --out out/dataset/val

로봇·물리 없이 카메라만 옮긴다 (프레임당 ~0.3 s). 시점은 scenario.pick_pose 가 주는
정차 자세 + perceive_isaac 의 헤드 카메라 오프셋에 위치·조준 흔들림을 더한 것 —
실제 정차 오차(3~4 cm, 2°)를 흉내낸다. 조명(돔 라이트)도 프레임마다 흔든다.

출력 (YOLO 형식):
  <out>/images/000123.png
  <out>/labels/000123.txt        class cx cy w h  (0~1 정규화), 가림 비율 > 0.7 은 뺀다
  <out>/classes.txt              상품명 순서 = class id
  <out>/meta.json                시점·프레임별 대상 프림
"""

from __future__ import annotations

import argparse
import json
import math
import random
import sys
from pathlib import Path

ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
ap.add_argument("usd", nargs="?", default="out/scenario_007.usda")
ap.add_argument("--n", type=int, default=300)
ap.add_argument("--out", default="out/dataset/train")
ap.add_argument("--seed", type=int, default=0)
ap.add_argument("--res", type=int, nargs=2, default=(640, 480))
ap.add_argument("--start", type=int, default=0, help="파일 번호 시작 (여러 매장을 한 세트로 합칠 때)")
args = ap.parse_args()

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": True, "renderer": "RayTracedLighting", "width": args.res[0], "height": args.res[1]})

import numpy as np  # noqa: E402
import omni.replicator.core as rep  # noqa: E402
import omni.usd  # noqa: E402
from PIL import Image  # noqa: E402
from pxr import Gf, Usd, UsdGeom  # noqa: E402

from scene.constants import ROBOT, SHELF, STORE  # noqa: E402
from scene.scenario import pick_pose, unit_normal  # noqa: E402
from scene.stock import load_catalog  # noqa: E402
from scene.store import placements  # noqa: E402
from tools.perceive_isaac import CAM_LOCAL, FOCAL_MM  # noqa: E402

rng = random.Random(args.seed)
ctx = omni.usd.get_context()
if not ctx.open_stage(str(Path(args.usd).resolve())):
    print(f"열 수 없음: {args.usd}")
    app.close()
    sys.exit(2)
stage = ctx.get_stage()
classes = [r["name"] for r in load_catalog()]
cls_id = {n: i for i, n in enumerate(classes)}

# 시점 후보: 맨 앞 상품 전부의 정차 자세 (파지 가능 여부는 상관없다 — 검출기는 다 봐야 한다)
cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
units = {u["path"]: u for u in placements(STORE, SHELF)}
views = []
for path, unit in units.items():
    stock = stage.GetPrimAtPath(f"{path}/Stock")
    if not stock:
        continue
    for prim in stock.GetChildren():
        if prim.GetAttribute("stock:facing").Get() != 0:
            continue
        r = cache.ComputeWorldBound(prim).ComputeAlignedRange()
        lo, hi = r.GetMin(), r.GetMax()
        c = tuple((lo[i] + hi[i]) / 2 for i in range(3))
        pose = pick_pose(unit, c)
        views.append((str(prim.GetPath()), c, pose))
rng.shuffle(views)
views = views[: args.n]
print(f"시점 {len(views)}개 (맨 앞 상품 정차 자세에서 샘플)")

dome = rep.create.light(light_type="dome", intensity=1000.0)
cam = UsdGeom.Camera.Define(stage, "/World/DatasetCam")
cam.CreateFocalLengthAttr(FOCAL_MM)
cam.CreateClippingRangeAttr(Gf.Vec2f(0.05, 50.0))
rp = rep.create.render_product("/World/DatasetCam", tuple(args.res))
ann_rgb = rep.AnnotatorRegistry.get_annotator("rgb")
ann_bb = rep.AnnotatorRegistry.get_annotator("bounding_box_2d_tight_fast")
ann_rgb.attach([rp])
ann_bb.attach([rp])

out = Path(args.out)
(out / "images").mkdir(parents=True, exist_ok=True)
(out / "labels").mkdir(parents=True, exist_ok=True)
(out / "classes.txt").write_text("\n".join(classes) + "\n")
W, H = args.res
meta = []
for k, (prim_path, c, pose) in enumerate(views):
    i = args.start + k
    # 정차 오차 흉내: 위치 ±4 cm, yaw ±2°
    x = pose["x"] + rng.gauss(0, 0.02)
    y = pose["y"] + rng.gauss(0, 0.02)
    yaw = math.radians(pose["yaw_deg"] + rng.gauss(0, 1.0))
    cs, sn = math.cos(yaw), math.sin(yaw)
    ex, ey, ez = CAM_LOCAL
    eye = Gf.Vec3d(x + cs * ex - sn * ey, y + sn * ex + cs * ey, ROBOT.spawn_z + ez)
    # 조준: 대상 중심 ± 몇 cm (팬틸트 오차)
    at = Gf.Vec3d(c[0] + rng.gauss(0, 0.03), c[1] + rng.gauss(0, 0.03), c[2] + rng.gauss(0, 0.03))
    m = Gf.Matrix4d().SetLookAt(eye, at, Gf.Vec3d(0, 0, 1)).GetInverse()
    xf = UsdGeom.Xformable(cam)
    xf.ClearXformOpOrder()
    xf.AddTransformOp().Set(m)
    # 조명 흔들림
    with dome:
        rep.modify.attribute("inputs:intensity", rng.uniform(400.0, 1600.0))
    # 물리가 없으니 orchestrator.step 을 써도 된다 (주행 중 캡처와 다르다). annotator 가 한 프레임 늦어 두 번
    rep.orchestrator.step(rt_subframes=2)
    rep.orchestrator.step(rt_subframes=2)
    rgb = ann_rgb.get_data()
    bb = ann_bb.get_data()
    if rgb is None or rgb.size == 0:
        continue
    Image.fromarray(rgb[..., :3]).save(out / "images" / f"{i:06d}.png")
    labels = bb["info"].get("idToLabels", {})
    lines = []
    n_kept = 0
    for row, path in zip(bb["data"], bb["info"].get("primPaths", [])):
        if float(row["occlusionRatio"]) > 0.7:
            continue
        lab = labels.get(str(row["semanticId"]), labels.get(int(row["semanticId"]), {}))
        name = lab.get("class") if isinstance(lab, dict) else None
        if name not in cls_id:
            continue
        x0, y0, x1, y1 = int(row["x_min"]), int(row["y_min"]), int(row["x_max"]), int(row["y_max"])
        if x1 - x0 < 4 or y1 - y0 < 4:
            continue
        lines.append(f"{cls_id[name]} {(x0 + x1) / 2 / W:.6f} {(y0 + y1) / 2 / H:.6f} {(x1 - x0) / W:.6f} {(y1 - y0) / H:.6f}")
        n_kept += 1
    (out / "labels" / f"{i:06d}.txt").write_text("\n".join(lines) + ("\n" if lines else ""))
    meta.append({"image": f"{i:06d}.png", "target": prim_path, "n_boxes": n_kept, "usd": args.usd})
    if k % 50 == 0:
        print(f"  {k}/{len(views)}  박스 {n_kept}")
(out / f"meta_{Path(args.usd).stem}.json").write_text(json.dumps(meta, ensure_ascii=False))
print(f"저장: {out}  이미지 {len(meta)}장, 박스 평균 {np.mean([m['n_boxes'] for m in meta]):.1f}")
app.close()
