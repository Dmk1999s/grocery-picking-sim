"""Isaac Sim 헤드리스로 USD 를 실제 렌더한다 (RTX).

    source ~/.isaac_cache_env
    ~/isaac6-venv/bin/python -m tools.render_isaac out/store.usda --out docs/img/isaac

카메라 몇 대를 정해진 자리에 놓고 RGB 를 저장한다. 첫 실행은 셰이더 컴파일로
몇 분 걸린다. matplotlib 렌더(render_3d.py)는 형상 확인용이고, 이건 포트폴리오·
인지 파이프라인용이다.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

ap = argparse.ArgumentParser(description=__doc__)
ap.add_argument("usd")
ap.add_argument("--out", default="out/isaac")
ap.add_argument("--res", type=int, nargs=2, default=(1600, 900))
ap.add_argument("--frames", type=int, default=48, help="수렴용 프레임 수 (RTX 누적)")
ap.add_argument("--views", default="all", help="all 또는 콤마 구분 이름")
args = ap.parse_args()

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": True, "renderer": "RayTracedLighting", "width": args.res[0], "height": args.res[1]})

import omni.replicator.core as rep  # noqa: E402
import omni.usd  # noqa: E402
from pxr import Gf, Usd, UsdGeom  # noqa: E402

from scene.constants import SHELF, STORE  # noqa: E402

usd_path = str(Path(args.usd).resolve())
ctx = omni.usd.get_context()
ok = ctx.open_stage(usd_path)
if not ok:
    print(f"열 수 없음: {usd_path}")
    app.close()
    sys.exit(2)
stage = ctx.get_stage()

lx, ly = STORE.footprint(SHELF)
is_store = bool(stage.GetPrimAtPath("/World/Store"))

# 카메라 시점: 이름 → (위치, 바라보는 점, 화각 mm)
if is_store:
    a0 = STORE.aisle_x_range(0, SHELF)
    ax0 = (a0[0] + a0[1]) / 2
    ry = STORE.run_y_range(SHELF)
    VIEWS = {
        # 매장 전체를 앞 모서리 천장 바로 아래에서 (벽 안쪽)
        "overview": ((0.8, 0.8, STORE.ceiling_h - 0.4), (lx * 0.55, ly * 0.55, 0.6), 14.0),
        # 부통로 0 안, 사람 눈높이에서 통로 끝을 향해
        "aisle": ((ax0, ry[0] - 0.8, 1.55), (ax0, ry[1], 1.1), 24.0),
        # 곤돌라 한 면을 통로 반대편에서 정면으로 (팔 카메라가 볼 시야)
        "shelf_face": ((a0[0] + 1.6, ry[0] + 2.4, 1.2), (a0[0] - 0.3, ry[0] + 2.4, 0.9), 20.0),
        # 앞 주통로에서 엔드캡 열을 비스듬히
        "main_aisle": ((1.0, 1.0, 1.6), (lx * 0.6, STORE.main_aisle_width + 1.5, 0.9), 24.0),
    }
else:
    s = SHELF
    VIEWS = {
        "shelf": ((-2.6, -2.1, 1.5), (s.depth / 2, 0.0, s.height * 0.5), 22.0),
    }
names = list(VIEWS) if args.views == "all" else args.views.split(",")

# RayTracedLighting 모드는 간접광이 약해 그림자 쪽이 검게 나온다.
# 렌더용으로만 돔 라이트를 얹어 간접광을 흉내낸다 (장면 USD 에는 없다).
rep.create.light(light_type="dome", intensity=1000.0)

out_dir = Path(args.out).resolve()   # 상대경로면 BasicWriter 가 ~/omni.replicator_out 아래에 쓴다
out_dir.mkdir(parents=True, exist_ok=True)

for name in names:
    pos, look, focal = VIEWS[name]
    cam = rep.create.camera(position=pos, look_at=look, focal_length=focal, clipping_range=(0.05, 200.0))
    rp = rep.create.render_product(cam, tuple(args.res))
    writer = rep.WriterRegistry.get("BasicWriter")
    writer.initialize(output_dir=str(out_dir / name), rgb=True)
    writer.attach([rp])
    # 프레임을 여러 번 돌려 RTX 누적을 수렴시킨다. 마지막 프레임만 쓰면 된다.
    for _ in range(args.frames):
        # 물리를 멈추고 렌더만. 안 멈추면 프레임마다 실시간 경과(RTX 1 s/프레임)만큼 물리가 진행돼 빼곡한 상품이 터진다
        rep.orchestrator.step(rt_subframes=4, delta_time=0.0, pause_timeline=True)
    rep.orchestrator.wait_until_complete()
    writer.detach()
    rp.destroy()
    print(f"렌더: {name} → {out_dir / name}")

app.close()
