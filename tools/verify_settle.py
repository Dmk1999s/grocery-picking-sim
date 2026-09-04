"""Isaac 물리를 잠깐 돌린 뒤 상품이 제자리에 있는지 본다 (트윈 물리 안정성).

    source ~/.isaac_cache_env
    ~/isaac6-venv/bin/python -m tools.verify_settle out/scenario_007.usda --seconds 2

verify_stock 은 기하(슬롯 안, 선반 접촉)를 보지만, 볼록껍질 콜라이더 밑면이 울퉁불퉁하면
물리가 시작되자마자 넘어지는 상품이 있다 (세정제가 두 seed 에서 매번 그랬다). 이건 USD 만
읽어서는 못 잡고 물리를 돌려야 보인다. 전체 상품의 이동량·회전을 재고 품목별로 집계한다.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from collections import Counter
from pathlib import Path

ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
ap.add_argument("usd", nargs="?", default="out/scenario_007.usda")
ap.add_argument("--seconds", type=float, default=2.0)
ap.add_argument("--tol", type=float, default=0.02, help="이동 허용 m")
ap.add_argument("--out", default=None, help="JSON 결과 (기본 <usd>_settle.json)")
args = ap.parse_args()

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": True})

import numpy as np  # noqa: E402
import omni.usd  # noqa: E402
import isaacsim.core.experimental.utils.app as app_utils  # noqa: E402
from isaacsim.core.experimental.prims import RigidPrim  # noqa: E402
from isaacsim.core.simulation_manager import SimulationManager  # noqa: E402

from tools.verify_shelf import Check  # noqa: E402

ctx = omni.usd.get_context()
if not ctx.open_stage(str(Path(args.usd).resolve())):
    print(f"열 수 없음: {args.usd}")
    app.close()
    sys.exit(2)
stage = ctx.get_stage()
paths = [str(p.GetPath()) for p in stage.Traverse() if p.GetPath().name.startswith("Item_") and p.GetParent().GetPath().name == "Stock"]
products = [stage.GetPrimAtPath(p).GetAttribute("stock:product").Get() for p in paths]
states = [stage.GetPrimAtPath(p).GetAttribute("stock:state").Get() for p in paths]
rp = RigidPrim(paths)
app.update()
SimulationManager.setup_simulation(dt=1 / 60, device="cpu")
app_utils.play()
app.update()
p0, q0 = rp.get_world_poses()
p0, q0 = p0.numpy().copy(), q0.numpy().copy()
SimulationManager.step(steps=int(args.seconds * 60))
p1, q1 = rp.get_world_poses()
p1, q1 = p1.numpy(), q1.numpy()
moved = np.linalg.norm(p1 - p0, axis=1)
# 회전각: 쿼터니언 내적
dots = np.clip(np.abs((q0 * q1).sum(axis=1)), 0, 1)
rot = np.degrees(2 * np.arccos(dots))

is_fallen = np.array([s_ == "fallen" for s_ in states])
# 서 있는 상품은 2 cm, 누운 상품은 구를 수 있으니 10 cm (둥근 병이 4~5 cm 구른다 — 물리적으로 맞다)
bad = np.where(is_fallen, moved > 5 * args.tol, moved > args.tol)
c = Check()
print(f"\n{args.usd}  물리 {args.seconds:.1f} s 뒤\n")
up = ~is_fallen
c.true(f"서 있는 상품 {up.sum()}개 중 {args.tol * 100:.0f} cm 넘게 움직인 것 없음", not bad[up].any(),
       f"{bad[up].sum()}개 ({bad[up].mean() * 100:.2f} %)  최대 이동 {moved[up].max() * 100:.1f} cm, 최대 회전 {rot[up].max():.0f}°")
by_prod = Counter(products[i] for i in np.where(bad)[0])
by_state = Counter(states[i] for i in np.where(bad)[0])
tot = Counter(products)
for name, n in by_prod.most_common(8):
    print(f"      {name:<26} {n}/{tot[name]}  ({n / tot[name] * 100:.0f} %)")
if by_state:
    print(f"      상태별: {dict(by_state)}")
if is_fallen.any():
    c.true(f"누운 상품 {is_fallen.sum()}개가 {5 * args.tol * 100:.0f} cm 넘게 안 굴러감", not bad[is_fallen].any(),
           f"{bad[is_fallen].sum()}개 굴러감, 최대 {moved[is_fallen].max() * 100:.1f} cm")
out = Path(args.out) if args.out else Path(args.usd).with_name(Path(args.usd).stem + "_settle.json")
out.write_text(json.dumps({"usd": args.usd, "seconds": args.seconds, "tol": args.tol, "n": len(paths), "moved": int(bad.sum()),
                           "by_product": dict(by_prod), "by_state": dict(by_state),
                           "worst": [{"prim": paths[i], "product": products[i], "moved_m": round(float(moved[i]), 3), "rot_deg": round(float(rot[i]), 1)}
                                     for i in np.argsort(-moved)[:20]]}, ensure_ascii=False, indent=1))
print(f"\n저장: {out}")
rc = 1 if c.report() else 0
app.close()
sys.exit(rc)
