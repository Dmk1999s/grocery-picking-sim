"""Isaac Sim 에서 AMR(Carter v1) 이 시나리오 주문 경로를 실제로 주행한다.

    source ~/.isaac_cache_env
    ~/isaac6-venv/bin/python -m tools.drive_isaac out/scenario_007.json --order 2 --out out/drive
    ~/isaac6-venv/bin/python -m tools.drive_isaac out/scenario_007.json --order 2 --record   # 카메라 프레임 + GIF

scenario.py 가 낸 JSON 의 경유점(dock → via → pick → … → dock)을 차동구동으로
따라간다. 경유점은 계획이고, 여기서 나오는 건 **물리 시뮬 결과**다:
  - 정차점마다 계획 대비 위치·yaw 오차
  - 주행 중 본체(OBB)와 진열대·기둥·벽 AABB 의 최소 간격, 겹침(충돌) 프레임 수
  - 총 이동 거리, 시뮬 시간

제어는 단순 유니사이클 추종기다 (제자리 회전 → 직진, 도착 근처 감속). 위치는
시뮬의 참값(get_world_poses)을 쓴다 — 로컬라이제이션은 아직 없다. 바퀴 부호는
시작할 때 짧게 굴려 보고 스스로 정한다 (에셋마다 조인트 축 방향이 다르다).

결과: <out>/drive_<seed>_<order>.json, --record 면 <out>/frames_*/ 프레임, 정차 순간
PNG, 그리고 tools/frames_to_gif 로 묶은 GIF (체이스 카메라 · 1인칭)
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
ap.add_argument("json", nargs="?", default="out/scenario_007.json")
ap.add_argument("--order", type=int, default=0)
ap.add_argument("--out", default="out/drive")
ap.add_argument("--record", action="store_true", help="체이스·1인칭 카메라 프레임을 저장하고 GIF 를 만든다")
ap.add_argument("--res", type=int, nargs=2, default=(640, 360))
ap.add_argument("--render-every", type=int, default=18, help="물리 스텝 몇 번마다 한 프레임 (60 Hz 기준 18 = 3.3 fps)")
ap.add_argument("--gif-fps", type=int, default=10)
ap.add_argument("--gif-width", type=int, default=400)
ap.add_argument("--gif-frames", type=int, default=120, help="GIF 최대 프레임 (넘으면 건너뛰며 고른다)")
ap.add_argument("--dwell", type=float, default=1.5, help="정차점에서 머무는 시간 s (팔 없을 때)")
ap.add_argument("--teleport", action="store_true", help="주행 없이 정차 자세로 순간이동 (파지 실험용)")
ap.add_argument("--arm", action="store_true", help="Carter 위에 Franka 를 얹고 정차마다 상품을 집어 바구니에 넣는다")
ap.add_argument("--max-steps", type=int, default=60 * 600, help="안전장치")
ap.add_argument("--dt", type=float, default=1 / 60)
args = ap.parse_args()

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": True, "renderer": "RayTracedLighting", "width": args.res[0], "height": args.res[1]})

import numpy as np  # noqa: E402
import omni.usd  # noqa: E402
from pxr import Gf, Usd, UsdGeom  # noqa: E402
import isaacsim.core.experimental.utils.app as app_utils  # noqa: E402
from isaacsim.core.simulation_manager import SimulationManager  # noqa: E402
from isaacsim.robot.experimental.wheeled_robots.controllers.differential_controller import DifferentialController  # noqa: E402
from isaacsim.robot.experimental.wheeled_robots.robots.wheeled_robot import WheeledRobot  # noqa: E402
from isaacsim.storage.native import get_assets_root_path  # noqa: E402

from scene.constants import ROBOT, SHELF, STORE  # noqa: E402
from scene.store import placements  # noqa: E402

sc = json.loads(Path(args.json).read_text())
order = sc["orders"][args.order]
waypoints = order["route"]["waypoints"]
out_dir = Path(args.out).resolve()
out_dir.mkdir(parents=True, exist_ok=True)
tag = f"{sc['seed']:03d}_{order['id']}"

# ── 스테이지
ctx = omni.usd.get_context()
if not ctx.open_stage(str(Path(sc["usd"]).resolve())):
    print(f"열 수 없음: {sc['usd']}")
    app.close()
    sys.exit(2)
stage = ctx.get_stage()

# 장애물 AABB (xy). 검증기와 같은 소스: USD 형상
cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
obstacles: list[tuple[str, np.ndarray, np.ndarray]] = []
for u in placements(STORE, SHELF):
    r = cache.ComputeWorldBound(stage.GetPrimAtPath(u["path"])).ComputeAlignedRange()
    obstacles.append((u["path"], np.array(r.GetMin())[:2], np.array(r.GetMax())[:2]))
for scope in ("/World/Store/Columns", "/World/Store/Walls"):
    for prim in stage.GetPrimAtPath(scope).GetChildren():
        r = cache.ComputeWorldBound(prim).ComputeAlignedRange()
        obstacles.append((str(prim.GetPath()), np.array(r.GetMin())[:2], np.array(r.GetMax())[:2]))
OBS_LO = np.array([o[1] for o in obstacles])
OBS_HI = np.array([o[2] for o in obstacles])

# ── 로봇
dock = sc["dock"]


def quat_z(yaw_deg: float) -> list[float]:
    h = math.radians(yaw_deg) / 2
    return [math.cos(h), 0.0, 0.0, math.sin(h)]


robot = WheeledRobot(
    "/World/Robot",
    wheel_dof_names=list(ROBOT.wheel_joints),
    usd_path=get_assets_root_path() + ROBOT.asset,
    positions=[[dock["x"], dock["y"], ROBOT.spawn_z]],
    orientations=[quat_z(dock["yaw_deg"])],
)
arm = None
if args.arm:
    from tools.arm_isaac import Arm
    arm = Arm(stage, app, init_pose=(dock["x"], dock["y"], math.radians(dock["yaw_deg"])))
app.update()
SimulationManager.setup_simulation(dt=args.dt, device="cpu")
app_utils.play()
app.update()
if arm:
    arm.start()
    arm.follow(dock["x"], dock["y"], math.radians(dock["yaw_deg"]))
    print(f"팔: Franka 베이스 {ROBOT.arm_base_dz:.3f} m, 어깨 {ROBOT.arm_mount_z():.2f} m, tuck 손끝(베이스 기준) {arm.tuck_tcp}, 바구니 위 자세 IK 오차 {arm.over_local_err * 1000:.0f} mm")
# 바퀴는 속도 드라이브: 강성 0, 감쇠는 에셋 값이 작으면 올린다
st, dp = robot.get_dof_gains()
wi = robot._resolve_wheel_dof_indices()
dp = dp.numpy()[0]
robot.set_dof_gains(stiffnesses=[0.0, 0.0], dampings=[max(d, 1e4) for d in dp[wi]], dof_indices=wi)
ctrl = DifferentialController(wheel_radius=ROBOT.wheel_radius, wheel_base=ROBOT.wheel_base,
                              max_linear_speed=ROBOT.v_max, max_angular_speed=ROBOT.w_max)


def pose() -> tuple[float, float, float]:
    p, q = robot.get_world_poses()
    p, q = p.numpy()[0], q.numpy()[0]           # wxyz
    w, x, y, z = q
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return float(p[0]), float(p[1]), yaw


def wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def footprint_points(x: float, y: float, yaw: float, n: int = 6) -> np.ndarray:
    """본체 사각형 둘레 표본점 (간격 계산용)."""
    hl, hw = ROBOT.base_l / 2, ROBOT.base_w / 2
    t = np.linspace(-1, 1, n)
    edge = np.concatenate([
        np.c_[t * hl, np.full(n, hw)], np.c_[t * hl, np.full(n, -hw)],
        np.c_[np.full(n, hl), t * hw], np.c_[np.full(n, -hl), t * hw]])
    c, s = math.cos(yaw), math.sin(yaw)
    return edge @ np.array([[c, s], [-s, c]]) + np.array([x, y])


def clearance(x: float, y: float, yaw: float) -> tuple[float, str]:
    """본체 둘레점과 장애물 AABB 사이 최소 거리 (음수면 겹침). (거리, 장애물)."""
    pts = footprint_points(x, y, yaw)
    d_out = np.maximum(np.maximum(OBS_LO[None] - pts[:, None], pts[:, None] - OBS_HI[None]), 0)   # (P, O, 2)
    dist = np.linalg.norm(d_out, axis=2)
    inside = dist == 0
    if inside.any():
        depth = np.minimum(pts[:, None] - OBS_LO[None], OBS_HI[None] - pts[:, None]).min(axis=2)
        dist = np.where(inside, -depth, dist)
    i = np.unravel_index(np.argmin(dist), dist.shape)
    return float(dist[i]), obstacles[i[1]][0]


# ── 기록용 카메라
recorder = None
if args.record:
    import omni.replicator.core as rep
    from PIL import Image

    rep.create.light(light_type="dome", intensity=1000.0)     # 렌더용 간접광 (render_isaac 과 같은 이유)
    chase = UsdGeom.Camera.Define(stage, "/World/ChaseCam")
    chase.CreateFocalLengthAttr(18.0)
    chase.CreateClippingRangeAttr(Gf.Vec2f(0.05, 200.0))
    fp_path = None
    for prim in Usd.PrimRange(stage.GetPrimAtPath("/World/Robot")):
        if prim.IsA(UsdGeom.Camera) and "first_person" in prim.GetName():
            fp_path = str(prim.GetPath())
    close = UsdGeom.Camera.Define(stage, "/World/CloseCam")      # 파지 순간 손끝 클로즈업 (라벨 프레임에만 저장)
    close.CreateFocalLengthAttr(24.0)
    close.CreateClippingRangeAttr(Gf.Vec2f(0.05, 200.0))
    cams = {"chase": "/World/ChaseCam", "close": "/World/CloseCam"} | ({"fpv": fp_path} if fp_path else {})
    annots = {}
    for name, path in cams.items():
        rp = rep.create.render_product(path, tuple(args.res))
        an = rep.AnnotatorRegistry.get_annotator("rgb")
        an.attach([rp])
        annots[name] = an
    frames_dir = out_dir / f"frames_{tag}"
    frames_dir.mkdir(exist_ok=True)

    def place_chase(x: float, y: float, yaw: float) -> None:
        eye = Gf.Vec3d(x - 2.6 * math.cos(yaw), y - 2.6 * math.sin(yaw), 1.9)
        at = Gf.Vec3d(x + 1.2 * math.cos(yaw), y + 1.2 * math.sin(yaw), 0.5)
        m = Gf.Matrix4d().SetLookAt(eye, at, Gf.Vec3d(0, 0, 1)).GetInverse()
        xf = UsdGeom.Xformable(chase)
        xf.ClearXformOpOrder()
        xf.AddTransformOp().Set(m)

    frame_idx = [0]
    close_target = [None]                 # (eye, at) 를 pick 이 정해 준다

    def place_close() -> None:
        if close_target[0] is None:
            return
        eye, at = close_target[0]
        m = Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye), Gf.Vec3d(*at), Gf.Vec3d(0, 0, 1)).GetInverse()
        xf = UsdGeom.Xformable(close)
        xf.ClearXformOpOrder()
        xf.AddTransformOp().Set(m)

    def capture(label: str | None = None) -> None:
        x, y, yaw = pose()
        place_chase(x, y, yaw)
        place_close()
        app.update()                          # 렌더 (물리도 한 스텝 간다)
        if label:
            app.update()                      # annotator 는 직전 프레임을 돌려주므로 카메라를 옮긴 뒤 한 번 더
        for name, an in annots.items():
            if name == "close" and not label:
                continue
            img = an.get_data()
            if img is None or img.size == 0:
                continue
            im = Image.fromarray(img[..., :3])
            if name != "close":
                im.save(frames_dir / f"{name}_{frame_idx[0]:05d}.png")
            if label:
                im.save(out_dir / f"{tag}_{label}_{name}.png")
        frame_idx[0] += 1

    capture.close_target = close_target

    recorder = capture


def step(n: int = 1) -> None:
    SimulationManager.step(steps=n)


# ── 바퀴 부호 보정: 잠깐 앞으로 굴려 보고 heading 방향으로 갔는지 본다
x0, y0, yaw0 = pose()
robot.apply_wheel_actions(ctrl.forward([0.3, 0.0]))
step(45)
x1, y1, _ = pose()
moved = (x1 - x0) * math.cos(yaw0) + (y1 - y0) * math.sin(yaw0)
SIGN = 1.0 if moved >= 0 else -1.0
robot.apply_wheel_actions([0.0, 0.0])
step(30)
print(f"바퀴 부호 보정: 0.75 s 에 {moved:+.3f} m → sign {SIGN:+.0f}")
# 도크로 되돌린다
robot.set_world_poses(positions=[[dock["x"], dock["y"], ROBOT.spawn_z]], orientations=[quat_z(dock["yaw_deg"])])
robot.set_velocities(linear_velocities=[[0.0, 0.0, 0.0]], angular_velocities=[[0.0, 0.0, 0.0]])
step(10)


def command(v: float, w: float) -> None:
    robot.apply_wheel_actions(SIGN * ctrl.forward([v, w]))


# ── 경유점 추종
K_W, K_V = 2.5, 1.2
TOL_POS, TOL_YAW = 0.03, math.radians(2.0)
sim_t = 0.0
dist_total = 0.0
min_clear, min_clear_at, collide_frames = 1e9, "", 0
picks, trace = [], []
prev = pose()
steps = 0
t_wall = time.time()


BRAKE = [False]     # 파지 중 본체 제동 (속도 0 유지)


def tick(n: int = 1) -> None:
    global sim_t, dist_total, steps, prev, min_clear, min_clear_at, collide_frames
    if arm:
        arm.follow(*prev)
    if BRAKE[0]:
        robot.set_velocities(linear_velocities=[[0.0, 0.0, 0.0]], angular_velocities=[[0.0, 0.0, 0.0]])
    step(n)
    steps += n
    sim_t += n * args.dt
    x, y, yaw = pose()
    dist_total += math.hypot(x - prev[0], y - prev[1])
    prev = (x, y, yaw)
    if steps % 6 == 0:
        c, who = clearance(x, y, yaw)
        if c < min_clear:
            min_clear, min_clear_at = c, who
        if c < 0:
            collide_frames += 1
        trace.append([round(sim_t, 3), round(x, 4), round(y, 4), round(math.degrees(yaw), 1), round(c, 4)])
    if recorder and steps % args.render_every < n:
        recorder()


def turn_to(target_yaw: float) -> None:
    while steps < args.max_steps:
        _, _, yaw = pose()
        e = wrap(target_yaw - yaw)
        if abs(e) < TOL_YAW:
            break
        command(0.0, max(-ROBOT.w_max, min(ROBOT.w_max, K_W * e)))
        tick()
    command(0.0, 0.0)
    tick(6)


def go_to(tx: float, ty: float) -> None:
    while steps < args.max_steps:
        x, y, yaw = pose()
        dx, dy = tx - x, ty - y
        d = math.hypot(dx, dy)
        if d < TOL_POS:
            break
        e = wrap(math.atan2(dy, dx) - yaw)
        if abs(e) > math.radians(30):        # 많이 틀어졌으면 멈추고 돈다
            command(0.0, max(-ROBOT.w_max, min(ROBOT.w_max, K_W * e)))
        else:
            v = max(0.08, min(ROBOT.v_max, K_V * d))
            command(v, max(-ROBOT.w_max, min(ROBOT.w_max, K_W * e)))
        tick()
    command(0.0, 0.0)
    tick(6)


for i, wp in enumerate(waypoints[1:], 1):
    tx, ty = wp["x"], wp["y"]
    if args.teleport:
        if wp["kind"] != "pick":
            continue
        robot.set_world_poses(positions=[[tx, ty, ROBOT.spawn_z]], orientations=[quat_z(wp["yaw_deg"])])
        robot.set_velocities(linear_velocities=[[0.0, 0.0, 0.0]], angular_velocities=[[0.0, 0.0, 0.0]])
        prev = (tx, ty, math.radians(wp["yaw_deg"]))
        tick(30)
    else:
        x, y, _ = pose()
        turn_to(math.atan2(ty - y, tx - x))
        go_to(tx, ty)
    if wp["kind"] in ("pick", "dock") and not args.teleport:
        # 제자리 회전 중 캐스터가 본체를 몇 cm 밀어낸다. 오차가 남으면 한 번 다가간 뒤 다시 돈다.
        # 마지막 동작은 항상 회전이어야 정차 yaw 가 맞는다
        turn_to(math.radians(wp["yaw_deg"]))
        x, y, _ = pose()
        if math.hypot(x - tx, y - ty) > TOL_POS:
            go_to(tx, ty)
            turn_to(math.radians(wp["yaw_deg"]))
    x, y, yaw = pose()
    err = math.hypot(x - tx, y - ty)
    if wp["kind"] == "pick":
        line = order["lines"][wp["line"]]
        yaw_err = math.degrees(wrap(math.radians(wp["yaw_deg"]) - yaw))
        picks.append({
            "line": wp["line"], "product": line["product"], "prim": line["prim"],
            "planned": [tx, ty, wp["yaw_deg"]], "actual": [round(x, 4), round(y, 4), round(math.degrees(yaw), 1)],
            "pos_err_m": round(err, 4), "yaw_err_deg": round(yaw_err, 2), "t_arrive_s": round(sim_t, 2),
        })
        print(f"  정차 {len(picks)}/{len(order['lines'])}  {line['product']:<24} 위치 오차 {err * 100:.1f} cm  yaw 오차 {yaw_err:+.1f}°  t={sim_t:.1f}s")
        if recorder:
            recorder(label=f"pick{len(picks)}")
        if arm:
            from isaacsim.core.experimental.prims import RigidPrim
            unit_items = [str(p.GetPath()) for p in stage.GetPrimAtPath(f"{line['unit']}/Stock").GetChildren()]
            rp = RigidPrim(unit_items)

            def item_poses():
                pos = rp.get_world_poses()[0].numpy()
                return {k: tuple(pos[i]) for i, k in enumerate(unit_items)}

            def on_event(name):
                if recorder:
                    # 클로즈업: 상품 앞 통로 쪽, 비스듬히 위에서 상품 중심을 본다
                    cx_, cy_, cz_ = line["item_center"]
                    nx_, ny_, _ = line["approach_dir"]
                    fx_, fy_ = math.cos(math.radians(wp["yaw_deg"])), math.sin(math.radians(wp["yaw_deg"]))
                    recorder.close_target[0] = ((cx_ - nx_ * 0.55 + fx_ * 0.45, cy_ - ny_ * 0.55 + fy_ * 0.45, cz_ + 0.30), (cx_, cy_, cz_))
                    recorder(label=f"pick{len(picks)}_{name}")

            BRAKE[0] = True
            g = arm.pick(line, item_poses, tick, args.dt, on_event=on_event)
            BRAKE[0] = False
            picks[-1]["grasp"] = g
            print(f"      파지 {'성공' if g.get('success') else '실패'}  단계 {g['phase']}  계획 대비 이동 {g.get('moved_before_pick_m', 0) * 100:.1f} cm  손가락 간격 {g.get('finger_gap_m', 0) * 100:.1f} cm (폭 {g.get('grasp_width_m', 0) * 100:.1f})  들림 {g.get('lift_m', 0) * 100:.1f} cm  잡음 {g['held_after_retract']}  바구니 {g['in_bin']}  이웃 교란 {g['disturbed_neighbors']}  IK 오차 {g['ik_err_max_m'] * 1000:.0f} mm")
        else:
            tick(int(args.dwell / args.dt))
    elif i % 3 == 0:
        print(f"  경유점 {i}/{len(waypoints) - 1}  t={sim_t:.1f}s  이동 {dist_total:.1f} m")

x, y, yaw = pose()
result = {
    "scenario": args.json, "order": order["id"], "robot": ROBOT.asset, "wheel_sign": SIGN,
    "planned_length_m": order["route"]["length_m"], "driven_length_m": round(dist_total, 3),
    "sim_time_s": round(sim_t, 2), "steps": steps, "wall_time_s": round(time.time() - t_wall, 1),
    "completed": steps < args.max_steps,
    "dock_return_err_m": round(math.hypot(x - dock["x"], y - dock["y"]), 4),
    "min_clearance_m": round(min_clear, 4), "min_clearance_at": min_clear_at, "collision_frames": collide_frames,
    "arm": bool(arm),
    "teleport": bool(args.teleport),
    "grasp_success": sum(1 for p in picks if p.get("grasp", {}).get("success")) if arm else None,
    "picks": picks,
    "trace": trace,
}
res_path = out_dir / f"drive_{tag}.json"
res_path.write_text(json.dumps(result, ensure_ascii=False, indent=1))
print(f"\n저장: {res_path}")
print(f"  계획 {result['planned_length_m']} m → 주행 {result['driven_length_m']} m, 시뮬 {sim_t:.1f} s, 벽시계 {result['wall_time_s']} s")
print(f"  최소 간격 {min_clear * 100:.1f} cm ({min_clear_at}), 충돌 프레임 {collide_frames}, 도크 복귀 오차 {result['dock_return_err_m'] * 100:.1f} cm")
if arm:
    print(f"  파지 성공 {result['grasp_success']}/{len(picks)}")

if recorder:
    from tools.frames_to_gif import make_gif
    for name in annots:
        gif = make_gif(frames_dir, name, out_dir / f"drive_{tag}_{name}.gif", width=args.gif_width, fps=args.gif_fps, max_frames=args.gif_frames)
        if gif:
            print(f"  GIF: {gif}  ({gif.stat().st_size / 1e6:.1f} MB)")

app.close()
