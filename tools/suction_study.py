"""상품 앞면이 흡착 컵에 붙을 수 있는지 — 메시에서 평탄도를 계산한다 (Isaac 불필요).

    .venv/bin/python -m tools.suction_study            # 표 + assets/ycb/suction.json

진열된 자세(라벨이 통로를 봄)에서 컵이 닿는 자리(앞면 중앙)를 잡고, 컵 지름 안의 표면이
얼마나 평평한지 잰다. 벨로즈 컵은 약간의 굴곡을 메우지만 한계가 있다.

  요철(sag)   컵 지름 안에서 앞면이 앞뒤로 벌어진 정도. ⌀100 mm 캔에 ⌀40 mm 컵이면 4.2 mm
  기울기      앞면 평면의 법선과 접근 방향 사이 각
  무게        컵 하나가 드는 한계. 수직면이므로 무게는 전단이고, 진공력 × 마찰이 버틴다:
              W ≤ μ · (부압 × 컵 면적) / 안전율.  컵을 키우면 무게는 되지만 굴곡을 못 탄다 (요철 ∝ 컵반경²)

세 조건을 다 넘으면 그 상품은 이 컵으로 못 집는다 — 컵을 키우거나(무게↑, 요철 허용↑),
여러 컵을 쓰거나, 다른 면을 노려야 한다. 그 판단의 근거를 숫자로 남기는 것이 이 도구다.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np
from pxr import Usd, UsdGeom

from scene.constants import ROBOT
from scene.stock import CATALOG, load_catalog


def mesh_points(usd_path: Path) -> np.ndarray:
    stage = Usd.Stage.Open(str(usd_path))
    chunks = []
    for prim in stage.Traverse():
        if not prim.IsA(UsdGeom.Mesh):
            continue
        pts = np.asarray(UsdGeom.Mesh(prim).GetPointsAttr().Get(), dtype=float)
        m = np.asarray(UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default()), dtype=float)
        chunks.append((np.c_[pts, np.ones(len(pts))] @ m)[:, :3])
    return np.concatenate(chunks)


def _disc_stats(q: np.ndarray, cy: float, cz: float, cup_r: float) -> dict | None:
    """컵 중심 (cy, cz) 에 앉혔을 때 앞면 평탄도."""
    d2 = (q[:, 1] - cy) ** 2 + (q[:, 2] - cz) ** 2
    disc = q[d2 <= cup_r ** 2]
    if len(disc) < 20:
        return None
    shell = disc[disc[:, 0] <= disc[:, 0].min() + 0.03]      # 앞 껍질만 (뒷면 제외)
    if len(shell) < 20:
        return None
    A = np.c_[shell[:, 1] - cy, shell[:, 2] - cz, np.ones(len(shell))]
    coef, *_ = np.linalg.lstsq(A, shell[:, 0], rcond=None)
    resid = shell[:, 0] - A @ coef
    normal = np.array([-1.0, coef[0], coef[1]])
    normal /= np.linalg.norm(normal)
    tilt = math.degrees(math.acos(min(1.0, abs(np.dot(normal, [-1.0, 0, 0])))))
    return {"sag_m": float(resid.max() - resid.min()), "rms_m": float(np.sqrt((resid ** 2).mean())),
            "tilt_deg": float(tilt), "cy": float(cy), "cz": float(cz), "face_x": float(disc[:, 0].min()), "n": int(len(shell))}


def front_face_stats(pts: np.ndarray, yaw_deg: float, cup_r: float, z_off: float = 0.0, search: bool = True) -> dict:
    """진열 자세(yaw)의 앞면(−X 쪽)에서 **컵을 앉힐 가장 좋은 자리**를 찾는다.

    +X = 선반 안쪽이므로 앞면은 x 최소 쪽. 실제 흡착 계획기가 하듯 면 위를 훑어
    기울기 조건을 만족하면서 요철이 가장 작은 자리를 고른다 (병은 어깨가 아니라 몸통을 문다).
    컵이 면 밖으로 나가면 진공이 새므로 컵 반경 + 5 mm 안쪽만 후보다.
    """
    a = math.radians(yaw_deg)
    rz = np.array([[math.cos(a), -math.sin(a), 0], [math.sin(a), math.cos(a), 0], [0, 0, 1]])
    q = pts @ rz.T
    lo, hi = q.min(axis=0), q.max(axis=0)
    face_w, face_h = hi[1] - lo[1], hi[2] - lo[2]
    margin = cup_r + 0.005
    too_small = face_w < 2 * margin or face_h < 2 * margin
    if too_small:
        st = _disc_stats(q, (lo[1] + hi[1]) / 2, (lo[2] + hi[2]) / 2 + z_off, cup_r)
        if st is None:
            return {"ok": False, "reason": "컵 자리에 표면이 없음"}
        return {"ok": True, "too_small": True, "width_m": float(face_w), "height_m": float(face_h), **st}
    if search:
        ys = np.linspace(lo[1] + margin, hi[1] - margin, 5)
        zs = np.linspace(lo[2] + margin, hi[2] - margin, 7)
    else:
        ys, zs = [(lo[1] + hi[1]) / 2], [(lo[2] + hi[2]) / 2 + z_off]
    best = None
    for cy in ys:
        for cz in zs:
            st = _disc_stats(q, cy, cz, cup_r)
            if st is None:
                continue
            # 기울기 조건을 만족하는 자리 중 요철 최소. 아무 데도 안 되면 요철 최소 자리를 남겨 이유를 보여준다
            key = (st["tilt_deg"] > ROBOT.suction_max_tilt_deg, st["sag_m"])
            if best is None or key < best[0]:
                best = (key, st)
    if best is None:
        return {"ok": False, "reason": "컵 자리에 표면이 없음"}
    return {"ok": True, "too_small": False, "width_m": float(face_w), "height_m": float(face_h), **best[1]}


def evaluate(rows_pts: list[tuple[dict, np.ndarray]], cup_r: float, sag_limit: float, z_off: float = 0.0) -> list[dict]:
    """컵 반경·요철 한계 하나에 대해 상품별 판정."""
    payload = ROBOT.suction_payload_kg(cup_r)
    out = []
    for r, pts in rows_pts:
        best = None
        for yaw in (0.0, 90.0):
            st = front_face_stats(pts, yaw, cup_r, z_off)
            if not st["ok"]:
                continue
            st["yaw"] = yaw
            key = (st["too_small"], st["tilt_deg"] > ROBOT.suction_max_tilt_deg, st["sag_m"])
            if best is None or key < best[0]:
                best = (key, st)
        best = best[1] if best else None
        if best is None:
            continue
        heavy, small = r["mass_kg"] > payload, best["too_small"]
        tilt_bad = best["tilt_deg"] > ROBOT.suction_max_tilt_deg
        sag_bad = best["sag_m"] > sag_limit
        why = "무거움" if heavy else ("면이 컵보다 작음" if small else ("기울기" if tilt_bad else ("요철" if sag_bad else "")))
        out.append({"name": r["name"], "category": r["category"], "mass_kg": r["mass_kg"], "yaw": best["yaw"],
                    "sag_mm": round(best["sag_m"] * 1000, 1), "rms_mm": round(best["rms_m"] * 1000, 1),
                    "tilt_deg": round(best["tilt_deg"], 1), "too_heavy": heavy, "too_small": small,
                    "face_w_mm": round(best["width_m"] * 1000), "face_h_mm": round(best["height_m"] * 1000),
                    "cup_dy_m": round(best["cy"], 4), "cup_dz_m": round(best["cz"], 4),
                    "payload_kg": round(payload, 2), "why": why, "suction_ok": not (heavy or small or tilt_bad or sag_bad)})
    return out


def sweep(rows_pts, sag_limit: float, out_png: str | None) -> None:
    """컵 지름을 훑어 몇 종을 집을 수 있는지. 컵 선택의 근거."""
    radii = [0.005 * i for i in range(1, 7)]
    print(f"\n컵 지름 스윕 (요철 한계 {sag_limit * 1000:.0f} mm, 기울기 {ROBOT.suction_max_tilt_deg:.0f}°)")
    print(f"  {'컵⌀mm':>7}{'드는무게kg':>11}{'가능':>6}{'요철X':>7}{'작은면X':>8}{'기울기X':>8}{'무거움X':>8}")
    xs, ys = [], []
    for r in radii:
        rows = evaluate(rows_pts, r, sag_limit)
        c = {k: sum(1 for x in rows if x["why"] == k) for k in ("요철", "면이 컵보다 작음", "기울기", "무거움")}
        n_ok = sum(1 for x in rows if x["suction_ok"])
        xs.append(r * 2000); ys.append(n_ok / len(rows) * 100)
        print(f"  {r * 2000:>7.0f}{ROBOT.suction_payload_kg(r):>11.2f}{n_ok:>6}{c['요철']:>7}{c['면이 컵보다 작음']:>8}{c['기울기']:>8}{c['무거움']:>8}")
    # 두 컵을 갈아 끼우면(툴 체인저) 어디까지 되는가 — 작은 컵은 굴곡, 큰 컵은 무게를 맡는다
    best_pair, best_n = None, -1
    evals = {r: {x["name"]: x["suction_ok"] for x in evaluate(rows_pts, r, sag_limit)} for r in radii}
    names = list(next(iter(evals.values())))
    for i, r1 in enumerate(radii):
        for r2 in radii[i + 1:]:
            n = sum(1 for nm in names if evals[r1].get(nm) or evals[r2].get(nm))
            if n > best_n:
                best_pair, best_n = (r1, r2), n
    print(f"  두 컵 (⌀{best_pair[0] * 2000:.0f} + ⌀{best_pair[1] * 2000:.0f} mm): {best_n}/{len(names)} ({best_n / len(names) * 100:.0f} %)")
    if out_png:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from tools.plan_store import _have  # 한글 폰트
        fig, ax = plt.subplots(figsize=(6, 3.6))
        ax.plot(xs, ys, marker="o", color="#1f77b4")
        for x, y in zip(xs, ys):
            ax.annotate(f"{ROBOT.suction_payload_kg(x / 2000):.1f} kg", (x, y), textcoords="offset points", xytext=(0, 7), ha="center", fontsize=7, color="#555")
        ax.set_xlabel("흡착 컵 지름 [mm]  (라벨 = 그 컵이 드는 무게)")
        ax.set_ylabel("집을 수 있는 상품 [%]")
        ax.set_title(f"컵이 크면 무겁게 들지만 굴곡을 못 탄다 (요철 한계 {sag_limit * 1000:.0f} mm)", fontsize=9)
        ax.grid(alpha=0.3); ax.set_ylim(0, 100)
        fig.tight_layout(); fig.savefig(out_png, dpi=130)
        print(f"저장: {out_png}")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--catalog", default=str(CATALOG))
    ap.add_argument("--out", default="assets/ycb/suction.json")
    ap.add_argument("--cup-r", type=float, default=ROBOT.suction_cup_radius)
    ap.add_argument("--z-off", type=float, default=0.0, help="컵 중심 높이 (앞면 중앙 기준)")
    ap.add_argument("--sag", type=float, default=ROBOT.suction_max_gap, help="컵이 메울 수 있는 요철 한계 m")
    ap.add_argument("--sweep", action="store_true", help="컵 지름을 훑어 몇 종을 집을 수 있는지")
    ap.add_argument("--sweep-png", default="docs/img/suction_cup_sweep.png")
    args = ap.parse_args()

    rows_pts = [(r, mesh_points(Path(args.catalog).parent / "usd" / f"{r['name']}.usd")) for r in load_catalog(Path(args.catalog))]
    if args.sweep:
        sweep(rows_pts, args.sag, args.sweep_png)
    rows = evaluate(rows_pts, args.cup_r, args.sag, args.z_off)
    rows.sort(key=lambda r: r["sag_mm"])
    # 배치 생성기가 쓰는 형태: 상품 × 진열 자세(yaw) 별 판정. 실제로 놓인 자세로 조회한다
    payload = ROBOT.suction_payload_kg(args.cup_r)
    per_yaw = {}
    for r, pts in rows_pts:
        e = {}
        for yaw in (0, 90):
            st = front_face_stats(pts, float(yaw), args.cup_r, args.z_off)
            if not st["ok"]:
                continue
            zf = (st["cz"] - pts[:, 2].min()) / max(1e-6, pts[:, 2].max() - pts[:, 2].min())
            # 컵 자리의 표면은 AABB 앞면(가장 튀어나온 점)보다 이만큼 안쪽에 있다. 곡면 상품은 이걸 모르면 컵이 안 닿는다
            a_ = math.radians(yaw)
            rz_ = np.array([[math.cos(a_), -math.sin(a_), 0], [math.sin(a_), math.cos(a_), 0], [0, 0, 1]])
            face_dx = float(st["face_x"] - (pts @ rz_.T)[:, 0].min())
            e[str(yaw)] = {"sag_mm": round(st["sag_m"] * 1000, 2), "tilt_deg": round(st["tilt_deg"], 1),
                           "too_small": bool(st["too_small"]), "cup_z_frac": round(float(zf), 3),
                           "face_dx_m": round(face_dx, 4),
                           "ok": bool(not st["too_small"] and st["sag_m"] <= args.sag
                                      and st["tilt_deg"] <= ROBOT.suction_max_tilt_deg and r["mass_kg"] <= payload)}
        per_yaw[r["name"]] = {"mass_kg": r["mass_kg"], "yaw": e}
    Path(args.out).write_text(json.dumps({"cup_radius_m": args.cup_r, "sag_limit_m": args.sag,
                                          "tilt_limit_deg": ROBOT.suction_max_tilt_deg, "payload_kg": round(payload, 3),
                                          "products": per_yaw, "table": rows}, ensure_ascii=False, indent=1))
    ok = [r for r in rows if r["suction_ok"]]
    print(f"\n컵 ⌀{args.cup_r * 2000:.0f} mm, 요철 한계 {args.sag * 1000:.0f} mm, 기울기 {ROBOT.suction_max_tilt_deg:.0f}°, 드는 무게 {ROBOT.suction_payload_kg(args.cup_r):.2f} kg")
    print(f"상품 {len(rows)}종 중 흡착 가능 {len(ok)}종 ({len(ok) / len(rows) * 100:.0f} %)\n")
    print(f"  {'상품':<24}{'분류':<8}{'요철mm':>7}{'기울기°':>8}{'무게kg':>8}{'면 mm':>12}  판정")
    for r in rows:
        why = r["why"]
        print(f"  {r['name'].split('_', 1)[1]:<24}{r['category']:<8}{r['sag_mm']:>7.1f}{r['tilt_deg']:>8.1f}{r['mass_kg']:>8.3f}"
              f"{r['face_w_mm']:>5.0f}×{r['face_h_mm']:<5.0f}  {'O' if r['suction_ok'] else 'X ' + why}")
    print(f"\n저장: {args.out}")


if __name__ == "__main__":
    main()
