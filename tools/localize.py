"""2D 로컬라이제이션 — 라이다 + 매장 지도 + 파티클 필터 (순수 numpy, Isaac 은 drive_isaac 이 붙인다).

지도: constants/store.placements 의 진열대·기둥·벽 AABB 를 5 cm 격자로 굽는다. 실측 매장이라면 CAD 도면과 같다.
센서 모델: 우도장(likelihood field). 격자마다 "가장 가까운 장애물까지 거리"를 미리 계산해 두고,
  스캔 끝점을 파티클 자세로 투영한 자리의 그 거리를 가우시안(σ 8 cm)에 넣어 곱한다.
운동 모델: 차동구동 오도메트리(바퀴 각속도 → v, ω) + 잡음.
파티클 300개, 스캔마다 리샘플(ESS < N/2), 추정 = 가중 평균 (yaw 는 원형 평균).

    python -m tools.localize --selftest      # 지도에서 합성 스캔을 만들어 PF 가 수렴하는지 (Isaac 없이)
"""

from __future__ import annotations

import argparse
import math

import numpy as np

from scene.constants import SHELF, STORE
from scene.store import placements

CELL = 0.05


class StoreMap:
    def __init__(self, cell: float = CELL):
        lx, ly = STORE.footprint(SHELF)
        self.cell = cell
        self.nx, self.ny = int(math.ceil(lx / cell)) + 2, int(math.ceil(ly / cell)) + 2
        occ = np.zeros((self.ny, self.nx), dtype=bool)
        rects = []
        for u in placements(STORE, SHELF):
            x, y, _ = u["translate"]; sp = u["spec"]; r = u["rotate"]
            # 배치 회전에 따른 바닥 사각형 (선반 로컬: x∈[0,depth], y∈[-w/2,w/2])
            if r == 0:      rects.append((x, y - sp.width / 2, x + sp.depth, y + sp.width / 2))
            elif r == 180:  rects.append((x - sp.depth, y - sp.width / 2, x, y + sp.width / 2))
            elif r == 90:   rects.append((x - sp.width / 2, y, x + sp.width / 2, y + sp.depth))
            else:           rects.append((x - sp.width / 2, y - sp.depth, x + sp.width / 2, y))
        h = STORE.column_size / 2
        for cx, cy in STORE.columns(SHELF):
            rects.append((cx - h, cy - h, cx + h, cy + h))
        t = STORE.wall_t
        rects += [(-t, -t, lx + t, 0), (-t, ly, lx + t, ly + t), (-t, -t, 0, ly + t), (lx, -t, lx + t, ly + t)]
        for x0, y0, x1, y1 in rects:
            i0, i1 = max(0, int(x0 / cell)), min(self.nx - 1, int(x1 / cell))
            j0, j1 = max(0, int(y0 / cell)), min(self.ny - 1, int(y1 / cell))
            occ[j0:j1 + 1, i0:i1 + 1] = True
        self.occ = occ
        self.rects = rects
        # 우도장: 장애물까지 거리 (m)
        from scipy.ndimage import distance_transform_edt
        self.dist = distance_transform_edt(~occ) * cell

    def lookup(self, xs, ys):
        i = np.clip((xs / self.cell).astype(int), 0, self.nx - 1)
        j = np.clip((ys / self.cell).astype(int), 0, self.ny - 1)
        return self.dist[j, i]

    def raycast(self, x, y, yaw, angles, max_range=20.0, step=None):
        """합성 스캔 (자체 시험·시각화용). 격자를 따라 걸어가 첫 장애물까지 거리."""
        step = step or self.cell / 2
        out = np.full(len(angles), max_range)
        for k, a in enumerate(angles):
            c, s = math.cos(yaw + a), math.sin(yaw + a)
            for r in np.arange(0.05, max_range, step):
                i, j = int((x + c * r) / self.cell), int((y + s * r) / self.cell)
                if i < 0 or j < 0 or i >= self.nx or j >= self.ny or self.occ[j, i]:
                    out[k] = r
                    break
        return out


class ParticleFilter:
    def __init__(self, store_map: StoreMap, n: int = 300, sigma_hit: float = 0.08, seed: int = 0):
        self.map = store_map
        self.n = n
        self.sigma = sigma_hit
        self.rng = np.random.default_rng(seed)
        self.p = np.zeros((n, 3))
        self.w = np.ones(n) / n

    def init_around(self, x, y, yaw, sxy=0.3, syaw=0.2):
        self.p[:, 0] = x + self.rng.normal(0, sxy, self.n)
        self.p[:, 1] = y + self.rng.normal(0, sxy, self.n)
        self.p[:, 2] = yaw + self.rng.normal(0, syaw, self.n)
        self.w[:] = 1 / self.n

    def predict(self, v, w, dt, a=(0.10, 0.05, 0.02, 0.05)):
        """오도메트리 (v m/s, w rad/s) 적분 + 잡음. a = 이동·회전 잡음 계수 [설계]. 진행 방향 잡음(a[0])을 크게 두는 이유:
        부통로에선 평행한 진열대만 보여 진행 방향이 약하게 관측되고, 출발 가속 때 바퀴가 미끄러져 오도메트리가 10 cm 넘게 틀어진다.
        파티클이 진행 방향으로 퍼져 있어야 멀리 있는 엔드캡·벽 빔이 맞는 쪽을 골라낸다."""
        d, dth = v * dt, w * dt
        d_n = d + self.rng.normal(0, a[0] * abs(d) + a[1] * abs(dth) + 1e-4, self.n)
        th_n = dth + self.rng.normal(0, a[2] * abs(d) + a[3] * abs(dth) + 1e-4, self.n)
        self.p[:, 0] += d_n * np.cos(self.p[:, 2] + th_n / 2)
        self.p[:, 1] += d_n * np.sin(self.p[:, 2] + th_n / 2)
        self.p[:, 2] = (self.p[:, 2] + th_n + math.pi) % (2 * math.pi) - math.pi

    def update(self, ranges, angles, max_range=20.0, sub=2, offset=0.0):
        """스캔으로 가중치. 빔을 sub 개마다 하나만 써서(독립 가정) 가중치 붕괴를 막는다. offset = 라이다가 본체 중심에서 앞으로 떨어진 거리."""
        idx = np.arange(0, len(ranges), sub)
        r, a = ranges[idx], angles[idx]
        ok = (r > 0.1) & (r < max_range * 0.98)
        r, a = r[ok], a[ok]
        if len(r) < 5:
            return
        th = self.p[:, 2][:, None] + a[None, :]
        lx = self.p[:, 0] + offset * np.cos(self.p[:, 2])
        ly = self.p[:, 1] + offset * np.sin(self.p[:, 2])
        ex = lx[:, None] + r[None, :] * np.cos(th)
        ey = ly[:, None] + r[None, :] * np.sin(th)
        d = self.map.lookup(ex.ravel(), ey.ravel()).reshape(self.n, -1)
        logw = -0.5 * (d / self.sigma) ** 2
        logw = np.clip(logw, -20, 0).sum(axis=1)
        logw -= logw.max()
        self.w = np.exp(logw)
        self.w /= self.w.sum()
        if 1.0 / (self.w ** 2).sum() < self.n / 2:
            self.resample()

    def resample(self):
        cdf = np.cumsum(self.w)
        u = (self.rng.random() + np.arange(self.n)) / self.n
        idx = np.searchsorted(cdf, u)
        self.p = self.p[idx] + self.rng.normal(0, [0.01, 0.01, 0.01], (self.n, 3))
        self.w[:] = 1 / self.n

    def estimate(self):
        x = (self.w * self.p[:, 0]).sum(); y = (self.w * self.p[:, 1]).sum()
        yaw = math.atan2((self.w * np.sin(self.p[:, 2])).sum(), (self.w * np.cos(self.p[:, 2])).sum())
        return x, y, yaw


def selftest() -> None:
    m = StoreMap()
    print(f"지도 {m.nx}×{m.ny} 셀 ({m.cell} m), 장애물 {m.occ.mean() * 100:.1f} %")
    angles = np.deg2rad(np.arange(-135, 135.1, 1.0))
    rng = np.random.default_rng(1)
    pf = ParticleFilter(m, n=300)
    # 참 궤적: 부통로 1 을 따라 올라간다
    x0, x1 = STORE.aisle_x_range(1, SHELF); x = (x0 + x1) / 2; y = 2.5; yaw = math.pi / 2
    pf.init_around(x + 0.2, y - 0.2, yaw + 0.1)
    errs = []
    for k in range(60):
        v, w = 0.5, (0.3 if 20 < k < 30 else 0.0)
        dt = 0.2
        x += v * dt * math.cos(yaw); y += v * dt * math.sin(yaw); yaw += w * dt
        pf.predict(v * (1 + rng.normal(0, 0.03)), w + rng.normal(0, 0.01), dt)
        scan = m.raycast(x, y, yaw, angles) + rng.normal(0, 0.02, len(angles))
        pf.update(scan, angles)
        ex, ey, eyaw = pf.estimate()
        errs.append((math.hypot(ex - x, ey - y), abs((eyaw - yaw + math.pi) % (2 * math.pi) - math.pi)))
    e = np.array(errs)
    print(f"60 스텝: 위치 오차 처음 {e[0, 0] * 100:.0f} cm → 마지막 {e[-1, 0] * 100:.1f} cm (중앙값 {np.median(e[10:, 0]) * 100:.1f} cm), yaw 마지막 {math.degrees(e[-1, 1]):.1f}°")


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--selftest", action="store_true")
    args = ap.parse_args()
    if args.selftest:
        selftest()
