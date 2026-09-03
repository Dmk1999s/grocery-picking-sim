"""치수 단일 진실 공급원.

이 파일의 숫자 하나를 바꾸면 선반 형상·매장 레이아웃·상품 슬롯·검증이
전부 따라온다. USD 를 손으로 만지지 말고 항상 여기서 시작할 것.

값의 출처를 [ ] 로 표시한다:
  [실측]  실제 마트에서 잰 값        ← 디지털 트윈의 근거
  [잠정]  아직 안 잰 값. 실측 후 교체 ← docs/SURVEY.md 참고
  [표준]  규격·문헌·시판 카탈로그에서 온 값
  [설계]  우리가 정한 값

잠정값은 국내 마트용 곤돌라 진열대 카탈로그(W1200 × D500 × H1800 급)와
대형마트 매장 설계 관행에서 가져왔다. 브랜드·지점마다 다르므로 실측 전에는
"실제 마트와 같은 구조, 대표적인 치수"로 읽어야 한다.

단위는 전부 미터(m), 킬로그램(kg). USD 스테이지도 metersPerUnit=1.0 이다.
"""

from dataclasses import dataclass, replace

# ─────────────────────────────────────────────────────────────
# 좌표계 규약
# ─────────────────────────────────────────────────────────────
# 월드: Z-up (Isaac Sim 기본).
#
# 선반 로컬 프레임 (ShelfSpec 이 만드는 Xform 기준):
#   원점 = 바닥 데크의 통로쪽 앞면 중앙, 바닥 높이
#   +X = 선반 안쪽(깊이 방향)      ← 로봇 팔이 진입하는 방향
#   +Y = 선반 폭 방향(정면에서 봤을 때 오른쪽)
#   +Z = 위
#
# 논문(arXiv:2409.15465) Fig.2a 의 item frame 과 축 방향을 맞췄다.
# 파지 계획을 yz 평면에 투영한다는 것은 곧 "통로에서 정면으로 본 실루엣"이다.
#
# 매장 프레임 (store.py):
#   원점 = 매장 바닥 모서리 (실측할 때 벽 모서리에서 재는 것과 같다)
#   +X = 통로를 가로지르는 방향 (진열대 열이 X 방향으로 번갈아 선다)
#   +Y = 통로를 따라가는 방향 (진열대 열의 길이 방향)

UP_AXIS = "Z"
METERS_PER_UNIT = 1.0


# ─────────────────────────────────────────────────────────────
# 진열대 (곤돌라 한 면)
# ─────────────────────────────────────────────────────────────
# 실제 곤돌라 구조:
#
#        ┌─백판─┐
#   지주 │      │ 지주        ← 측면은 뚫려 있다. 통판 측판은 열 끝(엔드
#        ├──────┤  상단 선반     사이드)에만 붙고, 그것도 얇은 장식판이다.
#        ├──────┤  (얕다, 뒤 지주에 브래킷으로 걸림)
#        ├──────┤
#     ┌──┴──────┤  바닥 데크 (깊다)
#     │  걸레받이 │  ← 앞면이 데크보다 안으로 들어가 있어 발이 들어간다
#   ──┴─────────┴──
#
# 팔이 측면에서 들어갈 수 있고, 카메라가 옆 진열대 너머를 볼 수 있다는 점이
# 상자 모델과 다르다. 바닥 데크가 상단보다 깊다는 것은 위 단의 상품이
# 통로에서 10 cm 안쪽에 있다는 뜻이라, 팔 도달 거리 계산에 바로 들어간다.
@dataclass(frozen=True)
class ShelfSpec:
    """곤돌라 한 면(단면 진열대 한 대)의 치수."""

    # 외형 ---------------------------------------------------
    width: float = 1.20          # [잠정] 폭 (Y). 마트 표준 1200, 편의점 900
    depth: float = 0.50          # [잠정] 바닥 데크 깊이 (X) = 한 면의 바닥 점유
    height: float = 1.80         # [잠정] 지주 높이 (Z). 1500/1800/2100 중 1800

    # 상단 선반 ----------------------------------------------
    shelf_depth: float = 0.40    # [잠정] 상단 선반 깊이. 바닥 데크보다 얕다
    board_t: float = 0.02        # [표준] 강판 선반 두께 (접힌 테두리 포함)

    # 바닥 데크 ----------------------------------------------
    deck_t: float = 0.03         # [표준] 바닥 데크 두께
    kick_setback: float = 0.04   # [표준] 걸레받이가 데크 앞면에서 들어간 거리

    # 지주 · 백판 ---------------------------------------------
    post_w: float = 0.03         # [표준] 지주 단면 폭 (Y)
    post_d: float = 0.05         # [표준] 지주 단면 깊이 (X)
    back_t: float = 0.02         # [표준] 백판 두께 (타공판)
    hole_pitch: float = 0.025    # [표준] 지주 브래킷 홀 피치. 선반 높이는 이 배수

    # 가격표 레일 --------------------------------------------
    # 선반 앞단에 걸리는 띠. 상품을 앞으로 끌어낼 때 걸리는 턱이 이것이다.
    rail_h: float = 0.04         # [표준] 레일 높이 (선반 윗면에서 아래로)
    rail_t: float = 0.01         # [표준] 레일 두께 (앞으로 튀어나옴)

    # 선반 단 ------------------------------------------------
    # Level_00 = 바닥 데크. 그 위로 상단 선반이 top_z 까지 등간격으로
    # 올라가고, 각 높이는 hole_pitch 배수로 스냅된다. 실측하면 개별 높이로
    # 바꾼다 (level_heights 를 리스트 반환으로).
    n_levels: int = 5            # [잠정] 단 수 (바닥 데크 포함)
    bottom_z: float = 0.15       # [잠정] 바닥 데크 윗면 높이
    top_z: float = 1.50          # [잠정] 최상단 선반 윗면 높이 (손이 닿는 한계)

    # 상품 배치용 슬롯 ---------------------------------------
    slots_per_level: int = 6     # [설계] 한 단을 몇 칸으로 나눌지
    slot_margin_y: float = 0.03  # [설계] 슬롯 좌우 여유
    slot_front_gap: float = 0.03 # [설계] 상품 앞면과 선반 앞단 사이 거리

    # 파생값 -------------------------------------------------
    def inner_width(self) -> float:
        """지주 사이 폭. 선반판·백판·데크가 이 폭이다."""
        return self.width - 2 * self.post_w

    def snap(self, z: float) -> float:
        """높이를 지주 홀 피치 배수로 맞춘다."""
        return round(z / self.hole_pitch) * self.hole_pitch

    def level_heights(self) -> list[float]:
        """각 단 윗면의 z. 상품은 이 높이 위에 놓인다. [0] 은 바닥 데크."""
        if self.n_levels == 1:
            return [self.bottom_z]
        step = (self.top_z - self.bottom_z) / (self.n_levels - 1)
        return [self.snap(self.bottom_z + step * i) for i in range(self.n_levels)]

    def level_fronts(self) -> list[float]:
        """각 단 앞단의 x (선반 로컬). 바닥 데크는 0, 상단 선반은 안으로 들어간다."""
        upper = self.depth - self.back_t - self.shelf_depth
        return [0.0] + [upper] * (self.n_levels - 1)

    def level_thickness(self) -> list[float]:
        return [self.deck_t] + [self.board_t] * (self.n_levels - 1)

    def level_clearances(self) -> list[float]:
        """단마다의 수직 여유. 상품 높이 상한이자 팔 진입 가능 높이.

        **단마다 다르다.** 중간 단은 위 선반판까지가 한계지만, 최상단은
        위에 판이 없어서 지주 높이가 한계다. 실제 진열대는 최상단이 뚫려
        있어 지주보다 큰 상품도 세울 수 있지만, 그러면 조명·천장·팔 도달
        문제가 되므로 보수적으로 지주 높이를 상한으로 둔다.
        """
        hs = self.level_heights()
        ts = self.level_thickness()
        out: list[float] = []
        for i, z in enumerate(hs):
            top = hs[i + 1] - ts[i + 1] if i + 1 < len(hs) else self.height
            out.append(top - z)
        return out

    def min_clearance(self) -> float:
        """가장 낮은 단 여유. 모든 단에 들어가는 상품의 높이 상한."""
        return min(self.level_clearances())

    def slot_width(self) -> float:
        """슬롯 하나의 폭. 상품 폭 상한."""
        return self.inner_width() / self.slots_per_level - 2 * self.slot_margin_y


# ─────────────────────────────────────────────────────────────
# 매장 레이아웃
# ─────────────────────────────────────────────────────────────
# 대형마트(이마트·홈플러스급) 식품 매장의 한 층 일부를 자른 모양:
#
#   y ↑
#     ┌──────────────────────────────────────────────────────┐ ← 벽
#     │                    뒤 주통로 (넓음)                     │
#     │  ┌──┐    ┌────┐    ┌────┐    ┌────┐          ┌──┐     │
#     │  │벽│부통로│엔드캡│부통로│엔드캡│부통로│엔드캡│ ...  부통로│벽│     │
#     │  │면│  0  ├────┤  1  ├────┤  2  ├────┤          │면│     │
#     │  │진│     │양면 │     │양면 │     │ ▣  │   ▣ 기둥  │진│     │
#     │  │열│     │곤돌라│     │곤돌라│     │(기둥│  자리는  │열│     │
#     │  │대│     │    │     │    │     │ 자리)│  비운다  │대│     │
#     │  │  │     ├────┤     ├────┤     ├────┤          │  │     │
#     │  └──┘     │엔드캡│     │엔드캡│     │엔드캡│          └──┘     │
#     │           └────┘     └────┘     └────┘                │
#     │                    앞 주통로 (넓음)                     │
#     └──────────────────────────────────────────────────────┘ → x
#
# - 부통로: 진열대 사이. 카트 두 대가 교행할 폭.
# - 주통로: 부통로 양 끝을 잇는 넓은 통로. AMR 이 회전하는 곳.
# - 양면 곤돌라: 등을 맞댄 두 면. 부통로 사이에 선다.
# - 엔드캡: 곤돌라 열 양 끝, 주통로를 보는 단면 진열대. 행사 상품 자리.
# - 벽면 진열대: 벽에 붙은 단면 진열대. 곤돌라보다 높다.
# - 기둥: 건물 구조 그리드. 설계 관행대로 곤돌라 열 위에 오게 맞추고,
#   기둥이 떨어진 자리의 진열대는 비운다 (실제 매장이 그렇게 한다).
#
# 규모와 실측의 관계: 매장 전체는 대형마트 **규모**를 재현하고, 실측은
# 부통로 1~2 개만 한다 (docs/SURVEY.md). 곤돌라 규격·통로 폭은 한 매장 안에서
# 반복되므로 한두 통로의 값이 전체에 적용된다.
@dataclass(frozen=True)
class StoreSpec:
    """대형마트 식품 매장 한 층의 일부."""

    # 통로 ---------------------------------------------------
    aisle_width: float = 2.20    # [잠정] 부통로 폭 (대형마트 2.0~2.4, 카트 교행) ← AMR 검증 핵심
    main_aisle_width: float = 3.50  # [잠정] 주통로 폭 (대형마트 3.0~4.0)
    n_aisles: int = 8            # [설계] 부통로 수
    shelves_per_run: int = 8     # [잠정] 곤돌라 한 열의 진열대 수 (8 × 1.2 = 9.6 m)

    # 건물 ---------------------------------------------------
    ceiling_h: float = 5.00      # [잠정] 천장 높이 (대형마트 노출 천장 4.5~6)
    wall_t: float = 0.20         # [설계] 벽 두께 (바닥 사각형 바깥에 붙는다)
    tile: float = 0.60           # [잠정] 바닥 타일 규격 — 실측 환산의 자(尺)

    # 기둥 그리드 --------------------------------------------
    # x 는 곤돌라 열 간격의 배수로 두어 기둥이 열 위에 오게 한다 (매장 설계 관행).
    # y 는 건물 스팬. 실측하면 그리드 대신 개별 좌표 리스트로 바꾼다.
    column_size: float = 0.60        # [잠정] 기둥 한 변 (정사각 단면)
    column_every_lines: int = 3      # [잠정] 곤돌라 열 몇 개마다 기둥 열이 오는가 (3 × 3.2 = 9.6 m)
    column_first_line: int = 0       # [설계] 첫 기둥 열 (0 = 왼쪽 벽면 진열대 선)
    column_pitch_y: float = 8.40     # [잠정] 기둥 y 간격 (건물 스팬)
    column_y0: float = 1.20          # [잠정] 첫 기둥 y (앞 주통로 안)

    # 조명 ---------------------------------------------------
    # 통로마다 천장에 라인 조명 하나. 시나리오 생성기가 세기를 흔든다.
    light_w: float = 0.30        # [설계] 라인 조명 폭
    light_intensity: float = 12000.0 # [설계] UsdLux RectLight intensity. 형광등 면휘도 ~1e4 cd/m² 급
    light_temp_k: float = 5000.0     # [설계] 색온도 (마트 백색 LED 4000~5700K)

    # 벽면 진열대 · 엔드캡 -----------------------------------
    wall_unit_height: float = 2.40   # [잠정] 벽면 진열대 지주 높이 (대형마트 2.1~2.4)
    wall_unit_top_z: float = 2.10    # [잠정] 벽면 진열대 최상단 선반 높이
    wall_unit_levels: int = 7        # [잠정]

    # 파생값 -------------------------------------------------
    def run_length(self, shelf: ShelfSpec) -> float:
        """곤돌라 한 열의 길이 (Y), 엔드캡 제외."""
        return shelf.width * self.shelves_per_run

    def endcap_spec(self, shelf: ShelfSpec) -> ShelfSpec:
        """엔드캡은 양면 곤돌라 폭(= 깊이 × 2)에 맞춘 단면 진열대."""
        return replace(shelf, width=2 * shelf.depth, slots_per_level=5)

    def wall_unit_spec(self, shelf: ShelfSpec) -> ShelfSpec:
        return replace(
            shelf,
            height=self.wall_unit_height,
            top_z=self.wall_unit_top_z,
            n_levels=self.wall_unit_levels,
        )

    def footprint(self, shelf: ShelfSpec) -> tuple[float, float]:
        """바닥 사각형 크기 (Lx, Ly). 벽은 이 바깥에 붙는다."""
        lx = self.n_aisles * (self.aisle_width + 2 * shelf.depth)
        endcap_d = self.endcap_spec(shelf).depth
        ly = 2 * self.main_aisle_width + 2 * endcap_d + self.run_length(shelf)
        return lx, ly

    def aisle_x_range(self, aisle: int, shelf: ShelfSpec) -> tuple[float, float]:
        """부통로 `aisle` 의 x 구간 (양쪽 진열대 앞면 사이)."""
        pitch = self.aisle_width + 2 * shelf.depth
        x0 = aisle * pitch + shelf.depth
        return x0, x0 + self.aisle_width

    def run_y_range(self, shelf: ShelfSpec) -> tuple[float, float]:
        """곤돌라 열(엔드캡 제외)이 차지하는 y 구간."""
        y0 = self.main_aisle_width + self.endcap_spec(shelf).depth
        return y0, y0 + self.run_length(shelf)

    def main_aisle_y_ranges(self, shelf: ShelfSpec) -> list[tuple[float, float]]:
        """앞·뒤 주통로의 y 구간."""
        _, ly = self.footprint(shelf)
        return [(0.0, self.main_aisle_width), (ly - self.main_aisle_width, ly)]

    def line_center_x(self, line: int, shelf: ShelfSpec) -> float:
        """진열대 열 `line` 의 중심 x. 0 = 왼쪽 벽면, 1..n-1 = 양면 곤돌라, n = 오른쪽 벽면."""
        pitch = self.aisle_width + 2 * shelf.depth
        if line == 0:
            return shelf.depth / 2
        if line == self.n_aisles:
            return self.n_aisles * pitch - shelf.depth / 2
        return line * pitch

    def columns(self, shelf: ShelfSpec) -> list[tuple[float, float]]:
        """기둥 중심 (x, y) 목록. 그리드를 진열대 열 위에 정렬해서 만든다."""
        lx, ly = self.footprint(shelf)
        c = self.column_size / 2
        xs = []
        for line in range(self.column_first_line, self.n_aisles + 1, self.column_every_lines):
            x = self.line_center_x(line, shelf)
            xs.append(min(max(x, c), lx - c))        # 벽면 선은 벽 안쪽으로 밀어 넣는다
        ys = []
        y = self.column_y0
        while y < ly - c:
            ys.append(y)
            y += self.column_pitch_y
        return [(x, y) for x in xs for y in ys]


# ─────────────────────────────────────────────────────────────
# 로봇 (통과 가능성 검증용 — 형상은 나중에)
# ─────────────────────────────────────────────────────────────
# AMR 은 Isaac Sim 기본 에셋 Carter v1 을 쓴다 (차동구동 + 뒤 캐스터, 1·3인칭 카메라 내장).
# 본체 치수는 에셋 AABB 를 Isaac 에서 읽은 값. 팔은 아직 없다 — 마운트 높이만 잡아 둔다.
@dataclass(frozen=True)
class RobotSpec:
    base_w: float = 0.63         # [표준] Carter v1 에셋 AABB 폭 (바퀴 포함)
    base_l: float = 0.67         # [표준] Carter v1 에셋 AABB 길이
    base_h: float = 0.35         # [설계] 팔 마운트 바닥 높이

    # Isaac 에셋 -------------------------------------------------
    asset: str = "/Isaac/Robots/NVIDIA/Carter/carter_v1.usd"   # [표준] get_assets_root_path() 뒤에 붙인다
    wheel_joints: tuple[str, str] = ("left_wheel", "right_wheel")  # [표준] 에셋 조인트 이름
    wheel_radius: float = 0.24   # [표준] 에셋 바퀴 AABB 0.482 / 2
    wheel_base: float = 0.53     # [표준] 좌우 바퀴 중심 간격 (에셋 y ±0.266)
    spawn_z: float = 0.255       # [표준] 에셋 원점이 바퀴 축 높이라 바닥에서 이만큼 띄워 놓는다 (AABB 밑 −0.251)

    # 주행 제한 ---------------------------------------------------
    v_max: float = 0.8           # [설계] 직진 최고 속도 m/s (매장 안 보행자 옆)
    w_max: float = 1.0           # [설계] 회전 최고 각속도 rad/s

    # 양팔 설계 / 한 팔 구현 — 마운트 자리는 처음부터 둘 다 잡아둔다.
    arm_mount_dy: float = 0.12   # [설계] 중심선에서 왼쪽으로 팔 베이스까지 (상판 반폭 0.29 안, 베이스 반경 0.1)
    n_arms_built: int = 1        # [설계] 지금 실제로 붙이는 팔 수

    safety_margin: float = 0.15  # [설계] 장애물과 유지할 편측 여유 (내비게이션 inflation)

    # 팔 — Franka Panda 를 Carter 상판에 얹는다. 정차할 때 왼쪽(+Y)이 진열대를 본다.
    # 도달 모델: 어깨(panda_joint1 축) 를 중심으로 반경 arm_reach 인 구. tools/reach_study.py 로
    # 마운트 높이를 훑어 봤다 (docs/img/reach_study.png). 어깨 0.85 m 가 최적(90 %)이지만 Carter
    # 상판이 0.667 m 라 그 위에 바로 얹으면 어깨가 1.0 m — 바닥 데크(0단) 상품은 대부분 못 닿는다.
    # 리프트 없이 가는 대신 그 사실을 통계로 남긴다.
    arm_asset: str = "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"  # [표준]
    arm_base_dx: float = -0.02   # [설계] 팔 베이스의 본체 중심 기준 전후 위치. 베이스가 뒤로 15 cm 나오므로 바구니(-0.20 뒤)와 안 겹치게
    arm_base_dz: float = 0.667 + 0.01   # [표준] Carter 상판(에셋 chassis AABB 위 0.412 + spawn 0.255) + 틈 1 cm
    arm_shoulder_dz: float = 0.333      # [표준] Franka 베이스 → panda_joint1 축 (어깨) 높이
    arm_reach: float = 0.855     # [표준] Franka 도달 반경 (어깨 기준)
    gripper_max_w: float = 0.08  # [표준] Franka 핸드 최대 벌림. 파지 폭 상한 (여유 두고 0.075 까지 집는다)
    gripper_tcp_dz: float = 0.1034  # [표준] panda_hand → 손끝 사이(right_gripper 프레임)
    pick_standoff: float = 0.20  # [설계] 정차 시 본체 측면과 진열대 앞면 사이 거리
    grasp_min_z_above_shelf: float = 0.055  # [표준] 손 몸통 반높이 ~4 cm + 여유. 손끝이 선반 위 이만큼 위에 있어야 한다
    grasp_min_height: float = 0.07          # [설계] 옆에서 집을 수 있는 상품 최소 높이 (= 위 값 + 윗면 여유 1.5 cm)

    def arm_mount_z(self) -> float:
        """도달 구의 중심(어깨) 높이. scenario.pick_pose 가 쓴다."""
        return self.arm_base_dz + self.arm_shoulder_dz

    def graspable_width(self) -> float:
        """주문에 넣을 상품의 통로 방향 폭 상한. 벌림 8 cm 에서 양쪽 5 mm 여유 (IK 오차 ~4 mm)."""
        return self.gripper_max_w - 0.010

    def turn_radius(self) -> float:
        """제자리 회전 시 필요한 반경 (차동구동 가정)."""
        return ((self.base_w / 2) ** 2 + (self.base_l / 2) ** 2) ** 0.5

    def corridor_width(self) -> float:
        """직진 통과에 필요한 최소 통로 폭 (여유 포함)."""
        return self.base_w + 2 * self.safety_margin

    def turn_diameter(self) -> float:
        """제자리 회전에 필요한 최소 사각 구간 한 변 (여유 포함)."""
        return 2 * self.turn_radius() + 2 * self.safety_margin


# ─────────────────────────────────────────────────────────────
# 물리 재질
# ─────────────────────────────────────────────────────────────
# μ 는 실물에서 측정 불가능한 값이다. 하나로 고정하지 말고
# 스윕 축으로 쓴다 (docs/ 의 검증 계약 참고).
@dataclass(frozen=True)
class PhysicsSpec:
    mu_floor: float = 0.80       # [설계] 바닥-바퀴
    mu_shelf: float = 0.40       # [설계] 선반판-상품  ← 스윕 대상
    mu_gripper: float = 0.90     # [설계] 그리퍼-상품  ← 스윕 대상
    restitution: float = 0.01    # [설계] 거의 튀지 않게


# ─────────────────────────────────────────────────────────────
# 시나리오 — 트윈에서 '어느 날의 매장'으로 흔드는 정도
# ─────────────────────────────────────────────────────────────
# 전부 [설계]. 실측 매장을 며칠 관찰하면 빈 자리·넘어짐·오배치 빈도를 재서
# 바꾼다 (docs/SURVEY.md). 확률은 슬롯(한 칸의 앞뒤 열) 단위다.
@dataclass(frozen=True)
class ScenarioSpec:
    fill: float = 0.85           # [설계] 슬롯이 채워질 확률 → 빈 자리 15 %
    p_jitter: float = 0.30       # [설계] yaw 가 흔들린 슬롯 비율
    jitter_deg: float = 15.0     # [설계] yaw 흔들림 최대 (±)
    p_fallen: float = 0.02       # [설계] 맨 앞 상품이 앞으로 넘어진 슬롯 비율
    p_misplaced: float = 0.03    # [설계] 다른 품목군 상품이 잘못 놓인 슬롯 비율
    light_scale: tuple[float, float] = (0.6, 1.2)  # [설계] 통로 조명 세기 배율 범위
    p_light_off: float = 0.10    # [설계] 등 하나가 꺼져 있을 확률
    n_orders: int = 5            # [설계] 주문 건수
    lines_per_order: int = 4     # [설계] 주문 한 건의 품목 수


SHELF = ShelfSpec()
STORE = StoreSpec()
ROBOT = RobotSpec()
PHYSICS = PhysicsSpec()
SCENARIO = ScenarioSpec()
