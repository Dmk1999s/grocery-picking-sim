"""치수 단일 진실 공급원.

이 파일의 숫자 하나를 바꾸면 선반 형상·매장 레이아웃·상품 슬롯·검증이
전부 따라온다. USD 를 손으로 만지지 말고 항상 여기서 시작할 것.

값의 출처를 [ ] 로 표시한다:
  [실측]  실제 마트에서 잰 값        ← 디지털 트윈의 근거
  [잠정]  아직 안 잰 값. 실측 후 교체 ← docs/SURVEY.md 참고
  [표준]  규격·문헌에서 온 값
  [설계]  우리가 정한 값

단위는 전부 미터(m), 킬로그램(kg). USD 스테이지도 metersPerUnit=1.0 이다.
"""

from dataclasses import dataclass, field

# ─────────────────────────────────────────────────────────────
# 좌표계 규약
# ─────────────────────────────────────────────────────────────
# 월드: Z-up (Isaac Sim 기본).
#
# 선반 로컬 프레임 (ShelfSpec 이 만드는 Xform 기준):
#   원점 = 선반 바닥면의 통로쪽 앞면 중앙
#   +X = 선반 안쪽(깊이 방향)      ← 로봇 팔이 진입하는 방향
#   +Y = 선반 폭 방향(정면에서 봤을 때 오른쪽)
#   +Z = 위
#
# 논문(arXiv:2409.15465) Fig.2a 의 item frame 과 축 방향을 맞췄다.
# 파지 계획을 yz 평면에 투영한다는 것은 곧 "통로에서 정면으로 본 실루엣"이다.

UP_AXIS = "Z"
METERS_PER_UNIT = 1.0


# ─────────────────────────────────────────────────────────────
# 선반 (진열대)
# ─────────────────────────────────────────────────────────────
@dataclass(frozen=True)
class ShelfSpec:
    """진열대 한 대(gondola 한 면)의 치수."""

    # 외형 ---------------------------------------------------
    width: float = 1.20          # [잠정] 폭 (Y)
    depth: float = 0.45          # [잠정] 깊이 (X)
    height: float = 1.80         # [잠정] 전체 높이 (Z)

    # 판재 두께 ----------------------------------------------
    board_t: float = 0.02        # [설계] 선반판 두께
    side_t: float = 0.03         # [설계] 측판 두께
    back_t: float = 0.02         # [설계] 뒷판 두께

    # 선반 단 ------------------------------------------------
    # 최하단 높이부터 등간격으로 n_levels 장. 실측하면 개별 높이로 바꾼다.
    n_levels: int = 5            # [잠정] 단 수
    bottom_z: float = 0.15       # [잠정] 최하단 선반판 윗면 높이
    top_z: float = 1.60          # [잠정] 최상단 선반판 윗면 높이

    # 상품 배치용 슬롯 ---------------------------------------
    slots_per_level: int = 6     # [설계] 한 단을 몇 칸으로 나눌지
    slot_margin_y: float = 0.03  # [설계] 슬롯 좌우 여유
    slot_front_gap: float = 0.05 # [설계] 상품 앞면과 선반 앞단 사이 거리

    def level_heights(self) -> list[float]:
        """각 선반판 윗면의 z. 상품은 이 높이 위에 놓인다."""
        if self.n_levels == 1:
            return [self.bottom_z]
        step = (self.top_z - self.bottom_z) / (self.n_levels - 1)
        return [self.bottom_z + step * i for i in range(self.n_levels)]

    def level_clearances(self) -> list[float]:
        """단마다의 수직 여유. 상품 높이 상한이자 팔 진입 가능 높이.

        **단마다 다르다.** 중간 단은 위 선반판까지가 한계지만, 최상단은
        위에 판이 없어서 선반 전체 높이가 한계다. 이걸 하나의 값으로
        쓰면 최상단에 들어가지 않는 상품을 배치하게 된다.

        실제 진열대가 최상단이 뚫려 있어서 키 큰 상품을 세울 수 있다면
        `height` 를 그만큼 올려 잡는다 — 여기서 예외 처리하지 말 것.
        """
        hs = self.level_heights()
        out: list[float] = []
        for i, z in enumerate(hs):
            top = hs[i + 1] - self.board_t if i + 1 < len(hs) else self.height
            out.append(top - z)
        return out

    def min_clearance(self) -> float:
        """가장 낮은 단 여유. 모든 단에 들어가는 상품의 높이 상한."""
        return min(self.level_clearances())

    def slot_width(self) -> float:
        """슬롯 하나의 폭. 상품 폭 상한."""
        inner_w = self.width - 2 * self.side_t
        return inner_w / self.slots_per_level - 2 * self.slot_margin_y


# ─────────────────────────────────────────────────────────────
# 매장 레이아웃
# ─────────────────────────────────────────────────────────────
@dataclass(frozen=True)
class StoreSpec:
    """매장 한 구역. 우선은 통로 1~2 개만 만든다."""

    aisle_width: float = 1.80    # [잠정] 통로 폭 ← AMR 통과 검증의 핵심 값
    n_aisles: int = 1            # [설계] 통로 수
    shelves_per_run: int = 4     # [설계] 한 줄에 놓을 진열대 수
    ceiling_h: float = 3.20      # [잠정] 천장 높이
    wall_margin: float = 1.50    # [설계] 진열대 열 끝과 벽 사이 여유

    def run_length(self, shelf: ShelfSpec) -> float:
        """진열대 한 줄의 전체 길이 (Y 방향)."""
        return shelf.width * self.shelves_per_run


# ─────────────────────────────────────────────────────────────
# 로봇 (통과 가능성 검증용 — 형상은 나중에)
# ─────────────────────────────────────────────────────────────
@dataclass(frozen=True)
class RobotSpec:
    base_w: float = 0.60         # [설계] AMR 폭
    base_l: float = 0.70         # [설계] AMR 길이
    base_h: float = 0.35         # [설계] AMR 높이 (팔 마운트 바닥)

    # 양팔 설계 / 한 팔 구현 — 마운트 자리는 처음부터 둘 다 잡아둔다.
    arm_mount_dy: float = 0.18   # [설계] 중심선에서 좌우 팔 마운트까지
    n_arms_built: int = 1        # [설계] 지금 실제로 붙이는 팔 수

    def turn_radius(self) -> float:
        """제자리 회전 시 필요한 반경 (차동구동 가정)."""
        return ((self.base_w / 2) ** 2 + (self.base_l / 2) ** 2) ** 0.5


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


SHELF = ShelfSpec()
STORE = StoreSpec()
ROBOT = RobotSpec()
PHYSICS = PhysicsSpec()
