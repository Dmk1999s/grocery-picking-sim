"""Carter 위의 Franka — 정차 자세에서 상품을 집어 바구니에 넣는다 (drive_isaac 이 --arm 으로 쓴다).

구조:
  - Franka 는 베이스가 월드에 고정된 별도 관절체다. 매 물리 스텝 Carter 자세에서 계산한
    마운트 자세로 베이스를 옮긴다 (기구학적으로 태운다). 정차 중에는 베이스가 서 있으므로
    고정 베이스 팔과 똑같이 동작한다. 주행 중 상품은 그리퍼가 아니라 바구니에 있다
  - 바구니는 Carter 상판 뒤쪽에 붙인 얇은 상자 (chassis 강체의 자식 콜라이더)
  - IK 는 Lula (isaacsim.robot_motion.motion_generation). 목표 프레임은 손끝 사이(right_gripper)

파지 순서 (전부 직교 좌표 직선 보간 + 매 스텝 IK):
  tuck → 프리그래스프(상품 앞 25 cm, 벌림) → 그래스프 지점(앞면에서 3.5 cm 안) → 닫기 →
  4 cm 들기 → 뒤로 빼기 → 바구니 위 → 벌리기 → tuck

손가락은 통로 방향으로 닫힌다: 맨 앞 상품 뒤에는 다음 상품이 1 cm 뒤에 붙어 있어
깊이 방향으로는 손가락이 들어갈 자리가 없다. 그래서 통로 방향 폭 ≤ 7.5 cm 인 상품만
주문에 들어온다 (scenario.py 의 graspable).

측정: 들렸는가(들기 뒤 z 상승 ≥ 3 cm), 빼낸 뒤에도 잡고 있는가(손끝↔상품 중심 ≤ 6 cm),
바구니에 들어갔는가, 같은 진열대의 다른 상품이 2 cm 넘게 움직였는가, IK 실패 단계.
"""

from __future__ import annotations

import math
import os

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade
from scipy.spatial.transform import Rotation, Slerp

from scene.constants import ROBOT, RobotSpec

# Franka 접힘 자세 [설계]. 손이 베이스 위로 올라와 본체 바닥 사각형 밖으로 안 나간다 (FK 로 확인해 로그에 남긴다)
TUCK = np.array([0.0, -1.70, 0.0, -2.90, 0.0, 1.25, 0.785])
FINGER_OPEN, FINGER_CLOSED = 0.04, 0.0
FINGER_FRICTION = float(os.environ.get("ARM_FRICTION", "1.0"))    # [설계] 손가락 패드 마찰 (고무)
FINGER_MASS = float(os.environ.get("ARM_FINGER_MASS", "0"))        # [설계] 0 이면 에셋값(14 g). 상품(0.4 kg)과 질량비가 커서 접촉 솔버가 흔들린다
SOLVER_ITERS = int(os.environ.get("ARM_SOLVER_ITERS", "0"))        # [설계] 0 이면 기본
CARRY_SLOW = float(os.environ.get("ARM_CARRY_SLOW", "1.0"))        # [설계] 카레 구간 시간 배율
ARM_GAIN_SCALE = 4.0                   # [설계] 관절 드라이브 강성 배율 (감쇠는 √배)
GRIP_STIFFNESS, GRIP_DAMPING, GRIP_MAX_FORCE = 5000.0, 200.0, 70.0   # [표준] Franka Hand 연속 파지력 70 N. 강성은 3 cm 오차에서 포화하게
BIN_SIZE = (0.18, 0.44, 0.20)          # [설계] 상판 뒤쪽 바구니 (x 전후, y 좌우, z 높이), 벽 1 cm. 15 cm 벽은 병이 튀어 넘었다
BIN_DX = -0.29                         # [설계] 본체 중심 기준 바구니 중심 x → x ∈ [-0.38, -0.20], 상판 뒤끝 -0.385 안, Franka 베이스(-0.174) 앞


def quat_wxyz_from_axes(x, y, z) -> np.ndarray:
    q = Rotation.from_matrix(np.c_[x, y, z]).as_quat()      # xyzw
    return np.r_[q[3], q[:3]]


def yaw_quat(yaw: float) -> np.ndarray:
    return np.array([math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)])


class Arm:
    def __init__(self, stage: Usd.Stage, app, spec: RobotSpec = ROBOT, robot_path: str = "/World/Robot",
                 init_pose: tuple[float, float, float] = (0.0, 0.0, 0.0)):
        """init_pose = (x, y, yaw) 본체 자세. 생성 위치가 진열대·벽 안이면 첫 스텝에 관절이 폭주하므로 꼭 준다."""
        from isaacsim.core.utils.extensions import enable_extension
        enable_extension("isaacsim.robot_motion.motion_generation")
        app.update()
        from isaacsim.core.experimental.prims import Articulation
        import isaacsim.core.experimental.utils.stage as stage_utils
        from isaacsim.robot_motion.motion_generation import LulaKinematicsSolver, interface_config_loader
        from isaacsim.storage.native import get_assets_root_path

        self.spec = spec
        self.stage = stage
        self.app = app
        stage_utils.add_reference_to_stage(usd_path=get_assets_root_path() + spec.arm_asset, path="/World/Arm")
        p0, q0 = self.base_pose(*init_pose)
        self.art = Articulation("/World/Arm", positions=[p0], orientations=[q0], reset_xform_op_properties=True)
        # 팔은 월드 고정 관절체를 기구학적으로 태운 것이라 Carter 와의 접촉은 물리적으로 의미가 없고
        # 닿기만 하면 본체가 밀린다 (처음에 바구니 벽과 베이스가 겹쳐 0.15 m/s 로 흘러갔다). 둘 사이 충돌을 끈다
        fp = UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath("/World/Arm"))
        fp.CreateFilteredPairsRel().AddTarget(robot_path)
        self._gripper_setup()
        self.ik = LulaKinematicsSolver(**interface_config_loader.load_supported_lula_kinematics_solver_config("Franka"))
        self._Articulation = Articulation
        self.arm_idx = None
        self.finger_idx = None
        self.base = (0.0, 0.0, 0.0)
        self.frozen = False          # 파지 중에는 베이스를 고정한다 (본체가 몇 mm 흘러도 팔이 따라 튀지 않게)
        self._bin(robot_path)

    def _gripper_setup(self) -> None:
        """에셋 그대로면 못 잡는다: finger_joint2 에는 드라이브가 없고 joint1 은 최대 7 N.
        실제 Franka Hand 사양(연속 70 N)으로 양쪽에 위치 드라이브를 넣고, 손가락 패드에 고무 마찰(μ 1.0)을 준다."""
        for j in ("panda_finger_joint1", "panda_finger_joint2"):
            prim = self.stage.GetPrimAtPath(f"/World/Arm/panda_hand/{j}")
            d = UsdPhysics.DriveAPI.Apply(prim, "linear")
            d.CreateTypeAttr("force")
            d.CreateStiffnessAttr(GRIP_STIFFNESS)
            d.CreateDampingAttr(GRIP_DAMPING)
            d.CreateMaxForceAttr(GRIP_MAX_FORCE)
        mat = UsdShade.Material.Define(self.stage, "/World/Arm/Looks/FingerRubber")
        pm = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
        pm.CreateStaticFrictionAttr(FINGER_FRICTION)
        pm.CreateDynamicFrictionAttr(FINGER_FRICTION)
        pm.CreateRestitutionAttr(0.0)
        for f in ("panda_leftfinger", "panda_rightfinger"):
            prim = self.stage.GetPrimAtPath(f"/World/Arm/{f}")
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(mat, UsdShade.Tokens.weakerThanDescendants, "physics")

    # ── 준비 (play 뒤에 부른다)
    def start(self) -> None:
        names = list(self.ik.get_joint_names())
        self.arm_idx = self.art.get_dof_indices(names).numpy().tolist()
        self.finger_idx = self.art.get_dof_indices(["panda_finger_joint1", "panda_finger_joint2"]).numpy().tolist()
        # 관절 드라이브를 세게: 에셋 기본값으론 팔을 뻗었을 때 손끝이 1.5 cm 처져 6 mm 여유의 파지를 놓친다
        st, dp = self.art.get_dof_gains()
        st, dp = st.numpy()[0][self.arm_idx], dp.numpy()[0][self.arm_idx]
        self.asset_gains = (st.tolist(), dp.tolist())
        self.art.set_dof_gains(stiffnesses=(st * ARM_GAIN_SCALE).tolist(), dampings=(dp * ARM_GAIN_SCALE ** 0.5).tolist(), dof_indices=self.arm_idx)
        self.art.set_dof_position_targets(TUCK, dof_indices=self.arm_idx)
        self.art.set_dof_positions(TUCK, dof_indices=self.arm_idx)
        self.art.set_dof_velocities(np.zeros(len(self.arm_idx)), dof_indices=self.arm_idx)
        if FINGER_MASS > 0:
            li = self.art.get_link_indices(["panda_leftfinger", "panda_rightfinger"]).numpy().tolist()
            self.art.set_link_masses([FINGER_MASS, FINGER_MASS], link_indices=li)
        if SOLVER_ITERS > 0:
            from pxr import PhysxSchema
            for prim in self.stage.Traverse():
                if prim.IsA(UsdPhysics.Scene):
                    ps = PhysxSchema.PhysxSceneAPI.Apply(prim)
                    ps.CreateMinPositionIterationCountAttr(SOLVER_ITERS)
                    ps.CreateMaxPositionIterationCountAttr(max(SOLVER_ITERS, 4))
        self.gripper(FINGER_OPEN)
        pos, _ = self.ik.compute_forward_kinematics("right_gripper", TUCK)
        self.tuck_tcp = [round(float(v), 3) for v in pos]      # 베이스 기준. 본체 사각형 안인지 로그로 확인

    def _bin(self, robot_path: str) -> None:
        """Carter 상판 뒤에 바구니. chassis_link 의 자식이라 강체에 붙어 같이 움직인다."""
        bx, by, bz = BIN_SIZE
        top = self.spec.arm_base_dz - self.spec.spawn_z - 0.01       # chassis 로컬에서 상판 높이
        root = UsdGeom.Xform.Define(self.stage, f"{robot_path}/chassis_link/Bin")
        t = 0.01
        parts = {
            "floor": ((bx, by, t), (BIN_DX, 0.0, top + t / 2)),
            "wall_front": ((t, by, bz), (BIN_DX + bx / 2 - t / 2, 0.0, top + bz / 2)),
            "wall_back": ((t, by, bz), (BIN_DX - bx / 2 + t / 2, 0.0, top + bz / 2)),
            "wall_left": ((bx, t, bz), (BIN_DX, by / 2 - t / 2, top + bz / 2)),
            "wall_right": ((bx, t, bz), (BIN_DX, -by / 2 + t / 2, top + bz / 2)),
        }
        for name, (size, center) in parts.items():
            cube = UsdGeom.Cube.Define(self.stage, f"{root.GetPath()}/{name}")
            cube.CreateSizeAttr(1.0)
            xf = UsdGeom.Xformable(cube)
            xf.AddTranslateOp().Set(Gf.Vec3d(*center))
            xf.AddScaleOp().Set(Gf.Vec3f(*size))
            UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
            cube.CreateDisplayColorAttr([Gf.Vec3f(0.25, 0.25, 0.28)])
        self.bin_local = (BIN_DX, 0.0, top)     # chassis 로컬 (= 본체 프레임, 원점은 바퀴 축 높이)

    # ── Carter 를 따라간다
    def base_pose(self, x: float, y: float, yaw: float) -> tuple[np.ndarray, np.ndarray]:
        s = self.spec
        c, sn = math.cos(yaw), math.sin(yaw)
        dx, dy = s.arm_base_dx, s.arm_mount_dy
        return np.array([x + c * dx - sn * dy, y + sn * dx + c * dy, s.arm_base_dz]), yaw_quat(yaw)

    def follow(self, x: float, y: float, yaw: float) -> None:
        if self.frozen:
            return
        self.base = (x, y, yaw)
        p, q = self.base_pose(x, y, yaw)
        self.art.set_world_poses(positions=[p], orientations=[q])
        self.ik.set_robot_base_pose(p, q)

    def gripper(self, opening: float) -> None:
        self.art.set_dof_position_targets([opening, opening], dof_indices=self.finger_idx)

    def joints(self) -> np.ndarray:
        return self.art.get_dof_positions().numpy()[0][self.arm_idx]

    def tcp(self) -> np.ndarray:
        pos, _ = self.ik.compute_forward_kinematics("right_gripper", self.joints())
        return np.asarray(pos)

    def tcp_pose(self) -> tuple[np.ndarray, np.ndarray]:
        pos, rot = self.ik.compute_forward_kinematics("right_gripper", self.joints())
        q = Rotation.from_matrix(np.asarray(rot)).as_quat()
        return np.asarray(pos), np.r_[q[3], q[:3]]

    def solve(self, pos, quat, warm=None) -> tuple[np.ndarray | None, float]:
        """IK. (관절, 위치 오차 m). 실패 시 (None, 오차). 빡빡한 자세 허용으로 먼저, 안 되면 손 기울기를 0.15 rad 까지 풀어 재시도."""
        warm = self.joints() if warm is None else warm
        best = None
        for otol in (0.02, 0.15):
            sol, ok = self.ik.compute_inverse_kinematics("right_gripper", np.asarray(pos), np.asarray(quat), warm_start=warm,
                                                         position_tolerance=0.002, orientation_tolerance=otol)
            fk, _ = self.ik.compute_forward_kinematics("right_gripper", sol)
            err = float(np.linalg.norm(np.asarray(fk) - np.asarray(pos)))
            if best is None or err < best[1]:
                best = (sol, err)
            if ok or err < 0.01:
                return sol, err
        return None, best[1]

    def move_line(self, p0, p1, quat, seconds: float, tick, dt: float, quat0=None) -> tuple[bool, float]:
        """TCP 를 p0 → p1 직선으로. 매 스텝 IK. quat0 를 주면 손 방향도 quat0 → quat 로 slerp. (성공, 최대 IK 오차)."""
        n = max(1, int(seconds / dt))
        worst = 0.0
        slerp = None
        if quat0 is not None:
            q0, q1 = np.asarray(quat0), np.asarray(quat)
            slerp = Slerp([0.0, 1.0], Rotation.from_quat([np.r_[q0[1:], q0[0]], np.r_[q1[1:], q1[0]]]))
        for i in range(1, n + 1):
            p = np.asarray(p0) + (np.asarray(p1) - np.asarray(p0)) * i / n
            if slerp is not None:
                qx = slerp(i / n).as_quat()
                quat = np.r_[qx[3], qx[:3]]
            sol, err = self.solve(p, quat)
            worst = max(worst, err)
            if sol is None:
                return False, worst
            self.art.set_dof_position_targets(sol, dof_indices=self.arm_idx)
            tick()
        return True, worst

    def move_line_pos(self, p0, p1, seconds: float, tick, dt: float) -> bool:
        """위치만 맞추는 직선 이동 (손 방향 자유). 카레 구간용."""
        n = max(1, int(seconds / dt))
        for i in range(1, n + 1):
            p = np.asarray(p0) + (np.asarray(p1) - np.asarray(p0)) * i / n
            sol, ok = self.ik.compute_inverse_kinematics("right_gripper", p, None, warm_start=self.joints(), position_tolerance=0.003)
            fk, _ = self.ik.compute_forward_kinematics("right_gripper", sol)
            if not ok and np.linalg.norm(np.asarray(fk) - p) > 0.02:
                return False
            self.art.set_dof_position_targets(sol, dof_indices=self.arm_idx)
            tick()
        return True

    def hold(self, seconds: float, tick, dt: float) -> None:
        for _ in range(int(seconds / dt)):
            tick()

    def go_tuck(self, seconds: float, tick, dt: float) -> None:
        q0 = self.joints()
        n = int(seconds / dt)
        for i in range(1, n + 1):
            self.art.set_dof_position_targets(q0 + (TUCK - q0) * i / n, dof_indices=self.arm_idx)
            tick()

    # ── 한 번 집기
    def pick(self, line: dict, item_poses, tick, dt: float, on_event=None) -> dict:
        """line: scenario JSON 의 주문 품목. item_poses(): {prim: (x,y,z)} 같은 진열대 상품 전부의 현재 위치."""
        try:
            return self._pick(line, item_poses, tick, dt, on_event)
        finally:
            self.frozen = False

    def _pick(self, line: dict, item_poses, tick, dt: float, on_event=None) -> dict:
        from isaacsim.core.experimental.prims import RigidPrim
        s = self.spec
        n = np.array(line["approach_dir"], dtype=float)           # 통로 → 진열대 (안쪽)
        _, _, yaw = self.base
        fwd = np.array([math.cos(yaw), math.sin(yaw), 0.0])       # 통로 방향 = 손가락이 닫히는 축
        lo, hi = np.array(line["item_aabb"][0]), np.array(line["item_aabb"][1])
        c = (lo + hi) / 2
        ext_n = float(abs(np.dot(hi - lo, n)))                    # 진열대 깊이 방향 크기
        front = c - n * ext_n / 2
        # 손끝(TCP)을 상품 중심보다 1 cm 뒤에: 패드(길이 ~2 cm)의 가운데가 중심에 온다. 캔처럼 둥근 것을
        # 앞쪽 현(弦)에서 잡으면 빗면이 상품을 밀어낸다. 뒤 상품(1 cm 간격)에 손끝이 닿지 않게 상한을 둔다
        grasp = front + n * min(ext_n / 2 + 0.01, ext_n - 0.01)
        # 높이: 상품 중심이되 선반 위 grasp_min_z_above_shelf 이상 (손 몸통이 선반에 닿지 않게), 윗면 1.5 cm 아래까지
        grasp[2] = min(max(c[2], lo[2] + s.grasp_min_z_above_shelf), hi[2] - 0.015)
        pre = grasp - n * 0.25
        quat = quat_wxyz_from_axes(np.cross(fwd, n), fwd, n)       # 손 z = 접근, 손 y = 손가락 축
        quat_ref = [quat]                                          # step_phase 가 쓰는 현재 손 방향 (카레 때 아래로 바꾼다)

        before = item_poses()
        item = RigidPrim(line["prim"])
        self.frozen = not os.environ.get("ARM_NO_FREEZE")   # 정차 중 본체 미세 이동을 팔에 전달하지 않는다. 실제 로봇은 브레이크를 잡는다
        z0 = float(item.get_world_poses()[0].numpy()[0][2])
        res = {"phase": "start", "ik_err_max_m": 0.0, "lifted": False, "held_after_retract": False, "in_bin": False,
               "disturbed_neighbors": 0, "grasp_width_m": line["stop"].get("grasp_width_m")}

        def ev(name):
            if on_event:
                on_event(name)

        def step_phase(name, p0, p1, seconds):
            res["phase"] = name
            ok, err = self.move_line(p0, p1, quat_ref[0], seconds, tick, dt)
            res["ik_err_max_m"] = round(max(res["ik_err_max_m"], err), 4)
            return ok

        self.gripper(FINGER_OPEN)
        # tuck → 프리그래스프: 직교 좌표 직선 + 손 방향 slerp. pre 는 본체 왼쪽 모서리 위(진열대 앞면 앞 ~20 cm)라
        # tuck 손끝(본체 중심 위)에서 직선으로 가면 경로가 전부 본체 위 공간이다. 관절 보간은 중간에 선반 판을 쳤다
        pos0, q0 = self.tcp_pose()
        res["phase"] = "to_pre"
        ok, err = self.move_line(pos0, pre, quat, 1.8, tick, dt, quat0=q0)
        res["ik_err_max_m"] = round(max(res["ik_err_max_m"], err), 4)
        self.hold(0.3, tick, dt)
        if not ok or np.linalg.norm(self.tcp() - pre) > 0.03:
            # 직선 경로가 안 풀리거나(낮은 단) 매 스텝 IK 해가 튀어 팔이 못 따라온 경우(높은 단): 관절 공간으로 한 번에
            res["to_pre_fallback"] = True
            sol, err = self.solve(pre, quat, warm=TUCK)
            res["ik_err_max_m"] = round(max(res["ik_err_max_m"], err), 4)
            if sol is None:
                res["phase"] = "to_pre_ik_fail"
                self.go_tuck(1.5, tick, dt)
                return res
            q_now = self.joints()
            nn = int(2.0 / dt)
            for i in range(1, nn + 1):
                self.art.set_dof_position_targets(q_now + (sol - q_now) * i / nn, dof_indices=self.arm_idx)
                tick()
        self.hold(0.3, tick, dt)
        res["pre_err_m"] = round(float(np.linalg.norm(self.tcp() - pre)), 4)       # 팔이 실제로 따라왔나
        ev("pre")
        if not step_phase("approach", pre, grasp, 1.6):
            return res
        self.hold(0.4, tick, dt)
        res["grasp_err_m"] = round(float(np.linalg.norm(self.tcp() - grasp)), 4)
        ev("approach")
        res["phase"] = "close"
        self.gripper(FINGER_CLOSED)
        self.hold(0.7, tick, dt)
        fingers = self.art.get_dof_positions().numpy()[0][self.finger_idx]
        res["finger_gap_m"] = round(float(fingers.sum()), 4)          # 닫힌 뒤 손가락 사이 = 잡은 폭. 0 이면 헛잡음
        offset0 = item.get_world_poses()[0].numpy()[0] - self.tcp()
        ev("grasp")
        lift = grasp + np.array([0, 0, 0.04])
        if not step_phase("lift", grasp, lift, 0.6 * CARRY_SLOW):
            return res
        self.hold(0.2, tick, dt)
        ev("lift")
        z1 = float(item.get_world_poses()[0].numpy()[0][2])
        res["lifted"] = (z1 - z0) >= 0.03
        res["lift_m"] = round(z1 - z0, 4)
        out = pre + np.array([0, 0, 0.04])
        if not step_phase("retract", lift, out, 1.2 * CARRY_SLOW):
            return res
        self.hold(0.2, tick, dt)
        ev("retract")
        ip = item.get_world_poses()[0].numpy()[0]
        # 잡았을 때의 손끝↔상품 중심 오프셋이 유지되면 들고 있는 것 (상품 중심이 손끝과 떨어져 있어도 된다)
        slip = float(np.linalg.norm((ip - self.tcp()) - offset0))
        res["held_after_retract"] = slip <= 0.03
        res["slip_m"] = round(slip, 4)
        # 바구니 위로. 관절 공간 직선은 진열대·본체를 뚫고 가려다 막힌다 (처음엔 그래서 상품을 통로 바닥에 떨궜다).
        # 직교 좌표로: 통로 위로 올리고(수평 자세 유지) → 그 자리에서 손을 아래로 돌리고 → 바구니 위로 → 내린다
        bx, by, bz = self.bin_local
        x, y, yaw = self.base
        cy, sy = math.cos(yaw), math.sin(yaw)
        bin_c = np.array([x + cy * bx - sy * by, y + sy * bx + cy * by, s.spawn_z + bz])
        quat_down = quat_wxyz_from_axes(np.cross(fwd, [0, 0, -1.0]), fwd, [0, 0, -1.0])   # 손 아래, 손가락 축은 통로 방향
        high = np.array([out[0], out[1], max(out[2] + 0.10, bin_c[2] + 0.30)])
        over = np.array([bin_c[0], bin_c[1], bin_c[2] + 0.30])      # 베이스 위 +0.29: IK 가 확실히 풀리는 높이 (probe)
        drop = np.array([bin_c[0], bin_c[1], bin_c[2] + 0.30])      # 잡은 상품 반높이(≤ 10 cm)가 벽(20 cm) 위에 오게
        res["phase"] = "carry"
        trace = []
        # 1) 위로: 방향은 자유 (위치만 맞추는 IK). 수평 자세를 고집하면 어깨 위 구간에서 해가 없다
        ok = self.move_line_pos(out, high, 1.5 * CARRY_SLOW, tick, dt)
        trace.append(round(float(np.linalg.norm(self.tcp() - high)), 3))
        ev("high")

        # 2) 바구니 위로: 직선 + 손 방향 slerp(수평 → 아래). high 와 over 모두 본체 위 공간이라 사이에 장애물이 없다.
        #    관절 보간(TUCK 워밍 IK 해로)은 팔꿈치가 크게 돌아 선반을 쳤다
        if ok:
            _, qh = self.tcp_pose()
            # 천천히 (3 s): 1.5 s 로 돌리면 팔이 목표를 10~50 cm 뒤따라오며 잡은 물건을 튕겨냈다
            ok, err = self.move_line(high, over, quat_down, 3.0 * CARRY_SLOW, tick, dt, quat0=qh)
            res["ik_err_max_m"] = round(max(res["ik_err_max_m"], err), 4)
            self.hold(0.4, tick, dt)
            quat_ref[0] = quat_down
        trace.append(round(float(np.linalg.norm(self.tcp() - over)), 3))
        ev("over")
        # 3) 내리기
        ok = ok and step_phase("carry_down", over, drop, 0.8)
        res["carry_trace_err_m"] = trace
        self.hold(0.3, tick, dt)
        reached = float(np.linalg.norm(self.tcp() - drop))
        res["carry_err_m"] = round(reached, 4)
        ev("over_bin")
        if not ok or reached > 0.05:
            res["phase"] = "carry_not_reached" if ok else res["phase"] + "_ik_fail"
            self.gripper(FINGER_OPEN)                # 어디든 놓고 접는다 (기록엔 실패로 남는다)
            self.hold(0.5, tick, dt)
            self.go_tuck(1.5, tick, dt)
            return res
        res["phase"] = "release"
        self.gripper(FINGER_OPEN)
        self.hold(1.0, tick, dt)
        ip = item.get_world_poses()[0].numpy()[0]
        # 바구니 안? 본체 로컬로 변환
        rel = np.array([ip[0] - x, ip[1] - y])
        lx_ = cy * rel[0] + sy * rel[1]
        ly_ = -sy * rel[0] + cy * rel[1]
        res["in_bin"] = bool(abs(lx_ - bx) <= BIN_SIZE[0] / 2 and abs(ly_ - by) <= BIN_SIZE[1] / 2
                             and s.spawn_z + bz - 0.02 <= ip[2] <= s.spawn_z + bz + BIN_SIZE[2] + 0.15)
        res["item_final"] = [round(float(v), 4) for v in ip]
        res["phase"] = "tuck"
        self.go_tuck(1.5, tick, dt)
        after = item_poses()
        res["disturbed_neighbors"] = sum(1 for k, p in after.items() if k != line["prim"] and k in before
                                         and np.linalg.norm(np.asarray(p) - np.asarray(before[k])) > 0.02)
        res["phase"] = "done"
        res["success"] = bool(res["lifted"] and res["in_bin"])
        return res
