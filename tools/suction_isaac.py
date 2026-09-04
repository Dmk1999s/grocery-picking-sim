"""흡착 그리퍼 — Carter 위 Franka 팔 끝에 진공 컵을 달고 상품 앞면에 붙여 빼낸다.

평행 그리퍼(tools/arm_isaac.py)가 빼곡한 진열에서 못 쓴다는 결론(docs/LOG.md (14)) 이후의 대안이다.
옆 틈이 필요 없고 앞에서만 접근한다. 대신 앞면이 컵보다 크고, 평평하고(요철), 덜 기울고,
무게가 컵 한계 안이어야 한다 — 그 판정은 tools/suction_study.py 가 미리 계산한다.

물리: Isaac Sim 의 SurfaceGripper (IsaacSurfaceGripper 프림 + D6 어태치먼트 조인트).
컵이 상품 표면에 max_grip_distance 안으로 닿은 채 켜면 조인트가 붙고, 축 방향/전단 힘 한계를
넘으면 떨어진다. 진공의 유체역학은 모사하지 않는다 — 그 부분은 suction_study 의 기하 조건이 맡는다.

정직한 한계: 시뮬의 붙음/떨어짐은 힘 한계로만 결정된다. 실제로는 표면 거칠기·다공성(망사·골판지
가장자리)·먼지가 진공을 새게 하는데 그건 모사하지 않는다. 그래서 suction_study 의 필터가 곧
'현실이 반영되는 자리'다.
"""

from __future__ import annotations

import math
import os

import numpy as np
from pxr import Gf, Sdf, UsdGeom, UsdPhysics

from scene.constants import ROBOT, RobotSpec
from tools.arm_isaac import CARRY_SLOW, FINGER_OPEN, TUCK, Arm, quat_wxyz_from_axes

GRIP_DIST = 0.02        # [설계] 컵이 이 거리 안에 표면을 두면 붙는다 (SurfaceGripper max_grip_distance)
# 조인트 힘 한계 — **진공 씰의 물리적 한계가 아니다.** 실제 한계는 ⌀20 mm·−60 kPa 에서
# 축 방향 18.8 N, 전단 9.4 N(μ 0.5)이고, 그건 suction_study 의 무게 필터(0.48 kg)가 이미 강제한다.
# 여기 값을 그 물리값 근처(25/60 N)로 두면 PhysX 조인트가 보고하는 힘에 솔버 잡음·구속 반력이 섞여
# 0.1 kg 테니스공도 떨어졌다 (같은 궤적에서 300 N 이면 4/4). 그래서 '가짜 실패'를 없앨 만큼 높게 두고,
# 무엇이 붙을 수 있는지는 기하·무게 필터가 정하게 한다.
COAXIAL_LIMIT = float(os.environ.get("SUC_COAXIAL", "200"))
SHEAR_LIMIT = float(os.environ.get("SUC_SHEAR", "150"))
RETRY = 0.5             # [설계] 켠 채로 못 붙었을 때 다시 시도하는 간격 s
SWING_Z = float(os.environ.get("SUC_SWING_Z", "0.45"))   # [설계] 바구니 바닥 위 스윙 고도 m


class SuctionArm(Arm):
    def __init__(self, stage, app, spec: RobotSpec = ROBOT, robot_path: str = "/World/Robot",
                 init_pose: tuple[float, float, float] = (0.0, 0.0, 0.0)):
        super().__init__(stage, app, spec, robot_path, init_pose)
        self._build_cup()

    # ── 컵 + 어태치먼트 조인트 + SurfaceGripper (play 전에 만들어야 한다)
    def _build_cup(self) -> None:
        from isaacsim.core.utils.extensions import enable_extension
        enable_extension("isaacsim.robot.surface_gripper")
        self.app.update()
        from usd.schema.isaac import robot_schema

        s = self.spec
        self.cup_z = float(s.gripper_tcp_dz + s.suction_cup_len)   # 손끝(TCP) 기준 컵 끝까지
        cup = UsdGeom.Cylinder.Define(self.stage, "/World/Arm/panda_hand/SuctionCup")
        cup.CreateRadiusAttr(float(s.suction_cup_radius))
        cup.CreateHeightAttr(float(s.suction_cup_len))
        cup.CreateAxisAttr("Z")
        UsdGeom.Xformable(cup).AddTranslateOp().Set(Gf.Vec3d(0, 0, self.cup_z - s.suction_cup_len / 2))
        UsdPhysics.CollisionAPI.Apply(cup.GetPrim())      # panda_hand 강체의 자식 → 같은 강체
        cup.CreateDisplayColorAttr([Gf.Vec3f(0.12, 0.12, 0.14)])

        UsdGeom.Scope.Define(self.stage, "/World/Arm/SuctionJoints")
        j = UsdPhysics.Joint.Define(self.stage, "/World/Arm/SuctionJoints/D6Joint")
        jp = j.GetPrim()
        j.CreateBody0Rel().SetTargets(["/World/Arm/panda_hand"])
        j.CreateExcludeFromArticulationAttr(True)
        j.CreateJointEnabledAttr(True)
        j.CreateLocalPos0Attr(Gf.Vec3f(0, 0, self.cup_z))
        j.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
        for ax in ("transX", "transY", "transZ", "rotX", "rotY", "rotZ"):
            UsdPhysics.LimitAPI.Apply(jp, ax)
        # transX·transY 는 잠근다 (low > high = 잠금). 한계를 안 주면 자유 축이라 상품이 컵에서 옆으로 미끄러진다
        for ax in ("transX", "transY"):
            lim = UsdPhysics.LimitAPI.Get(jp, ax)
            lim.CreateLowAttr(1.0)
            lim.CreateHighAttr(-1.0)
        lim = UsdPhysics.LimitAPI.Get(jp, "transZ")
        lim.CreateLowAttr(0.0)
        lim.CreateHighAttr(0.01)          # 컵이 1 cm 눌렸다 펴지는 벨로즈 유연성
        for ax in ("rotX", "rotY", "rotZ"):
            lim = UsdPhysics.LimitAPI.Get(jp, ax)
            lim.CreateLowAttr(-3.0)
            lim.CreateHighAttr(3.0)       # 회전 한계는 도 단위 → ±3°
        for ax, stiff, damp in (("transZ", 5000.0, 100.0), ("rotX", 100.0, 0.0), ("rotY", 100.0, 0.0), ("rotZ", 10000.0, 0.0)):
            d = UsdPhysics.DriveAPI.Apply(jp, ax)
            d.CreateStiffnessAttr(stiff)
            d.CreateDampingAttr(damp)
        jp.CreateAttribute("isaac:forwardAxis", Sdf.ValueTypeNames.Token).Set("Z")
        jp.CreateAttribute("isaac:clearanceOffset", Sdf.ValueTypeNames.Float).Set(0.008)
        jp.ApplyAPI("IsaacAttachmentPointAPI")

        gp = robot_schema.CreateSurfaceGripper(self.stage, "/World/Arm/SuctionGripper")
        gp.GetRelationship(robot_schema.Relations.ATTACHMENT_POINTS.name).SetTargets([j.GetPath()])
        gp.GetAttribute(robot_schema.Attributes.MAX_GRIP_DISTANCE.name).Set(GRIP_DIST)
        gp.GetAttribute(robot_schema.Attributes.COAXIAL_FORCE_LIMIT.name).Set(COAXIAL_LIMIT)
        gp.GetAttribute(robot_schema.Attributes.SHEAR_FORCE_LIMIT.name).Set(SHEAR_LIMIT)
        gp.GetAttribute(robot_schema.Attributes.RETRY_INTERVAL.name).Set(RETRY)
        self.gripper_path = str(gp.GetPath())

    def start(self) -> None:
        super().start()
        from isaacsim.robot.surface_gripper import GripperView
        self.gv = GripperView(paths=self.gripper_path)
        self.gripper(0.0)          # 손가락은 접어 둔다 (컵만 앞으로 나온다)

    def move_path_joint(self, points, quats, seconds: float, tick, dt: float, res: dict, on_wp=None) -> bool:
        """직교 좌표 경유점들을 관절 공간에서 매끄럽게 잇는다. 각 경유점 IK 는 앞 해를 워밍으로 풀어
        같은 가지에 머물게 하고, 그 사이는 관절 선형 보간으로 간다 (스텝마다 IK 를 다시 풀지 않는다).
        quats 는 경유점마다의 손 방향 — 한 번에 꺾으면 그 각가속도로 흡착이 떨어진다."""
        # 안 풀리는 경유점은 건너뛴다 (경로가 조금 달라질 뿐). 끝점만 반드시 풀려야 한다
        sols, warm, skipped = [], self.joints(), 0
        for k, (pt, qt) in enumerate(zip(points, quats)):
            sol, err = self.solve(pt, qt, warm=warm)
            res["ik_err_max_m"] = round(max(res.get("ik_err_max_m", 0.0), err), 4)
            if sol is None:
                if k == len(points) - 1:
                    return False
                skipped += 1
                continue
            sols.append(sol)
            warm = sol
        res["path_skipped"] = skipped
        if not sols:
            return False
        per = max(1, int(seconds / dt / len(sols)))
        q0 = self.joints()
        for k, sol in enumerate(sols):
            for i in range(1, per + 1):
                self.art.set_dof_position_targets(q0 + (sol - q0) * i / per, dof_indices=self.arm_idx)
                tick()
            q0 = sol
            if on_wp:
                on_wp(f"over{k}")
        return True

    def suction(self, on: bool) -> None:
        self.gv.apply_gripper_action([1.0 if on else -1.0])

    def attached(self) -> str | None:
        got = self.gv.get_gripped_objects()
        return got[0][0] if got and got[0] else None

    # ── 한 번 집기 (평행 그리퍼와 같은 반환 형식 — verify_drive 가 그대로 읽는다)
    def _pick(self, line: dict, item_poses, tick, dt: float, on_event=None, aabb_override=None, allow_moved: bool = False) -> dict:
        from isaacsim.core.experimental.prims import RigidPrim
        s = self.spec
        n = np.array(line["approach_dir"], dtype=float)
        _, _, yaw = self.base
        fwd = np.array([math.cos(yaw), math.sin(yaw), 0.0])
        quat = quat_wxyz_from_axes(np.cross(fwd, n), fwd, n)     # 손 z = 접근 방향(컵 축)
        quat_ref = [quat]

        item = RigidPrim(line["prim"])
        lo, hi = self.current_aabb(line["prim"], item)
        if aabb_override is not None:
            lo, hi = np.asarray(aabb_override[0], dtype=float), np.asarray(aabb_override[1], dtype=float)
        c = (lo + hi) / 2
        ext_n = float(abs(np.dot(hi - lo, n)))
        planned = (np.array(line["item_aabb"][0]) + np.array(line["item_aabb"][1])) / 2
        moved = float(np.linalg.norm(c - planned))

        before = item_poses()
        self.frozen = not os.environ.get("ARM_NO_FREEZE")
        res = {"phase": "start", "ik_err_max_m": 0.0, "lifted": False, "held_after_retract": False, "in_bin": False,
               "disturbed_neighbors": 0, "moved_before_pick_m": round(moved, 4), "finger_gap_m": None,
               "cup_z_frac": line["stop"].get("cup_z_frac", 0.5)}
        if moved > 0.10 and not allow_moved:
            res["phase"] = "not_graspable_now"
            return res

        # 컵 자리: 앞면의 폭 중앙, 높이는 suction_study 가 고른 지점 (병은 어깨가 아니라 몸통)
        site = c - n * ext_n / 2 + n * float(line["stop"].get("face_dx_m", 0.0))
        site[2] = lo[2] + res["cup_z_frac"] * (hi[2] - lo[2])
        pre = site - n * (0.20 + s.suction_cup_len)
        touch = site - n * (s.suction_touch_gap + s.suction_cup_len)

        def ev(name):
            if on_event:
                on_event(name)

        def step_phase(name, p0, p1, seconds):
            res["phase"] = name
            ok, err = self.move_line(p0, p1, quat_ref[0], seconds, tick, dt)
            res["ik_err_max_m"] = round(max(res["ik_err_max_m"], err), 4)
            return ok

        def abort(phase):
            res["phase"] = phase
            self.suction(False)
            self.hold(0.5, tick, dt)
            self.go_tuck(1.5, tick, dt)
            return res

        # 1) tuck → 프리그래스프 (본체 위 공간을 지나는 직선 + 손 방향 slerp)
        pos0, q0 = self.tcp_pose()
        res["phase"] = "to_pre"
        ok, err = self.move_line(pos0, pre, quat, 1.8, tick, dt, quat0=q0)
        res["ik_err_max_m"] = round(err, 4)
        self.hold(0.3, tick, dt)
        if not ok or np.linalg.norm(self.tcp() - pre) > 0.03:
            res["to_pre_fallback"] = True
            self.go_tuck(1.0, tick, dt)
            sol, err = self.solve(pre, quat, warm=TUCK)
            res["ik_err_max_m"] = round(max(res["ik_err_max_m"], err), 4)
            if sol is None:
                return abort("to_pre_ik_fail")
            q_now = self.joints()
            nn = int(2.0 / dt)
            for i in range(1, nn + 1):
                self.art.set_dof_position_targets(q_now + (sol - q_now) * i / nn, dof_indices=self.arm_idx)
                tick()
            self.hold(0.3, tick, dt)
        res["pre_err_m"] = round(float(np.linalg.norm(self.tcp() - pre)), 4)
        ev("pre")

        # 2) 컵을 앞면에 붙인다. 진공은 미리 켜 둔다 (실제도 접근 중 켜 놓는다)
        self.suction(True)
        if not step_phase("approach", pre, touch, 1.6):
            return abort("approach_ik_fail")
        res["grasp_err_m"] = round(float(np.linalg.norm(self.tcp() - touch)), 4)
        # 닿은 자리에서 다시 켜고 붙을 때까지 기다린다 (진공이 자리를 잡는 시간)
        got, press = None, 0
        for k_ in range(6):
            self.suction(True)
            self.hold(0.3, tick, dt)
            got = self.attached()
            if got:
                break
            if k_ % 2 == 1 and press < 3:      # 안 붙으면 3 mm 씩 더 눌러 본다 (표면이 예상보다 안쪽)
                press += 1
                self.move_line(self.tcp(), touch + n * 0.003 * press, quat_ref[0], 0.3, tick, dt)
        res["attach_tries"] = k_ + 1
        res["press_mm"] = press * 3
        res["attached_prim"] = got
        res["attached"] = got is not None
        ev("grasp")
        if not got:
            return abort("no_attach")
        if got != line["prim"]:            # 옆·뒤 상품에 붙었으면 그걸 집는 것으로 기록
            res["grasped_prim"] = got
            item = RigidPrim(got)
        else:
            res["grasped_prim"] = line["prim"]
        z0 = float(item.get_world_poses()[0].numpy()[0][2])
        pos_at_grasp = item.get_world_poses()[0].numpy()[0]
        dist0 = float(np.linalg.norm(pos_at_grasp - self.tcp()))

        # 3) 곧게 빼낸다 (흡착은 전단이 약하니 천천히). 빼자마자 **제자리에서** 손목을 아래로 돌려
        #    상품이 컵 밑에 매달리게 만든다 — 그러면 무게가 전단에서 인장으로 바뀌어 이후 이동이 안전하다
        if not step_phase("retract", touch, pre, 2.5 * CARRY_SLOW):
            return abort("retract_ik_fail")
        self.hold(0.4, tick, dt)
        ip = item.get_world_poses()[0].numpy()[0]
        res["lift_m"] = round(float(ip[2] - z0), 4)
        res["lifted"] = bool(self.attached() is not None)
        res["held_after_retract"] = res["lifted"]
        ev("retract")
        if not res["lifted"]:
            return abort("lost_on_retract")

        fwd_ = np.array([math.cos(yaw), math.sin(yaw), 0.0])
        quat_down = quat_wxyz_from_axes(np.cross(fwd_, [0, 0, -1.0]), fwd_, [0, 0, -1.0])
        _, q_now = self.tcp_pose()
        res["phase"] = "turn_down"
        okr, err = self.move_line(pre, pre, quat_down, 2.0 * CARRY_SLOW, tick, dt, quat0=q_now)
        res["ik_err_max_m"] = round(max(res["ik_err_max_m"], err), 4)
        self.hold(0.3, tick, dt)
        quat_ref[0] = quat_down
        out = pre + np.array([0, 0, 0.03])

        # 4) 바구니로 (평행 그리퍼와 같은 경로: 위로 → 바구니 위 → 내리기)
        bx, by, bz = self.bin_local
        x, y, yaw = self.base
        cy, sy = math.cos(yaw), math.sin(yaw)
        bin_c = np.array([x + cy * bx - sy * by, y + sy * bx + cy * by, s.spawn_z + bz])
        # 스윙 고도: 높을수록 매달린 상품이 팔·바구니를 안 치지만, 너무 높으면 팔이 안 닿는다(어깨 1.01 m, 도달 0.855).
        # 바구니 위 자세가 실제로 풀리는 가장 높은 고도를 고른다
        swing_z = SWING_Z
        for cand in (SWING_Z, 0.38, 0.32, 0.26):
            probe = np.array([bin_c[0], bin_c[1], bin_c[2] + cand])
            sol_, _ = self.solve(probe, quat_down, warm=TUCK)
            if sol_ is not None:
                swing_z = cand
                break
        res["swing_z"] = swing_z
        high = np.array([out[0], out[1], bin_c[2] + swing_z])   # 상품 높이에서 스윙 고도로 (위든 아래든)
        over = np.array([bin_c[0], bin_c[1], bin_c[2] + swing_z])
        drop = np.array([bin_c[0], bin_c[1], bin_c[2] + 0.22])
        held_trace = []

        def held_now(tag):
            ip_ = item.get_world_poses()[0].numpy()[0]
            held_trace.append((tag, round(float(abs(np.linalg.norm(ip_ - self.tcp()) - dist0)), 3),
                               1.0 if self.attached() else 0.0))

        held_now("retract")
        res["phase"] = "carry"
        # 손은 이미 아래를 보고 있다 (상품이 매달림). 자세를 고정한 채 위로 → 바구니 위로
        ok = self.move_line(out, high, quat_down, 1.5 * CARRY_SLOW, tick, dt)[0]
        if not ok or np.linalg.norm(self.tcp() - high) > 0.05:
            # 자세를 고정한 수직 이동이 안 풀리면 위치만 맞춘다 (매달린 상품은 방향이 조금 틀어져도 된다)
            res["up_fallback"] = True
            ok = self.move_line_pos(self.tcp(), high, 1.5 * CARRY_SLOW, tick, dt)
        held_now("up")
        ev("high")
        if ok:
            ok = self.move_path_joint([high + (over - high) * (i / 8) for i in range(1, 9)], [quat_down] * 8,
                                      3.0 * CARRY_SLOW, tick, dt, res, held_now)
            self.hold(0.4, tick, dt)
            if not ok or np.linalg.norm(self.tcp() - over) > 0.05:
                res["carry_over_fallback"] = True
                ok = self.move_line_pos(self.tcp(), over, 2.0 * CARRY_SLOW, tick, dt)
                self.hold(0.3, tick, dt)
        held_now("over")
        ev("over")
        ok = ok and step_phase("carry_down", over, drop, 1.0)
        self.hold(0.3, tick, dt)
        held_now("down")
        res["held_trace"] = held_trace
        reached = float(np.linalg.norm(self.tcp() - drop))
        res["carry_err_m"] = round(reached, 4)
        ev("over_bin")
        if not ok or reached > 0.05:
            return abort("carry_not_reached" if ok else res["phase"] + "_ik_fail")

        # 5) 진공 끊기
        res["phase"] = "release"
        self.suction(False)
        self.hold(1.0, tick, dt)
        ip = item.get_world_poses()[0].numpy()[0]
        rel = np.array([ip[0] - x, ip[1] - y])
        lx_ = cy * rel[0] + sy * rel[1]
        ly_ = -sy * rel[0] + cy * rel[1]
        from tools.arm_isaac import BIN_SIZE
        res["in_bin"] = bool(abs(lx_ - bx) <= BIN_SIZE[0] / 2 and abs(ly_ - by) <= BIN_SIZE[1] / 2
                             and s.spawn_z + bz - 0.02 <= ip[2] <= s.spawn_z + bz + BIN_SIZE[2] + 0.15)
        res["item_final"] = [round(float(v), 4) for v in ip]
        res["phase"] = "tuck"
        self.go_tuck(1.5, tick, dt)
        after = item_poses()
        res["disturbed_neighbors"] = sum(1 for k, p in after.items() if k not in (line["prim"], res.get("grasped_prim")) and k in before
                                         and np.linalg.norm(np.asarray(p) - np.asarray(before[k])) > 0.02)
        res["phase"] = "done"
        res["held_through_carry"] = bool(all(g > 0 for _, _, g in held_trace))
        res["success"] = bool(res["lifted"] and res["in_bin"])
        return res
