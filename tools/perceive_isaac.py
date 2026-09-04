"""헤드 카메라 인식 — 정차 자세에서 대상 상품의 3D 상자를 카메라로 추정한다 (drive_isaac --perceive).

파이프라인:
  팬틸트 헤드 카메라(Carter 상판, 팔 반대쪽) → 계획이 알려준 대상 방향으로 향한다
  → RGB · 깊이(distance_to_image_plane) · 인스턴스 분할(시맨틱 라벨 = 상품명)
  → 대상 프림의 마스크 픽셀을 카메라 내부 파라미터로 3D 로 올려 (핀홀) 월드 좌표 점군
  → 점군 AABB = 추정 상자 → arm_isaac.pick 에 넘긴다

정직한 범위: 검출·분할은 시뮬 정답 라벨(완벽한 검출기 가정)이고, 3D 위치는 깊이 카메라
기하로 계산한다. 즉 "검출기가 맞혔다고 치고, 그 뒤 기하가 파지까지 이어지는가"를 본다.
보이는 면의 점만 있으므로 추정 상자는 진짜 상자보다 얇다(뒷면이 없다) — 파지점 계산은
앞면 + 깊이를 쓰므로 앞면만 맞으면 된다. 같은 자리에서 RGB + 2D 박스를 데이터셋으로 남긴다.
"""

from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np
from pxr import Gf, UsdGeom

from scene.constants import ROBOT, RobotSpec

CAM_LOCAL = (0.05, 0.20, 0.86)   # [설계] 본체 프레임(원점 = 바퀴 축 높이 0.255): 바닥에서 1.12 m, 왼쪽(진열대 쪽)으로 20 cm. 0.7 m 로 두면 선반 아래에서 올려다보게 된다
FOCAL_MM = 12.0                  # [설계] 광각 (수평 화각 ~82°). 한 단 위아래가 같이 들어온다
RES = (640, 480)


class Perceiver:
    def __init__(self, stage, app, robot_path: str = "/World/Robot", spec: RobotSpec = ROBOT, res=RES):
        import omni.replicator.core as rep
        self.rep = rep
        self.app = app
        self.spec = spec
        self.res = res
        self.cam_path = f"{robot_path}/chassis_link/HeadCam"
        cam = UsdGeom.Camera.Define(stage, self.cam_path)
        cam.CreateFocalLengthAttr(FOCAL_MM)
        cam.CreateClippingRangeAttr(Gf.Vec2f(0.05, 50.0))
        self.cam = cam
        self.h_ap = cam.GetHorizontalApertureAttr().Get()
        self.v_ap = cam.GetVerticalApertureAttr().Get()
        self.rp = rep.create.render_product(self.cam_path, tuple(res))
        self.ann = {}
        for name, params in (("rgb", {}), ("distance_to_image_plane", {}), ("instance_segmentation_fast", {"colorize": False}),
                             ("bounding_box_2d_tight_fast", {})):
            a = rep.AnnotatorRegistry.get_annotator(name, init_params=params)
            a.attach([self.rp])
            self.ann[name] = a
        self.local_T = None
        self.last = None

    # ── 향하기: 본체 프레임에서 카메라가 target(월드) 을 보게
    def aim(self, robot_pose, target_world) -> None:
        x, y, yaw = robot_pose
        c, s = math.cos(yaw), math.sin(yaw)
        dx, dy, dz = target_world[0] - x, target_world[1] - y, target_world[2] - self.spec.spawn_z
        local = Gf.Vec3d(c * dx + s * dy, -s * dx + c * dy, dz)
        eye = Gf.Vec3d(*CAM_LOCAL)
        m = Gf.Matrix4d().SetLookAt(eye, local, Gf.Vec3d(0, 0, 1)).GetInverse()
        xf = UsdGeom.Xformable(self.cam)
        xf.ClearXformOpOrder()
        xf.AddTransformOp().Set(m)
        self.local_T = m

    def cam_world_matrix(self, robot_pose) -> Gf.Matrix4d:
        x, y, yaw = robot_pose
        R = Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(0, 0, 1), math.degrees(yaw)))
        T = Gf.Matrix4d().SetTranslate(Gf.Vec3d(x, y, self.spec.spawn_z))
        return self.local_T * R * T          # 행벡터 규약: 로컬 → 본체 → 월드

    # ── 찍기
    def observe(self) -> dict:
        for _ in range(3):                    # annotator 는 한 프레임 늦다. orchestrator.step 은 타임라인을 멈춰 물리 뷰를 깨므로 app.update
            self.app.update()
        rgb = self.ann["rgb"].get_data()
        depth = self.ann["distance_to_image_plane"].get_data()
        seg = self.ann["instance_segmentation_fast"].get_data()
        bb = self.ann["bounding_box_2d_tight_fast"].get_data()
        self.last = {"rgb": np.array(rgb[..., :3]), "depth": np.array(depth), "seg": np.array(seg["data"]),
                     "seg_labels": {int(k): v for k, v in seg["info"]["idToLabels"].items()},
                     "bboxes": bb["data"], "bbox_paths": list(bb["info"].get("primPaths", [])), "bbox_labels": bb["info"].get("idToLabels", {})}
        return self.last

    def _id_of(self, prim_path: str) -> int | None:
        for k, v in self.last["seg_labels"].items():
            if v == prim_path:
                return k
        return None

    # ── 대상 프림의 3D 상자 추정
    def locate(self, prim_path: str, robot_pose, floor_z: float | None = None) -> dict | None:
        if self.last is None:
            return None
        iid = self._id_of(prim_path)
        if iid is None:
            return None
        mask = self.last["seg"] == iid
        n = int(mask.sum())
        if n < 30:
            return None
        w, h = self.res
        fx = FOCAL_MM / self.h_ap * w
        fy = FOCAL_MM / self.v_ap * h
        cx, cy = w / 2, h / 2
        v, u = np.nonzero(mask)
        d = self.last["depth"][v, u].astype(np.float64)
        ok = np.isfinite(d) & (d > 0.05)
        u, v, d = u[ok], v[ok], d[ok]
        xc = (u + 0.5 - cx) / fx * d
        yc = -(v + 0.5 - cy) / fy * d
        zc = -d                                  # USD 카메라는 -Z 를 본다
        pts_cam = np.c_[xc, yc, zc]
        M = np.array(self.cam_world_matrix(robot_pose), dtype=float)
        pts = (np.c_[pts_cam, np.ones(len(pts_cam))] @ M)[:, :3]
        # 마스크 가장자리의 혼합 픽셀(깊이가 뒤 배경으로 튄다)이 상자를 키운다 → 백분위로 자른다
        lo, hi = np.percentile(pts, 2, axis=0), np.percentile(pts, 98, axis=0)
        # 상품 밑면은 카메라에 안 보인다 (위에서 보면 상품 자체가, 아래에서 보면 선반 판이 가린다).
        # 매장 모델이 선반 높이를 아니 그걸 밑면으로 쓴다 — 실제 로봇도 매장 지도의 선반 높이를 안다
        if floor_z is not None:
            lo[2] = floor_z
        return {"lo": lo, "hi": hi, "n_points": n, "n_used": int(ok.sum()), "u_range": (int(u.min()), int(u.max())), "v_range": (int(v.min()), int(v.max()))}

    # ── 검출기 기반: 정답 마스크 대신 YOLO 박스 + 박스 안 깊이 군집으로 3D 상자
    def load_detector(self, weights: str) -> None:
        from ultralytics import YOLO
        self.det = YOLO(weights)
        self.det_names = self.det.names

    def project(self, world_pt, robot_pose):
        """월드 점 → 픽셀 (u, v). 대상이 어디쯤 보여야 하는지(계획) 알기 위해."""
        M = np.array(self.cam_world_matrix(robot_pose), dtype=float)
        Minv = np.linalg.inv(M)
        pc = (np.r_[world_pt, 1.0] @ Minv)[:3]
        w, h = self.res
        fx = FOCAL_MM / self.h_ap * w
        fy = FOCAL_MM / self.v_ap * h
        d = -pc[2]
        if d <= 0:
            return None
        return (pc[0] / d * fx + w / 2, -pc[1] / d * fy + h / 2)

    def locate_detected(self, product: str, robot_pose, expected_world, floor_z: float | None = None, conf: float = 0.15) -> dict | None:
        """검출기가 찾은 같은 상품명 박스 중 계획 위치에 가장 가까운 것을 고르고, 박스 안 깊이 픽셀을
        박스 중앙 깊이 ±6 cm 로 걸러(배경·이웃 제외) 3D 로 올린다. 검출 실패·클래스 불일치는 None."""
        if self.last is None or not hasattr(self, "det"):
            return None
        res = self.det.predict(self.last["rgb"][..., ::-1].copy(), imgsz=640, conf=conf, verbose=False)[0]   # ultralytics 는 numpy 입력을 BGR 로 본다
        exp_uv = self.project(np.asarray(expected_world, dtype=float), robot_pose)
        self.last_det = res
        best, mismatch = None, False
        for b in res.boxes:
            name = self.det_names[int(b.cls)]
            if name != product:
                continue
            x0, y0, x1, y1 = [float(v) for v in b.xyxy[0]]
            cu, cv = (x0 + x1) / 2, (y0 + y1) / 2
            dist = math.hypot(cu - exp_uv[0], cv - exp_uv[1]) if exp_uv else 0.0
            if best is None or dist < best[0]:
                best = (dist, (x0, y0, x1, y1), float(b.conf), name)
        if best is None and exp_uv is not None:
            # 같은 상품명이 없으면: 계획 위치(픽셀) 60 px 안의 아무 박스. 검출기가 클래스를 틀린 경우다 — 기록에 남긴다
            for b in res.boxes:
                x0, y0, x1, y1 = [float(v) for v in b.xyxy[0]]
                cu, cv = (x0 + x1) / 2, (y0 + y1) / 2
                dist = math.hypot(cu - exp_uv[0], cv - exp_uv[1])
                if dist < 60 and (best is None or dist < best[0]):
                    best = (dist, (x0, y0, x1, y1), float(b.conf), self.det_names[int(b.cls)])
                    mismatch = True
        n_det = len(res.boxes)
        if best is None:
            return {"detected": False, "n_det": n_det}
        px_dist, (x0, y0, x1, y1), c, det_name = best
        w, h = self.res
        fx = FOCAL_MM / self.h_ap * w
        fy = FOCAL_MM / self.v_ap * h
        cx, cy = w / 2, h / 2
        # 박스 안쪽 80 % 만 (테두리는 배경이 섞인다)
        mx, my = (x1 - x0) * 0.1, (y1 - y0) * 0.1
        u0, u1 = int(max(0, x0 + mx)), int(min(w - 1, x1 - mx))
        v0, v1 = int(max(0, y0 + my)), int(min(h - 1, y1 - my))
        if u1 <= u0 or v1 <= v0:
            return {"detected": True, "n_det": n_det, "empty": True}
        depth = self.last["depth"][v0:v1, u0:u1]
        vv, uu = np.mgrid[v0:v1, u0:u1]
        d = depth.astype(np.float64).ravel(); uu = uu.ravel(); vv = vv.ravel()
        ok = np.isfinite(d) & (d > 0.05)
        d, uu, vv = d[ok], uu[ok], vv[ok]
        if len(d) < 30:
            return {"detected": True, "n_det": n_det, "empty": True}
        # 박스 중앙 근처 깊이를 기준으로 ±6 cm 만 (상품 한 개의 깊이 범위). 뒤 배경·앞 이웃을 잘라낸다
        ref = np.median(d[(np.abs(uu - (u0 + u1) / 2) < (u1 - u0) * 0.2) & (np.abs(vv - (v0 + v1) / 2) < (v1 - v0) * 0.2)]) if len(d) else np.median(d)
        keep = np.abs(d - ref) < 0.06
        d, uu, vv = d[keep], uu[keep], vv[keep]
        xc = (uu + 0.5 - cx) / fx * d
        yc = -(vv + 0.5 - cy) / fy * d
        pts_cam = np.c_[xc, yc, -d]
        M = np.array(self.cam_world_matrix(robot_pose), dtype=float)
        pts = (np.c_[pts_cam, np.ones(len(pts_cam))] @ M)[:, :3]
        lo, hi = np.percentile(pts, 2, axis=0), np.percentile(pts, 98, axis=0)
        if floor_z is not None:
            lo[2] = floor_z
        return {"detected": True, "n_det": n_det, "lo": lo, "hi": hi, "n_points": int(len(d)), "n_used": int(len(d)), "conf": round(c, 3),
                "px_dist_to_plan": round(px_dist, 1), "box": [round(x0), round(y0), round(x1), round(y1)],
                "class_mismatch": mismatch, "det_class": det_name}

    def save_detections(self, path) -> None:
        """검출기가 본 대로 그린 프레임 (디버그·README)."""
        if getattr(self, "last_det", None) is None:
            return
        from PIL import Image
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        Image.fromarray(self.last_det.plot(line_width=1, font_size=6)[..., ::-1]).save(path)

    # ── 데이터셋 기록: RGB + 대상 마스크 오버레이 + 보이는 상품 전부의 2D 박스
    def snapshot(self, out_dir: Path, tag: str, target_prim: str | None = None) -> dict:
        from PIL import Image
        out_dir.mkdir(parents=True, exist_ok=True)
        rgb = self.last["rgb"]
        Image.fromarray(rgb).save(out_dir / f"{tag}_rgb.png")
        if target_prim:
            iid = self._id_of(target_prim)
            if iid is not None:
                over = rgb.copy()
                m = self.last["seg"] == iid
                over[m] = (0.45 * over[m] + 0.55 * np.array([255, 40, 40])).astype(np.uint8)
                Image.fromarray(over).save(out_dir / f"{tag}_target.png")
        boxes = []
        for row, path in zip(self.last["bboxes"], self.last["bbox_paths"]):
            lab = self.last["bbox_labels"].get(str(row["semanticId"]), self.last["bbox_labels"].get(int(row["semanticId"]), {}))
            boxes.append({"prim": path, "class": lab.get("class") if isinstance(lab, dict) else lab,
                          "xyxy": [int(row["x_min"]), int(row["y_min"]), int(row["x_max"]), int(row["y_max"])], "occlusion": float(row["occlusionRatio"])})
        rec = {"image": f"{tag}_rgb.png", "size": list(self.res), "boxes": boxes, "target": target_prim}
        (out_dir / f"{tag}.json").write_text(json.dumps(rec, ensure_ascii=False))
        return rec
