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
