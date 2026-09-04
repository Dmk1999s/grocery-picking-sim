"""합성 데이터로 상품 검출기(YOLO)를 학습하고 검증 매장에서 평가한다.

    ~/yolo-venv/bin/python -m tools.train_detector --data out/dataset --epochs 40 --out out/detector

데이터는 tools/dataset_isaac.py 가 만든 YOLO 형식. 학습은 seed 7 매장, 검증은 seed 3 매장 —
같은 상품·같은 진열대지만 배치·조명·흔들림이 다른 "다른 날의 매장"이다.
결과: <out>/best.pt, <out>/metrics.json (mAP50, mAP50-95, 클래스별 AP)
"""

from __future__ import annotations

import argparse
import json
import shutil
from pathlib import Path


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--data", default="out/dataset")
    ap.add_argument("--out", default="out/detector")
    ap.add_argument("--model", default="yolov8n.pt", help="사전학습 가중치. n = 3.2 M 파라미터, 로봇 온보드용")
    ap.add_argument("--epochs", type=int, default=40)
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--batch", type=int, default=16)
    args = ap.parse_args()

    from ultralytics import YOLO

    data = Path(args.data).resolve()
    classes = (data / "train" / "classes.txt").read_text().split()
    yaml = data / "data.yaml"
    yaml.write_text(
        f"path: {data}\ntrain: train/images\nval: val/images\nnames:\n" + "".join(f"  {i}: {n}\n" for i, n in enumerate(classes))
    )
    out = Path(args.out).resolve()     # 상대경로면 ultralytics 가 runs/detect/ 아래로 붙인다
    out.mkdir(parents=True, exist_ok=True)
    model = YOLO(args.model)
    model.train(data=str(yaml), epochs=args.epochs, imgsz=args.imgsz, batch=args.batch, project=str(out), name="run",
                exist_ok=True, verbose=False, plots=True, workers=4)
    best = out / "run" / "weights" / "best.pt"
    shutil.copy(best, out / "best.pt")
    m = YOLO(str(best)).val(data=str(yaml), imgsz=args.imgsz, batch=args.batch, project=str(out), name="val", exist_ok=True, plots=True, verbose=False)
    per_class = {classes[int(i)]: round(float(v), 3) for i, v in zip(m.box.ap_class_index, m.box.ap50)}
    metrics = {"mAP50": round(float(m.box.map50), 4), "mAP50-95": round(float(m.box.map), 4), "precision": round(float(m.box.mp), 4), "recall": round(float(m.box.mr), 4),
               "epochs": args.epochs, "model": args.model, "n_train": len(list((data / "train" / "images").glob("*.png"))), "n_val": len(list((data / "val" / "images").glob("*.png"))),
               "ap50_per_class": dict(sorted(per_class.items(), key=lambda kv: kv[1]))}
    (out / "metrics.json").write_text(json.dumps(metrics, ensure_ascii=False, indent=1))
    print(json.dumps({k: v for k, v in metrics.items() if k != "ap50_per_class"}, ensure_ascii=False))
    print("AP50 낮은 순:", list(metrics["ap50_per_class"].items())[:6])


if __name__ == "__main__":
    main()
