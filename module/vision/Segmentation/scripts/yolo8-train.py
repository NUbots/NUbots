"""Train/test/infer with YOLOv8 (detection, not segmentation)."""
import argparse
import json
import time
from pathlib import Path

from ultralytics import YOLO

CHECKPOINT = "yolov8n.pt"
MODEL_NAME = "yolov8-det"


def train(args):
    model = YOLO(CHECKPOINT)
    records = {}
    start = {"t": None}

    def on_epoch_start(trainer):
        start["t"] = time.time()

    def on_epoch_end(trainer):
        if trainer.epoch >= trainer.epochs:
            return  # skip Ultralytics' extra final revalidation callback
        m = trainer.metrics
        # Detection metrics use the (B) box suffix, not (M) mask like the seg models.
        records[trainer.epoch + 1] = {
            "model": MODEL_NAME,
            "epoch": trainer.epoch + 1,
            "miou": round(float(m.get("metrics/mAP50-95(B)", 0.0)), 4),
            "per_class_iou": {},
            "loss": round(float(trainer.loss.item()), 4) if hasattr(trainer.loss, "item") else None,
            "inference_latency_ms": None,
            "model_size_mb": None,
            "epoch_train_time_s": round(time.time() - start["t"], 1) if start["t"] else None,
        }
        Path(args.results_dir).mkdir(parents=True, exist_ok=True)
        with open(Path(args.results_dir) / f"{MODEL_NAME.replace('-', '_')}.json", "w") as f:
            json.dump(list(records.values()), f, indent=2)

    model.add_callback("on_train_epoch_start", on_epoch_start)
    model.add_callback("on_fit_epoch_end", on_epoch_end)

    ckpt_dir = Path(args.checkpoints_dir).resolve()
    ckpt_dir.parent.mkdir(parents=True, exist_ok=True)
    model.train(
        data=args.data, epochs=args.epochs, imgsz=args.imgsz,
        project=str(ckpt_dir.parent), name=ckpt_dir.name, exist_ok=True,
    )

    best_src = ckpt_dir / "weights" / "best.pt"
    if best_src.exists():
        best_dst = ckpt_dir.parent / f"{MODEL_NAME.replace('-', '_')}_best.pt"
        best_dst.write_bytes(best_src.read_bytes())
        if records:
            last_epoch = max(records)
            records[last_epoch]["model_size_mb"] = round(best_dst.stat().st_size / (1024 * 1024), 2)
            with open(Path(args.results_dir) / f"{MODEL_NAME.replace('-', '_')}.json", "w") as f:
                json.dump(list(records.values()), f, indent=2)
        print(f"Best checkpoint: {best_dst}")


def test(args):
    model = YOLO(args.weights)
    model.val(data=args.data, imgsz=args.imgsz)

    val_images = list((Path(args.data).parent / "images" / "val").glob("*"))[:10]
    latencies = []
    for img in val_images:
        t0 = time.time()
        model(str(img), verbose=False)
        latencies.append((time.time() - t0) * 1000)
    if latencies:
        avg = sum(latencies) / len(latencies)
        print(f"Latency: {avg:.2f} ms ({1000/avg:.1f} FPS)")


def infer(args):
    model = YOLO(args.weights)
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    for img_path in args.images:
        for r in model(img_path, verbose=False):
            r.save(filename=str(out_dir / f"{Path(img_path).stem}_pred.jpg"))


def main():
    parser = argparse.ArgumentParser()
    sub = parser.add_subparsers(dest="command", required=True)

    p = sub.add_parser("train")
    p.add_argument("--data", required=True)
    p.add_argument("--epochs", type=int, default=30)
    p.add_argument("--imgsz", type=int, default=640)
    p.add_argument("--results-dir", default="results")
    p.add_argument("--checkpoints-dir", default=f"checkpoints/{MODEL_NAME.replace('-', '_')}_run")

    p = sub.add_parser("test")
    p.add_argument("--weights", required=True)
    p.add_argument("--data", required=True)
    p.add_argument("--imgsz", type=int, default=640)

    p = sub.add_parser("infer")
    p.add_argument("--weights", required=True)
    p.add_argument("--images", nargs="+", required=True)
    p.add_argument("--out-dir", default="inference_samples")

    args = parser.parse_args()
    {"train": train, "test": test, "infer": infer}[args.command](args)


if __name__ == "__main__":
    main()
