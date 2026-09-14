"""Train/test/infer with RF-DETR-Seg (nano).
Dataset must be COCO segmentation format:
    dataset_dir/train/_annotations.coco.json + images
    dataset_dir/valid/_annotations.coco.json + images
"""
import argparse
from pathlib import Path

from rfdetr import RFDETRSegNano


def train(args):
    model = RFDETRSegNano()
    model.train(
        dataset_dir=args.data,
        epochs=args.epochs,
        batch_size=args.batch_size,
        grad_accum_steps=1,
        output_dir=args.output_dir,
    )
    # RF-DETR writes checkpoint_best_ema.pth into output_dir itself —
    # no extra copy step needed.


def test(args):
    model = RFDETRSegNano(pretrain_weights=args.weights)
    model.optimize_for_inference()  # exports/optimizes before benchmarking latency
    import time
    val_images = list((Path(args.data) / "valid").glob("*.jpg"))[:10]
    latencies = []
    for img in val_images:
        t0 = time.time()
        model.predict(str(img), threshold=0.5)
        latencies.append((time.time() - t0) * 1000)
    if latencies:
        avg = sum(latencies) / len(latencies)
        print(f"Latency: {avg:.2f} ms ({1000/avg:.1f} FPS)")


def infer(args):
    import supervision as sv
    from PIL import Image

    model = RFDETRSegNano(pretrain_weights=args.weights)
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    for img_path in args.images:
        image = Image.open(img_path)
        detections = model.predict(image, threshold=0.5)
        annotated = sv.MaskAnnotator().annotate(image.copy(), detections)
        annotated.save(out_dir / f"{Path(img_path).stem}_pred.jpg")


def main():
    parser = argparse.ArgumentParser()
    sub = parser.add_subparsers(dest="command", required=True)

    p = sub.add_parser("train")
    p.add_argument("--data", required=True, help="COCO dataset_dir")
    p.add_argument("--epochs", type=int, default=30)
    p.add_argument("--batch-size", type=int, default=4)
    p.add_argument("--output-dir", default="checkpoints/rfdetr_run")

    p = sub.add_parser("test")
    p.add_argument("--weights", required=True)
    p.add_argument("--data", required=True)

    p = sub.add_parser("infer")
    p.add_argument("--weights", required=True)
    p.add_argument("--images", nargs="+", required=True)
    p.add_argument("--out-dir", default="inference_samples")

    args = parser.parse_args()
    {"train": train, "test": test, "infer": infer}[args.command](args)


if __name__ == "__main__":
    main()
