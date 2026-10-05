"""Generate graphs for a single model's results/<model_name>.json.

Usage:
    python graph_model.py --results results/yolo11_seg.json --out-dir report/yolo11_seg
"""
import argparse
import json
from pathlib import Path

import matplotlib.pyplot as plt


def load_records(results_path: Path) -> list[dict]:
    with open(results_path) as f:
        records = json.load(f)
    records.sort(key=lambda r: r.get("epoch", 0))
    return records


def best_epoch(records: list[dict]) -> dict:
    return max(records, key=lambda r: r.get("miou", float("-inf")))


def plot_training_curve(records, model_name, out_path, metric):
    epochs = [r["epoch"] for r in records if metric in r and r[metric] is not None]
    values = [r[metric] for r in records if metric in r and r[metric] is not None]
    if not epochs:
        print(f"No '{metric}' data found — skipping.")
        return

    fig, ax = plt.subplots(figsize=(7, 5))
    ax.plot(epochs, values, marker="o", markersize=4)
    ax.set_xlabel("Epoch")
    ax.set_ylabel(metric)
    ax.set_title(f"{model_name}: {metric} vs. epoch")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)


def plot_per_class_iou(records, model_name, out_path):
    best = best_epoch(records)
    class_iou = best.get("per_class_iou", {})
    if not class_iou:
        print("No per_class_iou data found — skipping per-class chart.")
        return

    classes = list(class_iou.keys())
    values = [class_iou[c] for c in classes]

    fig, ax = plt.subplots(figsize=(1.2 * len(classes) + 2, 5))
    ax.bar(classes, values)
    ax.set_ylabel("IoU")
    ax.set_title(f"{model_name}: per-class IoU (epoch {best.get('epoch')})")
    ax.set_ylim(0, 1)
    ax.grid(True, alpha=0.3, axis="y")
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)


def print_summary(records, model_name):
    best = best_epoch(records)
    latency_ms = best.get("inference_latency_ms")
    fps = round(1000.0 / latency_ms, 1) if latency_ms else None

    print(f"\n{model_name} summary (best epoch {best.get('epoch')}):")
    print(f"  mIoU:            {best.get('miou')}")
    print(f"  FPS:             {fps}")
    print(f"  Latency (ms):    {latency_ms}")
    print(f"  Model size (MB): {best.get('model_size_mb')}")
    avg_epoch_time = sum(r.get("epoch_train_time_s", 0) for r in records) / len(records)
    print(f"  Avg epoch time:  {round(avg_epoch_time, 1)}s")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--results", required=True, type=Path, help="Path to results/<model_name>.json")
    parser.add_argument("--out-dir", type=Path, default=None, help="Defaults to report/<model_name>/")
    args = parser.parse_args()

    records = load_records(args.results)
    if not records:
        raise ValueError(f"{args.results} is empty")
    model_name = records[0].get("model", args.results.stem)

    out_dir = args.out_dir or Path("report") / model_name.replace("-", "_")
    out_dir.mkdir(parents=True, exist_ok=True)

    plot_training_curve(records, model_name, out_dir / "miou_curve.png", metric="miou")
    plot_training_curve(records, model_name, out_dir / "loss_curve.png", metric="loss")
    plot_per_class_iou(records, model_name, out_dir / "per_class_iou.png")
    print_summary(records, model_name)

    print(f"\nGraphs written to {out_dir}/")


if __name__ == "__main__":
    main()
