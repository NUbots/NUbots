"""
NUbots segmentation model benchmarking — metrics aggregation + charts.

Expected input: one JSON file per model under `results/`, e.g.
    results/pp_liteseg.json
    results/bisenetv2.json
    results/yolov8_seg.json
    results/yolov11_seg.json
    results/rf_detr.json

Each file holds a JSON array of per-epoch records, matching the schema:
[
  {
    "model": "PP-LiteSeg",
    "epoch": 1,
    "miou": 0.512,
    "per_class_iou": {"ball": 0.60, "robot": 0.48, "goalpost": 0.55},
    "loss": 0.812,
    "inference_latency_ms": 12.4,
    "model_size_mb": 8.2,
    "epoch_train_time_s": 41.3
  },
  {
    "model": "PP-LiteSeg",
    "epoch": 2,
    ...
  }
]

Your per-model training script should accumulate one such record per
epoch and write the full array to results/<model_name>.json (e.g. once
at the end of training, or overwritten after every epoch so partial
progress survives a crash). This script only reads the common schema —
it doesn't care which framework produced it.

Usage:
    python benchmark_report.py --results-dir results --out-dir report
"""

import argparse
import json
from pathlib import Path
from collections import defaultdict

import matplotlib.pyplot as plt


# ---------------------------------------------------------------------------
# Loading
# ---------------------------------------------------------------------------

def load_logs(results_dir: Path) -> dict[str, list[dict]]:
    """Read every *.json file in results_dir into {model_name: [epoch records]}."""
    records_by_model: dict[str, list[dict]] = defaultdict(list)

    json_files = sorted(results_dir.glob("*.json"))
    if not json_files:
        raise FileNotFoundError(
            f"No .json files found in {results_dir}. "
            "Expected one file per model (e.g. results/pp_liteseg.json), "
            "each containing a JSON array of per-epoch records."
        )

    for path in json_files:
        with open(path) as f:
            try:
                records = json.load(f)
            except json.JSONDecodeError as e:
                raise ValueError(f"{path} is not valid JSON") from e

        if not isinstance(records, list):
            raise ValueError(
                f"{path} must contain a JSON array of per-epoch records, "
                f"got {type(records).__name__}"
            )

        for i, record in enumerate(records):
            if "model" not in record:
                raise ValueError(f"{path}[{i}] missing required 'model' field")
            records_by_model[record["model"]].append(record)

    # Sort each model's records by epoch so training curves are drawn in order.
    for model, records in records_by_model.items():
        records.sort(key=lambda r: r.get("epoch", 0))

    return dict(records_by_model)


def best_epoch(records: list[dict]) -> dict:
    """Pick the record with the highest mIoU for a model (its 'best checkpoint')."""
    return max(records, key=lambda r: r.get("miou", float("-inf")))


# ---------------------------------------------------------------------------
# Charts
# ---------------------------------------------------------------------------

def plot_accuracy_vs_speed(records_by_model: dict[str, list[dict]], out_path: Path):
    """Scatter: best mIoU (y) vs inference FPS (x), one point per model."""
    fig, ax = plt.subplots(figsize=(7, 5))

    for model, records in records_by_model.items():
        best = best_epoch(records)
        latency_ms = best.get("inference_latency_ms")
        if latency_ms is None or latency_ms <= 0:
            continue  # can't plot FPS without latency
        fps = 1000.0 / latency_ms
        miou = best.get("miou", 0)

        ax.scatter(fps, miou, s=90)
        ax.annotate(
            model,
            (fps, miou),
            textcoords="offset points",
            xytext=(6, 6),
            fontsize=9,
        )

    ax.set_xlabel("Inference speed (FPS)")
    ax.set_ylabel("Best mIoU")
    ax.set_title("Accuracy vs. Speed (best checkpoint per model)")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)


def plot_training_curves(records_by_model: dict[str, list[dict]], out_path: Path, metric: str = "miou"):
    """Line chart: metric vs epoch, one line per model."""
    fig, ax = plt.subplots(figsize=(8, 5))

    for model, records in records_by_model.items():
        epochs = [r.get("epoch") for r in records if metric in r]
        values = [r[metric] for r in records if metric in r]
        if not epochs:
            continue
        ax.plot(epochs, values, marker="o", markersize=3, label=model)

    ax.set_xlabel("Epoch")
    ax.set_ylabel(metric)
    ax.set_title(f"Training curves: {metric} vs. epoch")
    ax.legend()
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)


def plot_per_class_iou(records_by_model: dict[str, list[dict]], out_path: Path):
    """Grouped bar chart: per-class IoU at each model's best checkpoint."""
    # Collect the best-checkpoint per-class IoU for every model.
    per_model_class_iou: dict[str, dict[str, float]] = {}
    all_classes: list[str] = []

    for model, records in records_by_model.items():
        best = best_epoch(records)
        class_iou = best.get("per_class_iou", {})
        if not class_iou:
            continue
        per_model_class_iou[model] = class_iou
        for cls in class_iou:
            if cls not in all_classes:
                all_classes.append(cls)

    if not per_model_class_iou:
        print("No per_class_iou data found — skipping per-class chart.")
        return

    models = list(per_model_class_iou.keys())
    n_models = len(models)
    n_classes = len(all_classes)

    fig, ax = plt.subplots(figsize=(1.6 * n_classes + 2, 5))
    bar_width = 0.8 / n_models
    x = range(n_classes)

    for i, model in enumerate(models):
        values = [per_model_class_iou[model].get(cls, 0) for cls in all_classes]
        offsets = [xi + i * bar_width for xi in x]
        ax.bar(offsets, values, width=bar_width, label=model)

    ax.set_xticks([xi + bar_width * (n_models - 1) / 2 for xi in x])
    ax.set_xticklabels(all_classes)
    ax.set_ylabel("IoU")
    ax.set_title("Per-class IoU (best checkpoint per model)")
    ax.legend()
    ax.grid(True, alpha=0.3, axis="y")
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)


# ---------------------------------------------------------------------------
# Summary table (printed + saved as CSV)
# ---------------------------------------------------------------------------

def write_summary_table(records_by_model: dict[str, list[dict]], out_path: Path):
    rows = []
    for model, records in records_by_model.items():
        best = best_epoch(records)
        latency_ms = best.get("inference_latency_ms")
        fps = round(1000.0 / latency_ms, 1) if latency_ms else None
        rows.append(
            {
                "model": model,
                "best_epoch": best.get("epoch"),
                "miou": round(best.get("miou", 0), 4),
                "fps": fps,
                "latency_ms": round(latency_ms, 1) if latency_ms is not None else None,
                "model_size_mb": best.get("model_size_mb"),
                "avg_epoch_train_time_s": round(
                    sum(r.get("epoch_train_time_s", 0) for r in records) / len(records), 1
                ),
            }
        )

    rows.sort(key=lambda r: r["miou"], reverse=True)

    headers = ["model", "best_epoch", "miou", "fps", "latency_ms", "model_size_mb", "avg_epoch_train_time_s"]
    col_widths = {h: max(len(h), max(len(str(r[h])) for r in rows)) for h in headers}

    def fmt_row(values):
        return " | ".join(str(v).ljust(col_widths[h]) for h, v in zip(headers, values))

    lines = [fmt_row(headers), "-+-".join("-" * col_widths[h] for h in headers)]
    for r in rows:
        lines.append(fmt_row([r[h] for h in headers]))

    table_str = "\n".join(lines)
    print(table_str)

    with open(out_path, "w") as f:
        f.write(",".join(headers) + "\n")
        for r in rows:
            f.write(",".join(str(r[h]) for h in headers) + "\n")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="Aggregate and chart segmentation model benchmarking logs.")
    parser.add_argument("--results-dir", type=Path, default=Path("results"), help="Directory containing one .json file per model (each a JSON array of per-epoch records)")
    parser.add_argument("--out-dir", type=Path, default=Path("report"), help="Directory to write charts + summary CSV to")
    args = parser.parse_args()

    args.out_dir.mkdir(parents=True, exist_ok=True)

    records_by_model = load_logs(args.results_dir)
    print(f"Loaded logs for {len(records_by_model)} model(s): {', '.join(records_by_model)}\n")

    write_summary_table(records_by_model, args.out_dir / "summary.csv")

    plot_accuracy_vs_speed(records_by_model, args.out_dir / "accuracy_vs_speed.png")
    plot_training_curves(records_by_model, args.out_dir / "training_curve_miou.png", metric="miou")
    plot_training_curves(records_by_model, args.out_dir / "training_curve_loss.png", metric="loss")
    plot_per_class_iou(records_by_model, args.out_dir / "per_class_iou.png")

    print(f"\nCharts + summary written to {args.out_dir}/")


if __name__ == "__main__":
    main()
