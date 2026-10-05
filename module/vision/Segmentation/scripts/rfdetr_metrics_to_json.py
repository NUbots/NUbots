"""Convert RF-DETR's Lightning metrics.csv into results/rfdetr.json.

RF-DETR logs per-epoch metrics via PyTorch Lightning's CSVLogger directly into
your --output-dir (e.g. checkpoints/rfdetr_run/metrics.csv) — there is no
results/<model_name>.json written automatically like the YOLO scripts produce.
This script reads that CSV and writes the JSON array your graphing scripts
(graph_model.py, benchmark_report.py) expect.

Key columns used (confirmed from RF-DETR's source):
    epoch
    val/mAP_50_95        — box mAP (segm_mAP_50_95 used instead if segmentation=True)
    val/segm_mAP_50_95   — mask mAP, used preferentially for segmentation models
    train/loss

Usage:
    python rfdetr_metrics_to_json.py --metrics-csv checkpoints/rfdetr_run/metrics.csv \
        --weights checkpoints/rfdetr_run/checkpoint_best_total.pth \
        --model-name rfdetr --results-dir results
"""
import argparse
import json
from pathlib import Path

import pandas as pd


def convert(metrics_csv: Path, weights: Path | None, model_name: str, results_dir: Path):
    df = pd.read_csv(metrics_csv)

    # Lightning logs multiple rows per epoch (step-level + epoch-level); keep
    # only rows that actually have a validation mAP logged, then take the
    # last (most complete) row per epoch.
    miou_col = "val/segm_mAP_50_95" if "val/segm_mAP_50_95" in df.columns else "val/mAP_50_95"
    if miou_col not in df.columns:
        raise ValueError(
            f"Neither 'val/segm_mAP_50_95' nor 'val/mAP_50_95' found in {metrics_csv}. "
            f"Columns present: {list(df.columns)}"
        )

    val_rows = df[df[miou_col].notna()].copy()
    val_rows = val_rows.groupby("epoch", as_index=False).last()

    records = []
    for _, row in val_rows.iterrows():
        records.append({
            "model": model_name,
            "epoch": int(row["epoch"]) + 1,  # RF-DETR epochs are 0-indexed like Ultralytics
            "miou": round(float(row[miou_col]), 4),
            "per_class_iou": {},
            "loss": round(float(row["train/loss"]), 4) if "train/loss" in row and pd.notna(row["train/loss"]) else None,
            "inference_latency_ms": None,
            "model_size_mb": None,
            "epoch_train_time_s": None,  # not logged by RF-DETR's CSV logger
        })

    if weights and weights.exists() and records:
        records[-1]["model_size_mb"] = round(weights.stat().st_size / (1024 * 1024), 2)

    results_dir.mkdir(parents=True, exist_ok=True)
    out_path = results_dir / f"{model_name.replace('-', '_')}.json"
    with open(out_path, "w") as f:
        json.dump(records, f, indent=2)
    print(f"Wrote {len(records)} epoch records to {out_path}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--metrics-csv", required=True, type=Path)
    parser.add_argument("--weights", type=Path, default=None, help="checkpoint_best_total.pth, for model size")
    parser.add_argument("--model-name", default="rfdetr")
    parser.add_argument("--results-dir", default=Path("results"), type=Path)
    args = parser.parse_args()
    convert(args.metrics_csv, args.weights, args.model_name, args.results_dir)


if __name__ == "__main__":
    main()
