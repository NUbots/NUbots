"""Convert TORSO-21 annotations.yaml (bounding boxes) into YOLO-seg labels.

TORSO-21 only provides boxes for ball/robot/goalpost (no real polygons), so
each box becomes a 4-corner rectangle polygon. Fine for validating the
training pipeline; not real segmentation ground truth.

Usage:
    python torso21_to_yolo.py --data-root data/reality/train --out images_labels/train
"""
import argparse
import shutil
from pathlib import Path

import yaml

CLASSES = ["ball", "goalpost", "robot"]  # extend if you want intersections too
CLASS_TO_ID = {c: i for i, c in enumerate(CLASSES)}


def box_to_polygon(vector, width, height):
    # vector can have 2+ points depending on annotation type (a plain box is
    # 2 corner points, but some types store more) — take the bounding extent
    # of whatever points are given, rather than assuming exactly 2.
    xs_px = [p[0] for p in vector]
    ys_px = [p[1] for p in vector]
    x1, x2 = min(xs_px), max(xs_px)
    y1, y2 = min(ys_px), max(ys_px)
    xs = [x1, x2, x2, x1]
    ys = [y1, y1, y2, y2]
    return [x / width for x in xs], [y / height for y in ys]


def convert(data_root: Path, out_dir: Path):
    with open(data_root / "annotations.yaml") as f:
        ann = yaml.safe_load(f)

    images_out = out_dir / "images"
    labels_out = out_dir / "labels"
    images_out.mkdir(parents=True, exist_ok=True)
    labels_out.mkdir(parents=True, exist_ok=True)

    n_written = 0
    n_skipped_objs = 0
    for img_name, meta in ann["images"].items():
        width, height = meta["width"], meta["height"]
        lines = []
        for obj in meta.get("annotations", []):
            if not obj.get("in_image", False):
                continue
            cls = obj.get("type")
            if cls not in CLASS_TO_ID or "vector" not in obj:
                continue
            try:
                xs, ys = box_to_polygon(obj["vector"], width, height)
            except (TypeError, ValueError):
                n_skipped_objs += 1
                continue
            coords = " ".join(f"{x:.6f} {y:.6f}" for x, y in zip(xs, ys))
            lines.append(f"{CLASS_TO_ID[cls]} {coords}")

        if not lines:
            continue  # skip images with none of our target classes

        src_img = data_root / "images" / img_name
        if not src_img.exists():
            continue
        shutil.copy(src_img, images_out / img_name)
        (labels_out / f"{Path(img_name).stem}.txt").write_text("\n".join(lines))
        n_written += 1

    print(f"Converted {n_written} images to {out_dir}")
    if n_skipped_objs:
        print(f"Skipped {n_skipped_objs} malformed object annotations")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--data-root", required=True, type=Path, help="e.g. data/reality/train")
    parser.add_argument("--out", required=True, type=Path)
    args = parser.parse_args()
    convert(args.data_root, args.out)


if __name__ == "__main__":
    main()
