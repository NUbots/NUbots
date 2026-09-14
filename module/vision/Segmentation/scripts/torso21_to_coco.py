"""Convert TORSO-21 annotations.yaml (bounding boxes) into COCO segmentation
format for RF-DETR.

TORSO-21 only provides boxes for ball/robot/goalpost (no real polygons), so
each box becomes a 4-corner rectangle "segmentation" polygon. Fine for
validating the training pipeline; not true segmentation ground truth.

Usage:
    python torso21_to_coco.py --data-root data/reality/train --out torso21-coco/train
    python torso21_to_coco.py --data-root data/reality/test  --out torso21-coco/valid
"""
import argparse
import json
import shutil
from pathlib import Path

import yaml

CLASSES = ["ball", "goalpost", "robot"]  # extend if you want intersections too
CLASS_TO_ID = {c: i + 1 for i, c in enumerate(CLASSES)}  # COCO category ids start at 1


def box_extent(vector):
    # vector can have 2+ points depending on annotation type — take the
    # bounding extent of whatever points are given, not just assume 2.
    xs = [p[0] for p in vector]
    ys = [p[1] for p in vector]
    return min(xs), min(ys), max(xs), max(ys)


def convert(data_root: Path, out_dir: Path):
    with open(data_root / "annotations.yaml") as f:
        ann = yaml.safe_load(f)

    out_dir.mkdir(parents=True, exist_ok=True)

    images, annotations = [], []
    img_id, ann_id = 1, 1
    n_skipped_objs = 0

    for img_name, meta in ann["images"].items():
        width, height = meta["width"], meta["height"]
        objs = []
        for obj in meta.get("annotations", []):
            if not obj.get("in_image", False):
                continue
            cls = obj.get("type")
            if cls not in CLASS_TO_ID or "vector" not in obj:
                continue
            try:
                x1, y1, x2, y2 = box_extent(obj["vector"])
            except (TypeError, ValueError):
                n_skipped_objs += 1
                continue
            w, h = x2 - x1, y2 - y1
            objs.append({
                "id": ann_id,
                "image_id": img_id,
                "category_id": CLASS_TO_ID[cls],
                "segmentation": [[x1, y1, x2, y1, x2, y2, x1, y2]],
                "bbox": [x1, y1, w, h],
                "area": w * h,
                "iscrowd": 0,
            })
            ann_id += 1

        if not objs:
            continue  # skip images with none of our target classes

        src_img = data_root / "images" / img_name
        if not src_img.exists():
            continue
        shutil.copy(src_img, out_dir / img_name)

        images.append({"id": img_id, "file_name": img_name, "width": width, "height": height})
        annotations.extend(objs)
        img_id += 1

    coco = {
        "images": images,
        "annotations": annotations,
        "categories": [{"id": v, "name": k, "supercategory": "none"} for k, v in CLASS_TO_ID.items()],
    }
    with open(out_dir / "_annotations.coco.json", "w") as f:
        json.dump(coco, f)

    print(f"Converted {len(images)} images, {len(annotations)} annotations to {out_dir}")
    if n_skipped_objs:
        print(f"Skipped {n_skipped_objs} malformed object annotations")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--data-root", required=True, type=Path, help="e.g. data/reality/train")
    parser.add_argument("--out", required=True, type=Path, help="e.g. torso21-coco/train")
    args = parser.parse_args()
    convert(args.data_root, args.out)


if __name__ == "__main__":
    main()
