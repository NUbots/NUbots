# NUBots - New Segmentation Model

## Comparative Analysis

### Download data/ && models/

The following is the arg convention for downloading new models

```
./scripts/add_models.sh <URL> <DESTINATION>
```

The following will ensure that the data downloader helper &  script has the correct permissions before running. It is recommended to run the following before continuing.
Note: ~11gb of data will downloaded

```
chmod 777 scripts/ensure_dataset.sh
chmod 777 scripts/add_models.sh
./scripts/ensure_dataset.sh
./scripts/add_models.sh https://github.com/ultralytics/ultralytics ultralytics
./scripts/add_models.sh https://github.com/roboflow/rf-detr rf-detr
./scripts/add_models.sh https://github.com/PaddlePaddle/PaddleSeg pp-lite-seg
./scripts/add_models.sh https://github.com/CoinCheung/BiSeNet bisenetv2
```




# TORSO-21 → YOLO-seg: Convert & Run

## 1. Download TORSO-21

git clone https://github.com/bit-bots/TORSO_21_dataset.git
cd TORSO_21_dataset
poetry install --without=dev --no-root
./scripts/download_dataset.py --all

This gives you `data/reality/train/` and `data/reality/test/`, each with an
`annotations.yaml` and an `images/` folder.

## 2. Convert to YOLO-seg format

TORSO-21's native labels are bounding boxes (not polygons) for ball/robot/goalpost.
`torso21_to_yolo.py` converts each box into a 4-corner rectangle polygon — good
enough to validate the pipeline, not true segmentation ground truth.

Run it once per split — **train and test/val**:

python3 torso21_to_yolo.py --data-root data/reality/train --out torso21-yolo/train
python3 torso21_to_yolo.py --data-root data/reality/test  --out torso21-yolo/val

Each run prints how many images were converted. If it's 0, check that
`--data-root` points at a folder containing `annotations.yaml` + `images/`.

Result:
torso21-yolo/
├── train/
│   ├── images/
│   └── labels/
└── val/
    ├── images/
    └── labels/

## 3. Write data.yaml

Use an **absolute path** — relative paths are where most errors come from.

# torso21-yolo/data.yaml
path: /absolute/path/to/torso21-yolo
train: train/images
val: val/images
names:
  0: ball
  1: goalpost
  2: robot

Sanity check before training:
ls torso21-yolo/train/images torso21-yolo/train/labels
ls torso21-yolo/val/images   torso21-yolo/val/labels

Both train and val need non-empty images/ and labels/.

## 4. Basic test run (pipeline check, not real training)

Low epoch count — this just confirms training/logging/checkpointing work end to end:

python3 yolo11-seg-train.py train --data /absolute/path/to/torso21-yolo/data.yaml --epochs 3 --imgsz 640
python3 yolo26-seg-train.py train --data /absolute/path/to/torso21-yolo/data.yaml --epochs 3 --imgsz 640

Check afterward:
cat results/yolo11_seg.json
cat results/yolo26_seg.json

Should show exactly 3 epoch records each, and checkpoints/*_best.pt should exist.

## 5. Real training run

Once the basic test is clean, bump the epoch count to your actual budget:

python3 yolo11-seg-train.py train --data /absolute/path/to/torso21-yolo/data.yaml --epochs 30 --imgsz 640
python3 yolo26-seg-train.py train --data /absolute/path/to/torso21-yolo/data.yaml --epochs 30 --imgsz 640

## 6. Validate / test a trained checkpoint

python3 yolo11-seg-train.py test --weights checkpoints/yolo11_seg_best.pt --data /absolute/path/to/torso21-yolo/data.yaml

Prints mAP and measured inference latency/FPS.

## 7. Qualitative inference on a fixed image set

Use the **same images** across both models for a fair visual comparison:

python3 yolo11-seg-train.py infer --weights checkpoints/yolo11_seg_best.pt --images img1.jpg img2.jpg img3.jpg
python3 yolo26-seg-train.py infer --weights checkpoints/yolo26_seg_best.pt --images img1.jpg img2.jpg img3.jpg

Saves prediction overlays to inference_samples/.



# TORSO-21 → YOLO-seg / RF-DETR: Convert & Run

## 1. Download TORSO-21

git clone https://github.com/bit-bots/TORSO_21_dataset.git
cd TORSO_21_dataset
poetry install --without=dev --no-root
./scripts/download_dataset.py --all

This gives you `data/reality/train/` and `data/reality/test/`, each with an
`annotations.yaml` and an `images/` folder.

---

## 2A. Convert to YOLO-seg format

TORSO-21's native labels are bounding boxes (not polygons) for ball/robot/goalpost.
`torso21_to_yolo.py` converts each box into a 4-corner rectangle polygon — good
enough to validate the pipeline, not true segmentation ground truth.

python3 torso21_to_yolo.py --data-root data/reality/train --out torso21-yolo/train
python3 torso21_to_yolo.py --data-root data/reality/test  --out torso21-yolo/val

Result:
torso21-yolo/
├── train/
│   ├── images/
│   └── labels/
└── val/
    ├── images/
    └── labels/

Write data.yaml (use an absolute path):

# torso21-yolo/data.yaml
path: /absolute/path/to/torso21-yolo
train: train/images
val: val/images
names:
  0: ball
  1: goalpost
  2: robot

Sanity check before training:
ls torso21-yolo/train/images torso21-yolo/train/labels
ls torso21-yolo/val/images   torso21-yolo/val/labels

---

## 2B. Convert to COCO format (for RF-DETR)

RF-DETR expects COCO segmentation format instead. `torso21_to_coco.py` does
the same box-to-rectangle-polygon conversion, just writing COCO JSON.

Important: RF-DETR requires the validation folder to be named exactly
"valid", not "val". This is a Roboflow export convention - get this wrong
and you'll hit "Could not detect dataset format" even though everything else
is correct.

python3 torso21_to_coco.py --data-root data/reality/train --out torso21-coco/train
python3 torso21_to_coco.py --data-root data/reality/test  --out torso21-coco/valid

Result:
torso21-coco/
├── train/
│   ├── _annotations.coco.json
│   └── *.png (images)
└── valid/
    ├── _annotations.coco.json
    └── *.png (images)

Sanity check:
ls torso21-coco/train/ torso21-coco/valid/

---

## 3A. Basic test run - YOLO-seg (pipeline check, not real training)

Low epoch count - this just confirms training/logging/checkpointing work end to end:

python3
