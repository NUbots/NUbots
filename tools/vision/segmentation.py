#!/usr/bin/env python3
#
# MIT License
#
# Copyright (c) 2026 NUbots
#
# This file is part of the NUbots codebase.
# See https://github.com/NUbots/NUbots for further info.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#
"""Single-file segmentation pipeline: train, eval and ONNX export.

The model (`FieldSegNet`) is a compact depthwise-separable U-Net designed to run
in real time on the robot CPU via OpenVINO. It predicts three classes:

    0 -> field        (grey intensity ~255)
    1 -> field_line   (grey intensity ~127)
    2 -> background   (grey intensity ~0)

Masks in the dataset are (anti-aliased) greyscale, so the intensity of each pixel
is quantised to the nearest of the three class anchors.

Preprocessing matches the C++ inference module (module/vision/Segmentation):
images are RGB, scaled to [0, 1] and normalised with the ImageNet mean/std. The
exported ONNX takes a [1, 3, H, W] tensor named "input" and returns a
[1, num_classes, H, W] logit tensor named "output".

Usage:
    ./b vision segmentation train  --data_dir datasets/torso21 --gpus all
    ./b vision segmentation eval   --data_dir datasets/torso21
    ./b vision segmentation export
"""

import os
import random

import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
from PIL import Image, ImageEnhance
from torch.utils.data import DataLoader, Dataset
from tqdm import tqdm

from utility.dockerise import run_on_docker

# Class definitions shared with the C++ module (module/vision/Segmentation).
CLASS_NAMES = ["field", "field_line", "background"]
NUM_CLASSES = len(CLASS_NAMES)

# ImageNet normalisation (must match the C++ preprocessing).
IMAGENET_MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32)
IMAGENET_STD = np.array([0.229, 0.224, 0.225], dtype=np.float32)


# -----------------------------------------------------------------------------
#  Model: FieldSegNet - a depthwise-separable U-Net
# -----------------------------------------------------------------------------
class DSConv(nn.Module):
    """Depthwise-separable convolution (3x3 depthwise + 1x1 pointwise)."""

    def __init__(self, in_ch: int, out_ch: int):
        super().__init__()
        self.depthwise = nn.Conv2d(in_ch, in_ch, 3, padding=1, groups=in_ch, bias=False)
        self.pointwise = nn.Conv2d(in_ch, out_ch, 1, bias=False)
        self.bn = nn.BatchNorm2d(out_ch)
        self.act = nn.ReLU(inplace=True)

    def forward(self, x):
        return self.act(self.bn(self.pointwise(self.depthwise(x))))


class DSBlock(nn.Module):
    """Two stacked depthwise-separable convolutions."""

    def __init__(self, in_ch: int, out_ch: int):
        super().__init__()
        self.conv1 = DSConv(in_ch, out_ch)
        self.conv2 = DSConv(out_ch, out_ch)

    def forward(self, x):
        return self.conv2(self.conv1(x))


class FieldSegNet(nn.Module):
    """Compact U-Net with a depthwise-separable backbone.

    Four 2x downsampling stages with skip connections and bilinear upsampling.
    Input and output share the same spatial resolution, so a 512x512 input yields
    a [B, num_classes, 512, 512] logit map.
    """

    def __init__(self, in_channels: int = 3, num_classes: int = NUM_CLASSES, base_ch: int = 24):
        super().__init__()
        c1, c2, c3, c4 = base_ch, base_ch * 2, base_ch * 4, base_ch * 8

        # Encoder
        self.enc0 = DSBlock(in_channels, c1)  # full res
        self.enc1 = DSBlock(c1, c2)  # /2
        self.enc2 = DSBlock(c2, c3)  # /4
        self.enc3 = DSBlock(c3, c4)  # /8
        self.bottleneck = DSBlock(c4, c4)  # /16
        self.pool = nn.MaxPool2d(2)

        # Decoder (in_ch accounts for the concatenated skip connection)
        self.dec3 = DSBlock(c4 + c4, c3)
        self.dec2 = DSBlock(c3 + c3, c2)
        self.dec1 = DSBlock(c2 + c2, c1)
        self.dec0 = DSBlock(c1 + c1, c1)

        self.head = nn.Conv2d(c1, num_classes, 1)

    @staticmethod
    def _up(x):
        return F.interpolate(x, scale_factor=2, mode="bilinear", align_corners=False)

    def forward(self, x):
        s0 = self.enc0(x)  # full res
        s1 = self.enc1(self.pool(s0))  # /2
        s2 = self.enc2(self.pool(s1))  # /4
        s3 = self.enc3(self.pool(s2))  # /8
        b = self.bottleneck(self.pool(s3))  # /16

        d3 = self.dec3(torch.cat([self._up(b), s3], dim=1))  # /8
        d2 = self.dec2(torch.cat([self._up(d3), s2], dim=1))  # /4
        d1 = self.dec1(torch.cat([self._up(d2), s1], dim=1))  # /2
        d0 = self.dec0(torch.cat([self._up(d1), s0], dim=1))  # full res

        return self.head(d0)


# -----------------------------------------------------------------------------
#  Dataset
# -----------------------------------------------------------------------------
def mask_to_class(mask_rgb: np.ndarray) -> np.ndarray:
    """Quantise a (near-)greyscale mask to class indices.

    Anchors: background(2)=0, field_line(1)=127, field(0)=255. Midpoints between
    anchors are 63.5 and 191.
    """
    intensity = mask_rgb.mean(axis=2)
    class_mask = np.full(intensity.shape, 2, dtype=np.int64)  # background by default
    class_mask[intensity >= 191] = 0  # field
    class_mask[(intensity >= 63.5) & (intensity < 191)] = 1  # field lines
    return class_mask


class SegmentationDataset(Dataset):
    def __init__(self, img_dir: str, mask_dir: str, img_size: int = 512, augment: bool = False):
        self.img_dir = img_dir
        self.mask_dir = mask_dir
        self.img_size = img_size
        self.augment = augment
        self.images = sorted(f for f in os.listdir(img_dir) if f.endswith((".jpg", ".png")))
        if not self.images:
            raise FileNotFoundError(f"No images found in {img_dir}")

    def __len__(self):
        return len(self.images)

    def _find_mask(self, img_name: str) -> str:
        base, ext = os.path.splitext(img_name)
        for candidate in (img_name, f"{base}.png", f"{base}.jpg"):
            path = os.path.join(self.mask_dir, candidate)
            if os.path.exists(path):
                return path
        raise FileNotFoundError(f"No matching mask for image: {img_name}")

    def _augment(self, image: Image.Image, mask: Image.Image):
        if random.random() > 0.5:
            image = image.transpose(Image.FLIP_LEFT_RIGHT)
            mask = mask.transpose(Image.FLIP_LEFT_RIGHT)
        if random.random() > 0.5:
            angle = random.uniform(-12, 12)
            image = image.rotate(angle, Image.BILINEAR)
            mask = mask.rotate(angle, Image.NEAREST)
        if random.random() > 0.5:
            image = ImageEnhance.Brightness(image).enhance(random.uniform(0.75, 1.25))
        if random.random() > 0.5:
            image = ImageEnhance.Contrast(image).enhance(random.uniform(0.75, 1.25))
        return image, mask

    def __getitem__(self, idx: int):
        img_name = self.images[idx]
        image = Image.open(os.path.join(self.img_dir, img_name)).convert("RGB")
        mask = Image.open(self._find_mask(img_name)).convert("RGB")

        image = image.resize((self.img_size, self.img_size), Image.BILINEAR)
        mask = mask.resize((self.img_size, self.img_size), Image.NEAREST)

        if self.augment:
            image, mask = self._augment(image, mask)

        # Image -> normalised CHW float tensor (RGB order, ImageNet stats).
        img_np = np.asarray(image, dtype=np.float32) / 255.0
        img_np = (img_np - IMAGENET_MEAN) / IMAGENET_STD
        image_tensor = torch.from_numpy(img_np.transpose(2, 0, 1)).contiguous()

        class_mask = mask_to_class(np.asarray(mask))
        mask_tensor = torch.from_numpy(class_mask)

        return image_tensor, mask_tensor


# -----------------------------------------------------------------------------
#  Losses
# -----------------------------------------------------------------------------
class DiceLoss(nn.Module):
    """Soft multi-class Dice loss (handles thin structures like field lines)."""

    def __init__(self, num_classes: int = NUM_CLASSES, eps: float = 1.0):
        super().__init__()
        self.num_classes = num_classes
        self.eps = eps

    def forward(self, logits, target):
        probs = F.softmax(logits, dim=1)
        target_1h = F.one_hot(target, self.num_classes).permute(0, 3, 1, 2).float()
        dims = (0, 2, 3)
        intersection = (probs * target_1h).sum(dims)
        cardinality = probs.sum(dims) + target_1h.sum(dims)
        dice = (2.0 * intersection + self.eps) / (cardinality + self.eps)
        return 1.0 - dice.mean()


class CombinedLoss(nn.Module):
    """Weighted cross-entropy + Dice."""

    def __init__(self, class_weights=None, ce_weight: float = 1.0, dice_weight: float = 1.0):
        super().__init__()
        self.ce = nn.CrossEntropyLoss(weight=class_weights)
        self.dice = DiceLoss()
        self.ce_weight = ce_weight
        self.dice_weight = dice_weight

    def forward(self, logits, target):
        return self.ce_weight * self.ce(logits, target) + self.dice_weight * self.dice(logits, target)


def estimate_class_weights(dataset: SegmentationDataset, sample: int = 300, cap: float = 8.0) -> torch.Tensor:
    """Median-frequency balancing estimated from a sample of masks."""
    counts = np.zeros(NUM_CLASSES, dtype=np.float64)
    indices = list(range(len(dataset)))
    random.shuffle(indices)
    indices = indices[: min(sample, len(indices))]
    for i in tqdm(indices, desc="Estimating class weights"):
        img_name = dataset.images[i]
        mask = Image.open(dataset._find_mask(img_name)).convert("RGB")
        mask = mask.resize((dataset.img_size, dataset.img_size), Image.NEAREST)
        cm = mask_to_class(np.asarray(mask))
        binc = np.bincount(cm.ravel(), minlength=NUM_CLASSES)
        counts += binc

    freq = counts / max(counts.sum(), 1.0)
    freq = np.clip(freq, 1e-6, None)
    weights = np.median(freq) / freq
    weights = np.clip(weights, 1.0 / cap, cap)
    return torch.tensor(weights, dtype=torch.float32)


# -----------------------------------------------------------------------------
#  Metrics
# -----------------------------------------------------------------------------
def confusion_matrix(preds: np.ndarray, target: np.ndarray, num_classes: int = NUM_CLASSES) -> np.ndarray:
    k = (target >= 0) & (target < num_classes)
    idx = num_classes * target[k].astype(np.int64) + preds[k].astype(np.int64)
    return np.bincount(idx, minlength=num_classes * num_classes).reshape(num_classes, num_classes)


def metrics_from_confusion(conf: np.ndarray):
    intersection = np.diag(conf).astype(np.float64)
    true_sum = conf.sum(axis=1)
    pred_sum = conf.sum(axis=0)
    union = true_sum + pred_sum - intersection
    iou = intersection / np.maximum(union, 1)
    precision = intersection / np.maximum(pred_sum, 1)
    recall = intersection / np.maximum(true_sum, 1)
    pixel_acc = intersection.sum() / max(conf.sum(), 1)
    return {
        "iou": iou,
        "precision": precision,
        "recall": recall,
        "support": true_sum,
        "pixel_acc": float(pixel_acc),
        "mean_iou": float(iou.mean()),
    }


def print_metrics(m):
    print(f"  pixel_acc = {m['pixel_acc']:.4f}   mean_iou = {m['mean_iou']:.4f}")
    for i, name in enumerate(CLASS_NAMES):
        print(
            f"  {name:11s} IoU={m['iou'][i]:.4f}  P={m['precision'][i]:.4f}  "
            f"R={m['recall'][i]:.4f}  support={int(m['support'][i])}"
        )


@torch.no_grad()
def evaluate_model(model, loader, device, num_classes: int = NUM_CLASSES):
    model.eval()
    conf = np.zeros((num_classes, num_classes), dtype=np.int64)
    for images, masks in tqdm(loader, desc="Evaluating"):
        images = images.to(device)
        logits = model(images)
        preds = torch.argmax(logits, dim=1).cpu().numpy().ravel()
        conf += confusion_matrix(preds, masks.numpy().ravel(), num_classes)
    return metrics_from_confusion(conf)


# -----------------------------------------------------------------------------
#  Helpers
# -----------------------------------------------------------------------------
def build_model(base_ch: int, device) -> FieldSegNet:
    model = FieldSegNet(in_channels=3, num_classes=NUM_CLASSES, base_ch=base_ch)
    return model.to(device)


def resolve_dirs(data_dir, train_img_dir, train_mask_dir, test_img_dir, test_mask_dir):
    train_img_dir = train_img_dir or os.path.join(data_dir, "train", "images")
    train_mask_dir = train_mask_dir or os.path.join(data_dir, "train", "segmentations")
    test_img_dir = test_img_dir or os.path.join(data_dir, "test", "images")
    test_mask_dir = test_mask_dir or os.path.join(data_dir, "test", "segmentations")
    return train_img_dir, train_mask_dir, test_img_dir, test_mask_dir


def export_onnx(model, onnx_path, img_size, device):
    os.makedirs(os.path.dirname(onnx_path) or ".", exist_ok=True)
    model.eval()
    dummy = torch.randn(1, 3, img_size, img_size, device=device)
    torch.onnx.export(
        model,
        dummy,
        onnx_path,
        export_params=True,
        opset_version=11,
        input_names=["input"],
        output_names=["output"],
        dynamic_axes={"input": {0: "batch_size"}, "output": {0: "batch_size"}},
        dynamo=False,
    )
    print(f"Exported ONNX model to {onnx_path}")


# -----------------------------------------------------------------------------
#  Modes
# -----------------------------------------------------------------------------
def do_train(args, device):
    train_img_dir, train_mask_dir, test_img_dir, test_mask_dir = resolve_dirs(
        args.data_dir, args.train_img_dir, args.train_mask_dir, args.test_img_dir, args.test_mask_dir
    )

    train_ds = SegmentationDataset(train_img_dir, train_mask_dir, img_size=args.img_size, augment=True)
    val_ds = SegmentationDataset(test_img_dir, test_mask_dir, img_size=args.img_size, augment=False)
    print(f"Train images: {len(train_ds)}   Val images: {len(val_ds)}")

    train_loader = DataLoader(
        train_ds,
        batch_size=args.batch_size,
        shuffle=True,
        num_workers=args.num_workers,
        pin_memory=(device.type == "cuda"),
        drop_last=True,
    )
    val_loader = DataLoader(
        val_ds,
        batch_size=args.batch_size,
        shuffle=False,
        num_workers=args.num_workers,
        pin_memory=(device.type == "cuda"),
    )

    model = build_model(args.base_channels, device)
    n_params = sum(p.numel() for p in model.parameters())
    print(f"Model: FieldSegNet(base_ch={args.base_channels})  parameters={n_params / 1e6:.2f}M")

    class_weights = estimate_class_weights(train_ds, sample=args.weight_sample).to(device)
    print("Class weights:", {CLASS_NAMES[i]: round(float(class_weights[i]), 3) for i in range(NUM_CLASSES)})

    criterion = CombinedLoss(class_weights=class_weights, ce_weight=args.ce_weight, dice_weight=args.dice_weight)
    optimizer = torch.optim.AdamW(model.parameters(), lr=args.lr, weight_decay=args.weight_decay)
    scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(optimizer, T_max=args.epochs)

    use_amp = device.type == "cuda"
    scaler = torch.amp.GradScaler("cuda", enabled=use_amp)

    os.makedirs(os.path.dirname(args.model_path) or ".", exist_ok=True)
    best_miou = -1.0

    for epoch in range(args.epochs):
        model.train()
        running = 0.0
        for images, masks in tqdm(train_loader, desc=f"Epoch {epoch + 1}/{args.epochs} - train"):
            images = images.to(device, non_blocking=True)
            masks = masks.to(device, non_blocking=True)

            optimizer.zero_grad(set_to_none=True)
            with torch.amp.autocast("cuda", enabled=use_amp):
                logits = model(images)
                loss = criterion(logits, masks)
            scaler.scale(loss).backward()
            scaler.step(optimizer)
            scaler.update()
            running += loss.item() * images.size(0)

        scheduler.step()
        train_loss = running / len(train_loader.dataset)

        metrics = evaluate_model(model, val_loader, device)
        print(f"Epoch {epoch + 1}/{args.epochs}: train_loss={train_loss:.4f}")
        print_metrics(metrics)

        if metrics["mean_iou"] > best_miou:
            best_miou = metrics["mean_iou"]
            torch.save({"state_dict": model.state_dict(), "base_ch": args.base_channels}, args.model_path)
            print(f"  -> saved best model (mean_iou={best_miou:.4f}) to {args.model_path}")

    print(f"Training finished. Best mean_iou={best_miou:.4f}")

    if args.export:
        # Reload best weights before exporting.
        load_checkpoint(model, args.model_path, device)
        export_onnx(model, args.onnx_path, args.img_size, device)


def do_eval(args, device):
    _, _, test_img_dir, test_mask_dir = resolve_dirs(
        args.data_dir, args.train_img_dir, args.train_mask_dir, args.test_img_dir, args.test_mask_dir
    )
    val_ds = SegmentationDataset(test_img_dir, test_mask_dir, img_size=args.img_size, augment=False)
    if args.max_eval_images:
        val_ds.images = val_ds.images[: args.max_eval_images]
    print(f"Eval images: {len(val_ds)}")

    val_loader = DataLoader(
        val_ds,
        batch_size=args.batch_size,
        shuffle=False,
        num_workers=args.num_workers,
        pin_memory=(device.type == "cuda"),
    )

    model = build_model(args.base_channels, device)
    load_checkpoint(model, args.model_path, device)

    metrics = evaluate_model(model, val_loader, device)
    print("Evaluation results:")
    print_metrics(metrics)


def do_export(args, device):
    model = build_model(args.base_channels, device)
    load_checkpoint(model, args.model_path, device)
    export_onnx(model, args.onnx_path, args.img_size, device)


def load_checkpoint(model, path, device):
    if not os.path.exists(path):
        raise FileNotFoundError(f"Checkpoint not found: {path}")
    ckpt = torch.load(path, map_location=device)
    state = ckpt["state_dict"] if isinstance(ckpt, dict) and "state_dict" in ckpt else ckpt
    model.load_state_dict(state)
    print(f"Loaded checkpoint from {path}")


# -----------------------------------------------------------------------------
#  b tool entry points
# -----------------------------------------------------------------------------
@run_on_docker
def register(parser):
    parser.description = "Train, evaluate or export the field segmentation model."
    parser.add_argument("mode", choices=["train", "eval", "export"], help="Operation to perform")

    # Data
    parser.add_argument("--data_dir", type=str, default="datasets/torso21", help="Dataset root")
    parser.add_argument("--train_img_dir", type=str, default=None, help="Override training images dir")
    parser.add_argument("--train_mask_dir", type=str, default=None, help="Override training masks dir")
    parser.add_argument("--test_img_dir", type=str, default=None, help="Override test images dir")
    parser.add_argument("--test_mask_dir", type=str, default=None, help="Override test masks dir")

    # Model / IO
    parser.add_argument("--img_size", type=int, default=512, help="Input image size")
    parser.add_argument("--base_channels", type=int, default=24, help="Base channel width of FieldSegNet")
    parser.add_argument(
        "--model_path",
        type=str,
        default="recordings/torso21/field_seg.pth",
        help="Path to save/load the PyTorch checkpoint",
    )
    parser.add_argument(
        "--onnx_path",
        type=str,
        default="module/vision/Segmentation/data/segmentation_model.onnx",
        help="Path to write the exported ONNX model",
    )

    # Training
    parser.add_argument("--batch_size", type=int, default=16, help="Batch size")
    parser.add_argument("--epochs", type=int, default=40, help="Number of training epochs")
    parser.add_argument("--lr", type=float, default=1e-3, help="Initial learning rate")
    parser.add_argument("--weight_decay", type=float, default=1e-4, help="Optimizer weight decay")
    parser.add_argument("--num_workers", type=int, default=4, help="DataLoader workers (use 0 if Docker shm is small)")
    parser.add_argument("--ce_weight", type=float, default=1.0, help="Cross-entropy loss weight")
    parser.add_argument("--dice_weight", type=float, default=1.0, help="Dice loss weight")
    parser.add_argument("--weight_sample", type=int, default=300, help="Masks sampled for class-weight estimation")
    parser.add_argument("--export", action="store_true", help="Export ONNX after training")

    # Eval
    parser.add_argument("--max_eval_images", type=int, default=0, help="Limit eval images (0 = all)")
    parser.add_argument("--seed", type=int, default=42, help="Random seed")


@run_on_docker
def run(mode, seed, **kwargs):
    import argparse

    args = argparse.Namespace(mode=mode, seed=seed, **kwargs)

    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"Using device: {device}")

    if mode == "train":
        do_train(args, device)
    elif mode == "eval":
        do_eval(args, device)
    elif mode == "export":
        do_export(args, device)
