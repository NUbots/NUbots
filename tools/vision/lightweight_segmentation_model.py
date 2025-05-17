import os

import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
from PIL import Image
from torch.utils.data import DataLoader, Dataset
from torchvision import transforms
from tqdm import tqdm

# -----------------------------------------------------------------------------
#  Model: SE‑UNet‑Lite (replaces the previous small encoder–decoder)
# -----------------------------------------------------------------------------

class SEBlock(nn.Module):
    """Squeeze‑and‑Excitation recalibration."""

    def __init__(self, channels: int, reduction: int = 16):
        super().__init__()
        self.squeeze = nn.AdaptiveAvgPool2d(1)
        self.excite = nn.Sequential(
            nn.Conv2d(channels, channels // reduction, 1, bias=False),
            nn.ReLU(inplace=True),
            nn.Conv2d(channels // reduction, channels, 1, bias=False),
            nn.Sigmoid(),
        )

    def forward(self, x):
        w = self.squeeze(x)
        w = self.excite(w)
        return x * w


def double_conv(in_ch: int, out_ch: int) -> nn.Sequential:
    return nn.Sequential(
        nn.Conv2d(in_ch, out_ch, 3, padding=1, bias=False),
        nn.BatchNorm2d(out_ch),
        nn.ReLU(inplace=True),
        nn.Conv2d(out_ch, out_ch, 3, padding=1, bias=False),
        nn.BatchNorm2d(out_ch),
        nn.ReLU(inplace=True),
    )


class DownBlock(nn.Module):
    def __init__(self, in_ch: int, out_ch: int):
        super().__init__()
        self.conv = double_conv(in_ch, out_ch)
        self.se = SEBlock(out_ch)
        self.pool = nn.MaxPool2d(2)

    def forward(self, x):
        x = self.se(self.conv(x))
        p = self.pool(x)
        return x, p


class UpBlock(nn.Module):
    def __init__(self, in_ch: int, skip_ch: int, out_ch: int, use_transpose: bool = True):
        super().__init__()
        if use_transpose:
            self.up = nn.ConvTranspose2d(in_ch, in_ch // 2, 2, stride=2)
        else:
            self.up = nn.Sequential(
                nn.Upsample(scale_factor=2, mode="nearest"),
                nn.Conv2d(in_ch, in_ch // 2, 1, bias=False),
            )
        self.conv = double_conv(in_ch // 2 + skip_ch, out_ch)
        self.se = SEBlock(out_ch)

    def forward(self, x, skip):
        x = self.up(x)
        # pad to handle odd input dims
        if x.shape[-2:] != skip.shape[-2:]:
            diffY = skip.shape[-2] - x.shape[-2]
            diffX = skip.shape[-1] - x.shape[-1]
            x = F.pad(x, [diffX // 2, diffX - diffX // 2, diffY // 2, diffY - diffY // 2])
        x = torch.cat([skip, x], dim=1)
        x = self.se(self.conv(x))
        return x


class LightSegmentationModel(nn.Module):
    """SE‑UNet‑Lite with 3 encoder/decoder levels.

    Keeping the original class name so the downstream training code remains unchanged.
    """

    def __init__(self, in_channels: int = 3, num_classes: int = 3, base_ch: int = 32):
        super().__init__()
        b1, b2, b3 = base_ch, base_ch * 2, base_ch * 4

        self.down1 = DownBlock(in_channels, b1)
        self.down2 = DownBlock(b1, b2)
        self.down3 = DownBlock(b2, b3)

        self.bottleneck = nn.Sequential(
            double_conv(b3, b3 * 2),
            SEBlock(b3 * 2),
        )

        self.up3 = UpBlock(b3 * 2, b3, b2)
        self.up2 = UpBlock(b2, b2, b1)
        self.up1 = UpBlock(b1, b1, b1 // 2)

        self.head = nn.Conv2d(b1 // 2, num_classes, 1)

    def forward(self, x):
        s1, p1 = self.down1(x)
        s2, p2 = self.down2(p1)
        s3, p3 = self.down3(p2)

        b = self.bottleneck(p3)
        d3 = self.up3(b, s3)
        d2 = self.up2(d3, s2)
        d1 = self.up1(d2, s1)
        return self.head(d1)
