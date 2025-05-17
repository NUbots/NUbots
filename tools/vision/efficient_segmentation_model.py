#!/usr/bin/env python3
#
# MIT License
#
# Copyright (c) 2025 NUbots
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

import os
import random

import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
from PIL import Image
from torch.utils.data import Dataset
from torchvision import transforms
from tqdm import tqdm


class SqueezeExcitation(nn.Module):
    def __init__(self, channel, reduction=4):
        super().__init__()
        self.avg_pool = nn.AdaptiveAvgPool2d(1)
        self.fc = nn.Sequential(
            nn.Linear(channel, channel // reduction),
            nn.ReLU(inplace=True),
            nn.Linear(channel // reduction, channel),
            nn.Sigmoid()
        )

    def forward(self, x):
        b, c, _, _ = x.size()
        y = self.avg_pool(x).view(b, c)
        y = self.fc(y).view(b, c, 1, 1)
        return x * y

class InvertedResidual(nn.Module):
    def __init__(self, in_channels, out_channels, stride, expansion=6, use_se=False):
        super().__init__()
        hidden_dim = in_channels * expansion
        self.use_residual = stride == 1 and in_channels == out_channels
        self.use_se = use_se

        layers = []
        if expansion != 1:
            layers.extend([
                nn.Conv2d(in_channels, hidden_dim, 1, bias=False),
                nn.BatchNorm2d(hidden_dim),
                nn.ReLU6(inplace=True)
            ])

        layers.extend([
            nn.Conv2d(hidden_dim, hidden_dim, 3, stride, 1, groups=hidden_dim, bias=False),
            nn.BatchNorm2d(hidden_dim),
            nn.ReLU6(inplace=True),
            nn.Conv2d(hidden_dim, out_channels, 1, bias=False),
            nn.BatchNorm2d(out_channels)
        ])

        if self.use_se:
            layers.append(SqueezeExcitation(out_channels))

        self.conv = nn.Sequential(*layers)

    def forward(self, x):
        if self.use_residual:
            return x + self.conv(x)
        else:
            return self.conv(x)

class DecoderBlock(nn.Module):
    def __init__(self, in_channels, skip_channels, out_channels):
        super().__init__()
        self.up = nn.Upsample(scale_factor=2, mode='bilinear', align_corners=True)
        self.conv = nn.Sequential(
            nn.Conv2d(in_channels + skip_channels, out_channels, 3, padding=1, bias=False),
            nn.BatchNorm2d(out_channels),
            nn.ReLU(inplace=True),
            nn.Conv2d(out_channels, out_channels, 3, padding=1, bias=False),
            nn.BatchNorm2d(out_channels),
            nn.ReLU(inplace=True)
        )

    def forward(self, x, skip):
        x = self.up(x)
        if skip is not None:
            x = torch.cat([x, skip], dim=1)
        return self.conv(x)

class EfficientSegmentationModel(nn.Module):
    def __init__(self, num_classes=3):
        super().__init__()
        # Encoder
        self.encoder1 = nn.Sequential(
            nn.Conv2d(3, 16, 3, stride=2, padding=1, bias=False),
            nn.BatchNorm2d(16),
            nn.ReLU6(inplace=True)
        )
        self.encoder2 = InvertedResidual(16, 24, stride=2, expansion=1, use_se=True)
        self.encoder3 = InvertedResidual(24, 32, stride=2, expansion=6, use_se=True)
        self.encoder4 = InvertedResidual(32, 64, stride=2, expansion=6, use_se=True)

        # Middle
        self.middle = InvertedResidual(64, 64, stride=1, expansion=6, use_se=True)

        # Decoder
        self.decoder1 = DecoderBlock(64, 32, 64)
        self.decoder2 = DecoderBlock(64, 24, 32)
        self.decoder3 = DecoderBlock(32, 16, 24)
        self.decoder4 = DecoderBlock(24, 0, 16)

        # Final
        self.final = nn.Conv2d(16, num_classes, 1)

    def forward(self, x):
        # Encoder
        e1 = self.encoder1(x)   # 1/2
        e2 = self.encoder2(e1)  # 1/4
        e3 = self.encoder3(e2)  # 1/8
        e4 = self.encoder4(e3)  # 1/16

        # Middle
        m = self.middle(e4)     # 1/16

        # Decoder
        d1 = self.decoder1(m, e3)  # 1/8
        d2 = self.decoder2(d1, e2) # 1/4
        d3 = self.decoder3(d2, e1) # 1/2
        d4 = self.decoder4(d3, None) # Original size

        return self.final(d4)
