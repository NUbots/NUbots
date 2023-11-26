#!/usr/bin/env python3
#
# MIT License
#
# Copyright (c) 2020 NUbots
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

import math

import tensorflow as tf
from tqdm import tqdm


class IntrinsicProgress(tf.keras.callbacks.Callback):
    def __init__(self):
        super(IntrinsicProgress, self).__init__()
        self.progress = tqdm(unit=" epochs", dynamic_ncols=True)

    def on_epoch_end(self, epoch, logs):
        self.progress.update()
        self.progress.set_description(
            "ƒ:{focal_length:.3f} Δc:[{centre}] k:[{k}] (↔:{collinearity:.3f}º ||:{parallelity:.3f}º ⟂:{orthogonality:.3f}º)".format(
                focal_length=self.model.focal_length.numpy(),
                cx=self.model.centre[0].numpy(),
                cy=self.model.centre[1].numpy(),
                centre=", ".join(["{:+.3f}".format(v) for v in self.model.centre + 0]),
                k=", ".join(["{:+.3f}".format(v) for v in self.model.k + 0]),
                collinearity=math.sqrt(logs["collinearity"]) * 180 / math.pi,
                parallelity=math.sqrt(logs["parallelity"]) * 180 / math.pi,
                orthogonality=math.sqrt(logs["orthogonality"]) * 180 / math.pi,
            )
        )

    def on_train_end(self, logs=None):
        self.progress.close()
