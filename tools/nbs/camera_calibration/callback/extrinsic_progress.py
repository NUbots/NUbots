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

import tensorflow as tf
from tqdm import tqdm


class ExtrinsicProgress(tf.keras.callbacks.Callback):
    def __init__(self):
        super(ExtrinsicProgress, self).__init__()
        self.progress = tqdm(unit=" batches", dynamic_ncols=True)

    def on_batch_end(self, epoch, logs):

        self.progress.update()
        self.progress.set_description(
            "⟀:{alignment:.3f}º Δd:{absolute_distance:.3f}m ({relative_distance:.3f}%)".format(
                alignment=logs["alignment"],
                relative_distance=logs["relative_distance"] * 100,
                absolute_distance=logs["absolute_distance"],
            )
        )

    def on_train_end(self, logs=None):
        self.progress.close()
