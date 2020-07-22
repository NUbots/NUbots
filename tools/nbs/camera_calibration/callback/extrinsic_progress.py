#!/usr/bin/env python3

from tqdm import tqdm

import tensorflow as tf


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
