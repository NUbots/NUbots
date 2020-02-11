#!/usr/bin/env python3

from tqdm import tqdm

import tensorflow as tf


class IntrinsicProgress(tf.keras.callbacks.Callback):
    def __init__(self):
        super(IntrinsicProgress, self).__init__()
        self.progress = tqdm(unit=" epochs", dynamic_ncols=True)

    def on_epoch_end(self, epoch, logs):
        self.progress.update()
        self.progress.set_description(
            "ƒ:{focal_length:.3f} Δx:{cx:+.3f} Δy:{cy:+.3f} k[{k}] (↔:{collinearity:.3f}º ||:{parallelity:.3f}º ⟂:{orthogonality:.3f}º)".format(
                focal_length=self.model.focal_length.numpy(),
                cx=self.model.centre[0].numpy(),
                cy=self.model.centre[1].numpy(),
                k=", ".join(["{:+.3f}".format(v) for v in self.model.k + 0]),
                **logs
            )
        )

    def on_train_end(self, logs=None):
        self.progress.close()
