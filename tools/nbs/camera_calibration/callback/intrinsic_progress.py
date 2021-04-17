#!/usr/bin/env python3

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
