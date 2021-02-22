#!/usr/bin/env python3

import math

from tqdm import tqdm

import numpy as np
import si_prefix
import tensorflow as tf

from ..matrix import compose


class Progress(tf.keras.callbacks.Callback):
    def __init__(self, camera_names, n_flat_steps):
        super(Progress, self).__init__()
        self.n_flat_steps = n_flat_steps
        self.camera_names = camera_names
        self.progress = tqdm(
            unit=" epochs", total=self.n_flat_steps, leave=True, desc="Optimising Camera System", dynamic_ncols=True
        )

        self.intrinsic_spacer = tqdm(bar_format="{desc}", leave=True, desc="Intrinsics")
        self.intrinsic_trackers = {
            name: tqdm(bar_format="{desc}", desc=name, leave=True, dynamic_ncols=True) for name in camera_names
        }
        self.extrinsic_spacer = tqdm(bar_format="{desc}", leave=True, desc="Extrinsics")
        self.extrinsic_trackers = {}
        keys = sorted(camera_names)
        for i in range(len(keys)):
            k1 = keys[i]
            for j in range(i + 1, len(keys)):
                k2 = keys[j]
                self.extrinsic_trackers["{}/{}".format(k1, k2)] = tqdm(
                    bar_format="{desc}", desc="{}/{}".format(k1, k2), leave=True, dynamic_ncols=True
                )

    def _update_display(self, logs):

        max_name_length = max([len(s) for s in self.intrinsic_trackers.keys()])
        for k, tracker in self.intrinsic_trackers.items():
            m = self.model.cameras[k]

            # Extract variable
            distance = si_prefix.split(math.sqrt(logs["{}/grid_error".format(k)]) * 3)

            tracker.set_description(
                " ".join(
                    [
                        "{: >{}}".format(k, max_name_length),
                        "ƒ:{:#2.3f}".format(m.focal_length.numpy()),
                        "Δc:[{:+.3f}, {:+.3f}]".format(m.centre[0].numpy(), m.centre[1].numpy()),
                        "k:[{}]".format(", ".join(["{:+.3f}".format(v) for v in m.k + 0])),
                        "(↔:{:#.2f}º".format(math.sqrt(logs["{}/collinearity".format(k)]) * 3.0 * 180.0 / math.pi),
                        "||:{:#.2f}º".format(math.sqrt(logs["{}/parallelity".format(k)]) * 3.0 * 180.0 / math.pi),
                        "⟂:{:#.2f}º".format(math.sqrt(logs["{}/orthogonality".format(k)]) * 3.0 * 180.0 / math.pi),
                        "▱:{:#.2f}º".format(math.sqrt(logs["{}/coplanarity".format(k)]) * 3.0 * 180.0 / math.pi),
                        "#:{:#.2f} {}m)".format(distance[0], si_prefix.prefix(distance[1])),
                    ]
                )
            )
            tracker.update()

        max_name_length = max([len(s) for s in self.extrinsic_trackers.keys()])
        for k, tracker in self.extrinsic_trackers.items():
            a, b = k.split("/")

            # Extract the information we need
            distance = si_prefix.split(tf.linalg.norm(self.model.rCBbs[a] - self.model.rCBbs[b]).numpy())
            distance_error = si_prefix.split(math.sqrt(logs["{}/distance".format(k)] * 3.0))
            angle = self.model._angle_difference(compose(self.model.Rbcs[a])[:, 0], compose(self.model.Rbcs[b])[:, 0])

            tracker.set_description(
                " ".join(
                    [
                        "{: >{}}:".format("{} ⟶ {}".format(a, b), max_name_length + 2),
                        "{:#5.1f} {}m".format(distance[0], si_prefix.prefix(distance[1])),
                        "(±{:#5.1f} {}m)".format(distance_error[0], si_prefix.prefix(distance_error[1])),
                        "{:#4.2f}º".format(angle * 180 / math.pi),
                        "(◩:{:#4.2f}º".format(math.sqrt(logs["{}/dihedral".format(k)] * 3.0 * 180.0 / math.pi)),
                        "△:{:#4.2f}º)".format(math.sqrt(logs["{}/triangle".format(k)] * 3.0 * 180.0 / math.pi)),
                    ]
                )
            )
            tracker.update()

    def on_epoch_end(self, epoch, logs):

        # Update how many we have to go based on how many epochs without a loss improvement before we end
        h = self.model.history.history
        best_idx = np.argmin(h["loss"]) if "loss" in h else 0
        self.progress.total = best_idx + self.n_flat_steps
        self.progress.update()

        self._update_display(logs)

    def on_train_end(self, logs=None):

        # Update all of the logs to be the results from the best loss so that's what is left on screen
        # Since this callback is after the fast ending one it should restore the best weights before this callback
        h = self.model.history.history
        best_idx = np.argmin(h["loss"])
        logs = {k: h[k][best_idx] for k in h}
        self._update_display(logs)

        # Clean up the progress bars
        self.progress.close()

        # Close intrinsic bars
        self.intrinsic_spacer.close()
        for k in self.camera_names:
            self.intrinsic_trackers[k].close()

        # Close extrinsic bars
        self.extrinsic_spacer.close()
        keys = sorted(self.camera_names)
        for i in range(len(keys)):
            k1 = keys[i]
            for j in range(i + 1, len(keys)):
                k2 = keys[j]
                self.extrinsic_trackers["{}/{}".format(k1, k2)].close()
