#!/usr/bin/env python3


class Static:

  def __init__(self, policy):

    self.learning_rate = policy.learning_rate
    self._batches = policy.batches
    self._start_step = None if policy.hot_start else 0

  def update(self, global_step):
    # Update the start step
    self._start_step = global_step if self._start_step is None else self._start_step

  def finished(self, global_step):
    return self._start_step is not None and global_step >= self._start_step + self._batches
