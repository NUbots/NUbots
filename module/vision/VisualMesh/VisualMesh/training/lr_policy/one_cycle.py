#!/usr/bin/env python3


class OneCycle:

  def __init__(self, policy):
    self.learning_rate = policy.min_learning_rate

    # lr ranges
    self._min_lr = policy.min_learning_rate
    self._max_lr = policy.max_learning_rate
    self._decay_lr = policy.decay_learning_rate

    # Cycle size
    self._cycle_batches = policy.cycle_batches
    self._decay_batches = policy.decay_batches
    self._start_step = None if policy.hot_start else 0

  def update(self, global_step):

    # Update our start step if we haven't run yet
    self._start_step = global_step if self._start_step is None else self._start_step

    # While we are in the one cycle, cycle our learning rate
    cycle_phase = (global_step - self._start_step) / self._cycle_batches
    if cycle_phase < 0.5:  # Going up
      p = cycle_phase * 2  # Value from 0-1
      self.learning_rate = self._min_lr + (self._max_lr - self._min_lr) * p
    elif cycle_phase < 1.0:  # Going down
      p = 1.0 - (cycle_phase - 0.5) * 2  # Value from 1-0
      self.learning_rate = self._min_lr + (self._max_lr - self._min_lr) * p

    # After the one cycle, we just decay our learning rate slowly down to nothing
    else:
      decay_phase = (global_step - self._start_step - self._cycle_batches) / self._decay_batches
      self.learning_rate = self._min_lr * (1 - decay_phase) + self._decay_lr * (decay_phase)

  def finished(self, global_step):
    return self._start_step is not None and global_step >= self._start_step + self._cycle_batches + self._decay_batches
