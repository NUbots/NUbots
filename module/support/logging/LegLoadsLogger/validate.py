#!/usr/bin/python

import numpy
from matplotlib import pyplot

left_certainty = 0.5
left_uncertainty = 0.3

right_certainty = 0.5
right_uncertainty = 0.3

# Load our ground truth validation data
with open('validation_ground_truth', 'r') as f:
    data = f.readlines()
    left_truth = [float(d.strip().split(' ')[0]) for d in data[0::2]]
    right_truth = [float(d.strip().split(' ')[0]) for d in data[1::2]]

# Load our predicted output data
with open('validation_prediction', 'r') as f:
    data = f.readlines()
    left_probability = [float(d.strip().split(' ')[2]) for d in data[1::2]]
    right_probability = [float(d.strip().split(' ')[2]) for d in data[2::2]]

# Apply our bayesian filter
np = .000001
s = 0.5
n = 0.5
state = 1
left_predict = []
left_state = []
for v in left_probability:
    k = n / (n + np)  #2 * v)
    s = s + k * (v - s)
    n = (1 - k) * n + 1

    # Store our raw probability prediction
    left_predict.append(s)

    # Apply our hysteresis
    if s < left_uncertainty:
        state = 0
    elif s > left_certainty:
        state = 1

    # Store our state prediction
    left_state.append(state)

# Apply our bayesian filter
np = .000001
s = 0.5
n = 0.5
state = 1
right_predict = []
right_state = []
for v in right_probability:
    k = n / (n + np)  #2 * v)
    s = s + k * (v - s)
    n = (1 - k) * n + 1

    right_predict.append(s)

    # Apply our hysteresis
    if s < right_uncertainty:
        state = 0
    elif s > right_certainty:
        state = 1

    # Store our state prediction
    right_state.append(state)

left_fp = 0
left_fn = 0
left_tp = 0
left_tn = 0
for v in zip(left_truth, left_state):
    if v[0] == 0 and v[1] == 0:
        left_tn += 1
    elif v[0] == 0 and v[1] == 1:
        left_fp += 1
    elif v[0] == 1 and v[1] == 0:
        left_fn += 1
    elif v[0] == 1 and v[1] == 1:
        left_tp += 1

right_fp = 0
right_fn = 0
right_tp = 0
right_tn = 0
for v in zip(right_truth, right_state):
    if v[0] == 0 and v[1] == 0:
        right_tn += 1
    elif v[0] == 0 and v[1] == 1:
        right_fp += 1
    elif v[0] == 1 and v[1] == 0:
        right_fn += 1
    elif v[0] == 1 and v[1] == 1:
        right_tp += 1

print 'Left False Positive  {:5.2f}%'.format(100.0 * float(left_fp) / float(left_fp + left_fn + left_tp + left_tn))
print 'Left False Negative  {:5.2f}%'.format(100.0 * float(left_fn) / float(left_fp + left_fn + left_tp + left_tn))
print 'Left True Positive   {:5.2f}%'.format(100.0 * float(left_tp) / float(left_fp + left_fn + left_tp + left_tn))
print 'Left True Negative   {:5.2f}%'.format(100.0 * float(left_tn) / float(left_fp + left_fn + left_tp + left_tn))
print 'Left Accuracy        {:5.2f}%'.format(
    100.0 * float(left_tp + left_tn) / float(left_fp + left_fn + left_tp + left_tn)
)

print

print 'Right False Positive {:5.2f}%'.format(100.0 * float(right_fp) / float(right_fp + right_fn + right_tp + right_tn))
print 'Right False Negative {:5.2f}%'.format(100.0 * float(right_fn) / float(right_fp + right_fn + right_tp + right_tn))
print 'Right True Positive  {:5.2f}%'.format(100.0 * float(right_tp) / float(right_fp + right_fn + right_tp + right_tn))
print 'Right True Negative  {:5.2f}%'.format(100.0 * float(right_tn) / float(right_fp + right_fn + right_tp + right_tn))
print 'Right Accuracy       {:5.2f}%'.format(
    100.0 * float(right_tp + right_tn) / float(right_fp + right_fn + right_tp + right_tn)
)

print
print 'Total Accuracy       {:5.2f}%'.format(
    100.0 * float(right_tp + right_tn + left_tp + left_tn) /
    float(left_fp + left_fn + left_tp + left_tn + right_fp + right_fn + right_tp + right_tn)
)

# Plot our predicted state with some offset to make it easy to distinguish from the ground truth
pyplot.plot([s * 0.8 + 0.1 for s in right_state], marker='^')

# Plot our raw probabilities
#pyplot.plot(right_probability, marker='.')

# Plot our bayesian prediction values
pyplot.plot(right_predict, marker='x')

# Plot our ground truth
pyplot.plot(right_truth, marker='o')

# Extend a little above 0,1
pyplot.ylim((-0.05, 1.05))

# Show our graph
pyplot.show()
