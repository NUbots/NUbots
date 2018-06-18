#!/usr/bin/env python3

from matplotlib import pyplot


def bayesian_filter(probability, uncertainty=0.3, certainty=0.5):
    np = 0.000001
    s = 0.5
    n = 0.5
    predicted_state = 1
    prediction = []
    state = []

    for v in probability:
        k = n / (n + np)  #2 * v)
        s = s + k * (v - s)
        n = (1 - k) * n + 1

        # Store our raw probability prediction
        prediction.append(s)

        # Apply our hysteresis and store our state prediction
        if s < uncertainty:
            predicted_state = 0
        elif s > certainty:
            predicted_state = 1

        state.append(predicted_state)

    return prediction, state


def calculate_confusion_matrix(truth, prediction):
    fp = 0
    fn = 0
    tp = 0
    tn = 0

    for v in zip(truth, prediction):
        if v[0] == 0 and v[1] == 0:
            tn += 1
        elif v[0] == 0 and v[1] == 1:
            fp += 1
        elif v[0] == 1 and v[1] == 0:
            fn += 1
        elif v[0] == 1 and v[1] == 1:
            tp += 1

    return tp, fp, tn, fn


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
left_predict, left_state = bayesian_filter(left_probability)

# Apply our bayesian filter
right_predict, right_state = bayesian_filter(right_probability)

# Calculate our confusion matrices
left_tp, left_fp, left_tn, left_fn = calculate_confusion_matrix(left_truth, left_state)
right_tp, right_fp, right_tn, right_fn = calculate_confusion_matrix(right_truth, right_state)

print('Left False Positive.: {:5.2f}%'.format(100.0 * float(left_fp) / float(left_fp + left_fn + left_tp + left_tn)))
print('Left False Negative.: {:5.2f}%'.format(100.0 * float(left_fn) / float(left_fp + left_fn + left_tp + left_tn)))
print('Left True Positive..: {:5.2f}%'.format(100.0 * float(left_tp) / float(left_fp + left_fn + left_tp + left_tn)))
print('Left True Negative..: {:5.2f}%'.format(100.0 * float(left_tn) / float(left_fp + left_fn + left_tp + left_tn)))
print(
    'Left Accuracy.......: {:5.2f}%'.format(
        100.0 * float(left_tp + left_tn) / float(left_fp + left_fn + left_tp + left_tn)
    )
)

print()

print(
    'Right False Positive: {:5.2f}%'.format(100.0 * float(right_fp) / float(right_fp + right_fn + right_tp + right_tn))
)
print(
    'Right False Negative: {:5.2f}%'.format(100.0 * float(right_fn) / float(right_fp + right_fn + right_tp + right_tn))
)
print(
    'Right True Positive.: {:5.2f}%'.format(100.0 * float(right_tp) / float(right_fp + right_fn + right_tp + right_tn))
)
print(
    'Right True Negative.: {:5.2f}%'.format(100.0 * float(right_tn) / float(right_fp + right_fn + right_tp + right_tn))
)
print(
    'Right Accuracy......: {:5.2f}%'.format(
        100.0 * float(right_tp + right_tn) / float(right_fp + right_fn + right_tp + right_tn)
    )
)

print()
print(
    'Total Accuracy......: {:5.2f}%'.format(
        100.0 * float(right_tp + right_tn + left_tp + left_tn) /
        float(left_fp + left_fn + left_tp + left_tn + right_fp + right_fn + right_tp + right_tn)
    )
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
