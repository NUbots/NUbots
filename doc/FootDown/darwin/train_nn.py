#!/usr/bin/env python3

from sklearn.neural_network import MLPClassifier
import numpy as np


def loadfile(path, label=None, max_height_delta=0.008):
    with open(path, 'r') as f:
        # Get our lines from our csv
        lines = f.readlines()

        # Get our float values from our csv
        data = [[float(v) for v in l.strip().split(',')] for l in lines[1:]]

        output = []
        for point in zip(data, data[1:]):

            # Left
            output.append((
                int(label if label != None else point[1][1] < point[1][0] + max_height_delta),
                point[1][4],  # L_HIP_PITCH_PRESENT_VELOCITY
                point[1][4] - point[0][4],  # L_HIP_PITCH_ACCELERATION
                point[1][5],  # L_HIP_PITCH_LOAD
                point[1][8],  # L_KNEE_PRESENT_VELOCITY
                point[1][8] - point[0][8],  # L_KNEE_ACCELERATION
                point[1][9],  # L_KNEE_LOAD
                point[1][12],  # L_ANKLE_PITCH_PRESENT_VELOCITY
                point[1][12] - point[0][12],  # L_ANKLE_PITCH_ACCELERATION
                point[1][13]  # L_ANKLE_PITCH_LOAD
            ))

            # Right
            output.append((
                int(label if label != None else point[1][0] < point[1][1] + max_height_delta),
                point[1][2],  # R_HIP_PITCH_PRESENT_VELOCITY
                point[1][2] - point[0][2],  # R_HIP_PITCH_ACCELERATION
                point[1][3],  # R_HIP_PITCH_LOAD
                point[1][6],  # R_KNEE_PRESENT_VELOCITY
                point[1][6] - point[0][6],  # R_KNEE_ACCELERATION
                point[1][7],  # R_KNEE_LOAD
                point[1][10],  # R_ANKLE_PITCH_PRESENT_VELOCITY
                point[1][10] - point[0][10],  # R_ANKLE_PITCH_ACCELERATION
                point[1][11]  # R_ANKLE_PITCH_LOAD
            ))

    return output


###
### Load all our files
###
# Files that have the feet up
negative_files = [
    loadfile(f, 0) for f in [
        'laying_back.csv',
        'laying_front.csv',
        'picked_up.csv',
        'walking_picked_up.csv',
        'walking_laying_back.csv',
        'walking_laying_front.csv',
    ]
]

# Files that have the feet down
positive_files = [loadfile(f, 1) for f in ['standing.csv']]

# Files that have the feet changing based on the walk
mixed_files = [loadfile(f) for f in ['walking.csv', 'walking2.csv', 'walking3.csv', 'walking4.csv']]

# The file we will use for validation
test_files = [loadfile(f) for f in ['long_walk.csv']]

# Flatten the data
training_data = np.array([item for sub_list in (negative_files + positive_files + mixed_files) for item in sub_list])
test_data = np.array([item for sub_list in test_files for item in sub_list])

clf = MLPClassifier(
    alpha=1e-3, hidden_layer_sizes=(5), max_iter=5000, verbose=True, random_state=np.random.RandomState(1)
)
clf.fit(training_data[:, 1:], training_data[:, 0])

#matrix
print('Hidden Weights:')
for l in clf.coefs_[0].T:
    print('-', l.tolist())
print('Hidden Bias')
print(clf.intercepts_[0].tolist())

print('Output Weights:')
for l in clf.coefs_[1].T[0]:
    print('-', str([l]))
print('Output Bias')
print(clf.intercepts_[1])

probs = clf.predict_proba(test_data[:, 1:])
out = np.hstack([(probs[:, 1] > .5).reshape(-1, 1), probs])

# Write out our validation output
with open('validation_prediction', 'w') as f:
    f.write('labels 0 1\n')
    for d in out:
        f.write(str(list(d)).strip('[]').replace(',', '') + '\n')

with open('validation_ground_truth', 'w') as f:
    for d in test_data:
        f.write('{}\n'.format(d[0]))
