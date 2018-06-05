#!/usr/bin/python
import numpy.random
def loadfile(path, label=None, max_heigh_delta=0.005):
    with open(path, 'r') as f:
        # Get our lines from our csv
        lines = f.readlines()

        # Get our float values from our csv
        data = [[float(v) for v in l.strip().split(',')] for l in lines[1:]]

        output = []
        for point in zip(data, data[1:]):

            # Left
            output.append((int(label if label != None else point[1][1] < point[1][0] + max_heigh_delta)
                           , point[1][ 4]                 # L_HIP_PITCH_PRESENT_VELOCITY
                           , point[1][ 4] - point[0][ 4]  # L_HIP_PITCH_ACCELERATION
                           , point[1][ 5]                 # L_HIP_PITCH_LOAD
                           , point[1][ 8]                 # L_KNEE_PRESENT_VELOCITY
                           , point[1][ 8] - point[0][ 8]  # L_KNEE_ACCELERATION
                           , point[1][ 9]                 # L_KNEE_LOAD
                           , point[1][12]                 # L_ANKLE_PITCH_PRESENT_VELOCITY
                           , point[1][12] - point[0][12]  # L_ANKLE_PITCH_ACCELERATION
                           , point[1][13]                 # L_ANKLE_PITCH_LOAD
                           ))

            output.append((int(label if label != None else point[1][0] < point[1][1] +max_heigh_delta)
                          , point[1][ 2]                 # R_HIP_PITCH_PRESENT_VELOCITY
                          , point[1][ 2] - point[0][ 2]  # R_HIP_PITCH_ACCELERATION
                          , point[1][ 3]                 # R_HIP_PITCH_LOAD
                          , point[1][ 6]                 # R_KNEE_PRESENT_VELOCITY
                          , point[1][ 6] - point[0][ 6]  # R_KNEE_ACCELERATION
                          , point[1][ 7]                 # R_KNEE_LOAD
                          , point[1][10]                 # R_ANKLE_PITCH_PRESENT_VELOCITY
                          , point[1][10] - point[0][10]  # R_ANKLE_PITCH_ACCELERATION
                          , point[1][11]                 # R_ANKLE_PITCH_LOAD
                          ))

    return output

# Files that have the feet up
negfiles   = ['walk_load_datasets/airstand.csv'
              , 'walk_load_datasets/airstand_old.csv'
              , 'walk_load_datasets/airwalk.csv'
              , 'walk_load_datasets/airwalkforward.csv'
              , 'walk_load_datasets/evenmoreairwalk_old.csv'
              , 'walk_load_datasets/moreairwalk_old.csv'
              , 'walk_load_datasets/back_turtle.csv'
              , 'walk_load_datasets/lyingback.csv']

# Files that have the feet down
posfiles   = ['walk_load_datasets/standing.csv'
              , 'walk_load_datasets/standing_old.csv']

# Files that have the feet changing based on the walk
mixedfiles = ['walk_load_datasets/morewalking_old.csv'
              , 'walk_load_datasets/walkforward.csv'
              , 'walk_load_datasets/walking_old.csv'
              , 'walk_load_datasets/walkonspot.csv'
              , 'walk_load_datasets/walkspin.csv']

# The file we will use for validation
testfiles = ['walk_load_datasets/bigwalk.csv']

# Load all our files
negfiles   = [loadfile(f, 0) for f in negfiles]
posfiles   = [loadfile(f, 1) for f in posfiles]
mixedfiles = [loadfile(f)    for f in mixedfiles]
testfiles  = [loadfile(f)    for f in testfiles]

# Flatten the data
trainingdata = [item for sublist in (negfiles + posfiles + mixedfiles) for item in sublist]
testdata = [item for sublist in testfiles for item in sublist]

from sklearn.neural_network import MLPClassifier
import numpy
trainingdata = numpy.array(trainingdata)
testdata = numpy.array(testdata)
clf = MLPClassifier(alpha=1e-3, hidden_layer_sizes=(5), max_iter=5000,verbose=True,random_state=numpy.random.RandomState(1))
clf.fit(trainingdata[:,1:],trainingdata[:,0])

#matrix
print "Hidden Weights:"
for l in clf.coefs_[0].T:
    print "-",l.tolist()
print "Hidden Bias"
print clf.intercepts_[0].tolist()

print "Output Weights:"
for l in clf.coefs_[1].T[0]:
    print "-",str([l])
print "Output Bias"
print clf.intercepts_[1]

probs = clf.predict_proba(testdata[:,1:])
out = numpy.hstack([(probs[:,1] > .5).reshape(-1,1),probs])

# Write out our validation output
with open('validation_prediction', 'w') as f:
  f.write("labels 0 1\n")
  for d in out:
      f.write(str(list(d)).strip('[]').replace(',', '') + '\n')

with open('validation_ground_truth', 'w') as f:
  for d in testdata:
    f.write('{}\n'.format(d[0]))



