#!/usr/bin/python

import matplotlib.pyplot as plt
from numpy import linspace, meshgrid
from matplotlib.mlab import griddata
import matplotlib.ticker as mtick
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.colors import Normalize
import random

def grid(x, y, z, resX=100, resY=60):
    "Convert 3 column data to matplotlib grid"
    xi = linspace(min(x), max(x), resX)
    yi = linspace(min(y), max(y), resY)
    Z = griddata(x, y, z, xi, yi)
    X, Y = meshgrid(xi, yi)
    return X, Y, Z

def close(a, b, maxDistance):
    return abs(float(a) - float(b)) < maxDistance

def goalIsValid(goal):

    # Our fixed limits
    leftGoalLimits  = [[68,  57], [71,  194], [54,  195], [51,  58]]
    rightGoalLimits = [[286, 71], [285, 184], [271, 184], [269, 71]]
    points = [goal[2:4], goal[4:6], goal[6:8], goal[8:10]]

    isValid = True
    for a, b in zip(points, leftGoalLimits):
        for c, d in zip(a,b):
            if not close(c,d,20):
                isValid = False

    if isValid:
        return True

    isValid = True
    for a, b in zip(points, rightGoalLimits):
        for c, d in zip(a,b):
            if not close(c,d,20):
                isValid = False

    return isValid


def hue(timestamp, duration):
    return float(timestamp) / float(duration * 1000.0)

def process(intensity, duration, lines, kind):
    # Split our ball and goal lines
    ballLines = [ l.split(',') for l in lines if 'Ball' in l ]
    goalLines = [ l.split(',') for l in lines if 'Goal' in l ]

    # Filter out bad balls
    ballLines = [b for b in ballLines if (abs(float(b[2]) - 181.1)) < 2
                                     and (abs(float(b[3]) - 196.5)) < 2
                                     and (abs(float(b[4]) - 9.3)) < 2 ]

    # Build our ball list from the ballLines
    balls = [(hue(int(b[1]), duration), intensity, duration, kind) for b in ballLines]

    goalLines = [g for g in goalLines if goalIsValid(g)]

    goals = [(hue(int(g[1]), duration), intensity, duration, kind) for g in goalLines]

    # for b in balls:
    #     print '{},{},{},{},BALL'.format(*b)

    # for g in goals:
    #     print '{},{},{},{},GOAL'.format(*g)

    return (balls, goals)
    # Split into ball/goal lines

    # Throwout bad detections

    # Calculate the hue from the duration/origin

    # return a list of tuples of (hue, intensity, duration, type)

goals = []
balls = []

for f in xrange(0, 105, 5):
    intensity = "%.2f" % (f * 0.01)

    for d in xrange(5, 65, 5):

        with open('/Users/trent/Code/DatanStuff/newRes/StaticI{}D{}.csv'.format(intensity, d), 'r') as file:
            lines = file.readlines()
            lines = [l.strip() for l in lines]
            g,b = process(f, d, lines, 'STATIC')
            goals += g
            balls += b

        with open('/Users/trent/Code/DatanStuff/newRes/LayerI{}D{}.csv'.format(intensity, d), 'r') as file:
            lines = file.readlines()
            lines = [l.strip() for l in lines]
            g,b = process(f, d, lines, 'LAYER')
            goals += g
            balls += b

        with open('/Users/trent/Code/DatanStuff/newRes/PressureI{}D{}.csv'.format(intensity, d), 'r') as file:
            lines = file.readlines()
            lines = [l.strip() for l in lines]
            g,b = process(f, d, lines, 'PRESSURE')
            goals += g
            balls += b

contour = {}
print len(balls)
print len(goals)
# Build up our list
for i in xrange(0, 105, 5):
    for d in xrange(5, 65, 5):
        contour[(i,d)] = 0

# Sum our data in
for b in balls:
    if b[3] == 'LAYER':
        i = b[1]
        d = b[2]
        contour[(i,d)] += 1

for g in goals:
    if g[3] == 'LAYER':
        i = g[1]
        d = g[2]
        contour[(i,d)] += 1

x = []
y = []
z = []

for k in contour:
    x.append(k[0])
    y.append(k[1])
    z.append((float(contour[k]) / float(k[1] * 30 * 3)) * 100.0)

# Percentage formatter
percentTick = mtick.FormatStrFormatter('%.0f%%')

#fig = plt.figure()
X, Y, Z = grid(x, y, z)
cs = plt.contourf(X, Y, Z, 10, norm=Normalize(0.0,100))
plt.xlabel("Saturation (%)")
plt.ylabel("Hue Cycle Duration (s)")

# Colourbar
cbar = plt.colorbar(cs)
cbar.ax.set_ylabel('Detection Rate (%)')

plt.savefig("output.eps", trasparent=True)

#ax = fig.add_subplot(111, projection='3d')


#ax.scatter(xs=x,
#           ys=y,
#           zs=z)

# ax.scatter(xs=[b[0] for b in balls if b[3] == 'PRESSURE'],
#            ys=[b[1] for b in balls if b[3] == 'PRESSURE'],
#            zs=[b[2] for b in balls if b[3] == 'PRESSURE'])


plt.show()

# plt.contour(x,y,z)
# plt.show()
# Sum the hue for each duration/intensity
# Divide the sum by the duration

# balls = random.sample(balls, 10000)

# ax.scatter(xs=[b[0] for b in balls if b[3] == 'PRESSURE'],
#            ys=[b[1] for b in balls if b[3] == 'PRESSURE'],
#            zs=[b[2] for b in balls if b[3] == 'PRESSURE'])

# plt.show()
# Open each dataset

# Split into ball/goal detections and throwout bad detections

# Calculate the hue/intensity from the dataset name