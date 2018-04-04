#!/usr/bin/env python3

import sys
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle

xmin = -30
xmax = 30
ymin = -3
ymax = 10

# Grid of x, y points
nx, ny = 256, 256
xs = np.linspace(xmin, xmax, nx)
ys = np.linspace(ymin, ymax, ny)

dx = np.zeros((ny, nx))
dy = np.zeros((ny, nx))

d = 2
u = 10
h = 7

# Fills xs and ys with values from dx and dy function
for xi, x in enumerate(xs):
    for yi, y in enumerate(ys):
        dx[yi, xi] = - np.tanh(x) * h*2**(u/-abs(x**d))
        dy[yi, xi] = h*2**(u/-abs(x**d)) - y * abs((np.tanh(10 * y)))

fig = plt.figure()
ax = fig.add_subplot(111)

# Plot the streamlines with an appropriate colormap and arrow style
color = np.hypot(dx, dy)
ax.streamplot(xs, ys, dx, dy, color=color, linewidth=1, cmap=plt.cm.jet,
              density=2, arrowstyle='->', arrowsize=1.5)

ax.axhline(y=h)
ax.axhline(y=0)

ax.set_xlabel('$x$')
ax.set_ylabel('$y$')
ax.set_xlim(xmin,xmax)
ax.set_ylim(ymin,ymax)
ax.set_aspect('equal')
plt.show()