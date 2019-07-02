import numpy as np
import matplotlib.pyplot as plt


class Quad:
    def __init__(self, bl, tl, tr, br):
        self.bl = bl
        self.tl = tl
        self.tr = tr
        self.br = br

    def interp(self, x, y):
        return (
            (1 - x) * (1 - y) * self.bl
            + (1 - x) * (1 + y) * self.tl
            + (1 + x) * (1 + y) * self.tr
            + (1 + x) * (1 - y) * self.br
        ) / 4


bl = np.array([-3, -3])
br = np.array([3, -3])
tl = np.array([-3, 3])
tr = np.array([3, 3])

quad = Quad(bl, tl, tr, br)

# plt.figure()

x = np.linspace(-1, 1, 20)
y = x

u, v = np.array([]), np.array([])
for x_i in x:
    for y_i in y:
        u_i, v_i = quad.interp(x_i, y_i)
        u = np.append(u, [u_i])
        v = np.append(v, [v_i])

plt.plot(u, v, ".b")
axis_range = 3
plt.ylim([-axis_range, axis_range])
plt.xlim([-axis_range, axis_range])
print(u, v)
plt.show()
