import numpy as np
import math
import matplotlib.pyplot as plt


viewpoint = np.array([-1872.262, 399.786, 450.743])
tl = np.array([159.1, 1654, -1063.8])
tr = np.array([235.5, 1399.5, 1064.5])
bl = np.array([230.5, 61.96, -1045.6])
br = np.array([-178.6, 104.6, 849.7])

v_tl = tl - viewpoint
v_tr = tr - viewpoint
v_bl = bl - viewpoint
v_br = br - viewpoint

u_l = (v_tl + v_bl) / np.linalg.norm(v_tl + v_bl)
u_r = (v_tr + v_br) / np.linalg.norm(v_tr + v_br)
u_b = (v_bl + v_br) / np.linalg.norm(v_bl + v_br)
u_t = (v_tr + v_tl) / np.linalg.norm(v_tr + v_tl)

fovx = math.acos(np.dot(u_r, u_l))
fovy = math.acos(np.dot(u_t, u_b))

print(fovx, fovy)
print(180 * fovx / math.pi, 180 * fovy / math.pi)
