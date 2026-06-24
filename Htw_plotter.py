import json

import numpy as np
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation

json_file = "mocap_htw_6.json"

def HtwMessagetoHtw(HtwMessage):
    def v(row, col):
        return HtwMessage.get(row, {}).get(col, 0)
    return np.array([
        [v("x", "x"), v("x", "y"), v("x", "z"), 0],
        [v("y", "x"), v("y", "y"), v("y", "z"), 0],
        [v("z", "x"), v("z", "y"), v("z", "z"), 0],
        [v("t", "x"), v("t", "y"), v("t", "z"), 1],
    ])

def invert(H):
    R = H[:3, :3]
    t = H[3, :3]
    Hinv = np.eye(4)
    Hinv[:3, :3] = R.T
    Hinv[3, :3] = -t @ R.T
    return Hinv

def HtwtoCoord(Htw):
    return [float(Htw[3, 0]), float(Htw[3, 1])]

with open(json_file) as f:
    records = [r for line in f if line.strip()
               for r in (json.loads(line),)
               if r.get("type") == "message.localisation.OdometryRecord"]

def Hwt(rec, key):
    return invert(HtwMessagetoHtw(rec["data"][key]))

point_trail    = np.array([HtwtoCoord(Hwt(r, "Htw")) for r in records])
gt_trail       = np.array([HtwtoCoord(Hwt(r, "Htw_ground_truth")) for r in records])
kin_trail      = np.array([HtwtoCoord(Hwt(r, "Htw_kinematic")) for r in records])

fig, ax = plt.subplots()

all_x = np.concatenate([point_trail[:, 0], gt_trail[:, 0], kin_trail[:, 0]])
all_y = np.concatenate([point_trail[:, 1], gt_trail[:, 1], kin_trail[:, 1]])
pad = ((all_x.max() - all_x.min()) + (all_y.max() - all_y.min())) * 0.05 + 0.01
ax.set_xlim(all_x.min() - pad, all_x.max() + pad)
ax.set_ylim(all_y.min() - pad, all_y.max() + pad)
ax.set_aspect("equal")

gt_line,  = ax.plot([], [], ".", markersize=2, label="ground truth")
kin_line, = ax.plot([], [], ".", markersize=2, label="kinematic odometry")
nur_line, = ax.plot([], [], ".", markersize=2, label="NUral odometry")

# Current-position markers
gt_dot,  = ax.plot([], [], "o", markersize=6, color=gt_line.get_color())
kin_dot, = ax.plot([], [], "o", markersize=6, color=kin_line.get_color())
nur_dot, = ax.plot([], [], "o", markersize=6, color=nur_line.get_color())

ax.legend()

# How many data points to advance per animation frame
step = max(1, len(records) // 500)

def update(frame):
    i = min(frame * step + 1, len(records))
    gt_line.set_data(gt_trail[:i, 0], gt_trail[:i, 1])
    kin_line.set_data(kin_trail[:i, 0], kin_trail[:i, 1])
    nur_line.set_data(point_trail[:i, 0], point_trail[:i, 1])
    gt_dot.set_data([gt_trail[i - 1, 0]], [gt_trail[i - 1, 1]])
    kin_dot.set_data([kin_trail[i - 1, 0]], [kin_trail[i - 1, 1]])
    nur_dot.set_data([point_trail[i - 1, 0]], [point_trail[i - 1, 1]])
    return gt_line, kin_line, nur_line, gt_dot, kin_dot, nur_dot

n_frames = (len(records) + step - 1) // step
anim = FuncAnimation(fig, update, frames=n_frames, interval=20, blit=True)

plt.show()
