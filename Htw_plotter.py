import json

import numpy as np
from matplotlib import pyplot as plt

json_file = "mocap_htw_3.json"

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

point_trail = [HtwtoCoord(Hwt(r, "Htw")) for r in records]
gt_point_trail = [HtwtoCoord(Hwt(r, "Htw_ground_truth")) for r in records]
kin_point_trail = [HtwtoCoord(Hwt(r, "Htw_kinematic")) for r in records]

x, y = np.array(point_trail).T
gt_x, gt_y = np.array(gt_point_trail).T
kin_x, kin_y = np.array(kin_point_trail).T
plt.scatter(x, y, linewidths=0.001, label="NUral odometry")
plt.scatter(gt_x, gt_y, linewidths=0.001, label="ground truth")
plt.scatter(kin_x, kin_y, linewidths=0.001, label="kinematic odometry")
plt.legend()
plt.savefig("img.png")
plt.show()
