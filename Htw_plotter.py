import json

import numpy as np
from matplotlib import pyplot as plt  # # let's plot this bitch

json_file = "mocap_htw.json"
point_trail = []
gt_point_trail = []

odom_offset_xy = []
odom_offset_rad = 0
gt_offset_xy = []
gt_offset_rad = 0


def HtwtoCoord(Htw) -> float : # x, y
    return [float(Htw[3, 0]), float(Htw[3, 1])]

def HtwtoAngle(Htw) -> float : # radians
    R = Htw[:3, :3]
    yaw = np.arctan2(R[0, 1], R[0, 0])
    return yaw

def HtwMessagetoHtw(HtwMessage):
    def v(row, col):
        return HtwMessage.get(row, {}).get(col, 0)

    Htw = np.array([
        [v("x", "x"), v("x", "y"), v("x", "z"), 0],
        [v("y", "x"), v("y", "y"), v("y", "z"), 0],
        [v("z", "x"), v("z", "y"), v("z", "z"), 0],
        [v("t", "x"), v("t", "y"), v("t", "z"), 1],
    ])

    return Htw

def rot2d(rad):
    c, s = np.cos(-rad), np.sin(-rad)
    return np.array([[c, -s], [s, c]])

with open(json_file) as f:
    records = [r for line in f if line.strip() for r in (json.loads(line),) if r.get("type") == "message.localisation.OdometryRecord"]

## get the offsets
initial_record = records[0]
odom_offset_xy = HtwtoCoord(HtwMessagetoHtw(initial_record["data"]["Htw"]))
odom_offset_rad = HtwtoAngle(HtwMessagetoHtw(initial_record["data"]["Htw"]))
gt_offset_xy = HtwtoCoord(HtwMessagetoHtw(initial_record["data"]["Htw_ground_truth"]))
gt_offset_rad = HtwtoAngle(HtwMessagetoHtw(initial_record["data"]["Htw_ground_truth"]))

print(gt_offset_rad, odom_offset_rad)

for record in records:
    coord = HtwtoCoord(HtwMessagetoHtw(record["data"]["Htw"]))
    gt_coord = HtwtoCoord(HtwMessagetoHtw(record["data"]["Htw_ground_truth"]))

    point_trail.append([coord[0] - odom_offset_xy[0], coord[1] - odom_offset_xy[1]])
    gt_point_trail.append([gt_coord[0] - gt_offset_xy[0], gt_coord[1] - gt_offset_xy[1]])

coords = (rot2d(-0.25) @ np.array(point_trail).T).T
gt_coords = (rot2d(0) @ np.array(gt_point_trail).T).T

x, y = coords.T
gt_x, gt_y = gt_coords.T
plt.scatter(x, y, linewidths=0.001, label="odometry")
plt.scatter(gt_x, gt_y, linewidths=0.001, label="ground truth")
plt.legend()
plt.savefig("img.png")
plt.show()
