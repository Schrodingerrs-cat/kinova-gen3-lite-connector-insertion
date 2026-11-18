import sys
import numpy as np

sys.path.insert(0, "../Calibration")
from calib_lib import backproject

p = "../../Media/Grasp/work/runs/20261114_211721/"
f = np.load(p + "frame.npz")
normals = []

for name, box in [
    ("table", (120, 280, 40, 220)),
    ("board", (320, 360, 290, 420)),
]:
    y0, y1, x0, x1 = box
    depth = np.zeros_like(f["depth"])
    depth[y0:y1, x0:x1] = f["depth"][y0:y1, x0:x1]

    points = backproject(depth, f["K"], f["T_BC"], step=2)
    if len(points) < 100:
        raise RuntimeError("Insufficient points: " + name)

    centered = points - points.mean(axis=0)
    normal = np.linalg.svd(centered, full_matrices=False)[2][-1]
    if normal[2] < 0:
        normal = -normal

    residual = np.abs(centered @ normal) * 1000
    tilt = np.degrees(np.arccos(np.clip(normal[2], -1, 1)))
    normals.append(normal)

    print(name, "tilt:", round(tilt, 2))
    print("Residual median / 95th percentile (mm):",
          np.round(np.percentile(residual, [50, 95]), 2))

angle = np.degrees(np.arccos(
    np.clip(np.dot(normals[0], normals[1]), -1, 1)
))
print("Angle between surfaces:", round(angle, 2))
