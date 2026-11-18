import sys
import numpy as np

sys.path.insert(0, "../Calibration")
from calib_lib import backproject, fit_plane

p = "../../Media/Grasp/work/runs/20261114_211721/"
f = np.load(p + "frame.npz")

# Interior of the board's middle white panel in this saved image.
depth = np.zeros_like(f["depth"])
depth[320:360, 290:420] = f["depth"][320:360, 290:420]

points = backproject(depth, f["K"], f["T_BC"], step=1)
print("Valid board points:", len(points))

if len(points) < 100:
    raise RuntimeError("Not enough board depth for this check")

normal, offset, count = fit_plane(points)
tilt = np.degrees(np.arccos(np.clip(normal[2], -1, 1)))

print("Board normal in robot base:", normal)
print("Tilt from robot-base horizontal (degrees):", tilt)
print("Plane inlier fraction:", count / len(points))
