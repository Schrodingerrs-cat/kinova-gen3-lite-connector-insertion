import sys
import numpy as np

sys.path.insert(0, "../Calibration")
from calib_lib import backproject

p = "../../Media/Grasp/work/failed_grasps/20261115_033009/"
f = np.load(p + "diagnostic.npz")

# Clear white panel immediately left of the USB connector.
depth = np.zeros_like(f["depth"])
depth[165:230, 380:405] = f["depth"][165:230, 380:405]

points = backproject(depth, f["K"], f["T_BC"], step=1)
if len(points) < 100:
    raise RuntimeError("Insufficient board depth")

center = points.mean(axis=0)
normal = np.linalg.svd(
    points - center, full_matrices=False
)[2][-1]
if normal[2] < 0:
    normal = -normal

sticker = f["center"]
board_z = center[2] - (
    normal[:2] @ (sticker[:2] - center[:2])
) / normal[2]

residual = np.abs((points - center) @ normal)

print("Board points:", len(points))
print("Board Z at sticker XY (mm):", round(board_z * 1000, 1))
print("Sticker Z (mm):", round(sticker[2] * 1000, 1))
print("Sticker above board, vertical (mm):",
      round((sticker[2] - board_z) * 1000, 1))
print("Plane residual median / p95 (mm):",
      np.round(np.percentile(residual, [50, 95]) * 1000, 1))
