import json
import numpy as np
from housing_grasp.color_marker import marker_mask

p = "../../Media/Grasp/work/failed_grasps/20261115_033009/"
f = np.load(p + "diagnostic.npz")

with open(p + "settings.json") as stream:
    g = json.load(stream)["search"]

def points(mask):
    v, u = np.nonzero(
        mask & np.isfinite(f["depth"]) & (f["depth"] > 0)
    )
    z = f["depth"][v, u]
    K = f["K"]
    return np.column_stack((
        (u - K[0, 2]) * z / K[0, 0],
        (v - K[1, 2]) * z / K[1, 1],
        z,
    ))

board = np.zeros(f["depth"].shape, dtype=bool)
board[165:230, 380:405] = True
B = points(board)
S = points(marker_mask(f["color"], g))

center = B.mean(axis=0)
normal = np.linalg.svd(B - center, full_matrices=False)[2][-1]
if normal @ (-center) < 0:
    normal = -normal

distance = (S - center) @ normal * 1000
print("Sticker distance from board plane p5/median/p95 (mm):")
print(np.round(np.percentile(distance, [5, 50, 95]), 1))
print("Positive means on the camera-facing side of the board.")
