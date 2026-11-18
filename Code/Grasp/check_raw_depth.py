import json
import numpy as np
from housing_grasp.color_marker import marker_mask

p = "../../Media/Grasp/work/failed_grasps/20261115_033009/"
f = np.load(p + "diagnostic.npz")

with open(p + "settings.json") as stream:
    g = json.load(stream)["search"]

sticker = marker_mask(f["color"], g)
board = np.zeros(f["depth"].shape, dtype=bool)
board[165:230, 380:405] = True

for name, mask in [("sticker", sticker), ("board", board)]:
    values = f["depth"][mask]
    values = values[np.isfinite(values) & (values > 0)]
    print(name, "valid pixels:", len(values))
    if len(values):
        print("Camera depth p5 / median / p95 (mm):",
              np.round(np.percentile(values, [5, 50, 95]) * 1000, 1))
