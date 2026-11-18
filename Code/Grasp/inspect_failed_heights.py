import json
import numpy as np

p = "../../Media/Grasp/work/failed_grasps/20261115_033009/"
f = np.load(p + "diagnostic.npz")

with open(p + "settings.json") as stream:
    settings = json.load(stream)

g = settings["search"]
center = f["center"]
tips = center[2] - g["grasp_drop_m"] - np.array(g["extra_depth_m"])

print("Sticker center XYZ (mm):", np.round(center * 1000, 1))
print("Tested fingertip heights (mm):", np.round(tips * 1000, 1))

for name in ["head", "scene"]:
    points = f[name]
    print(name, "point count:", len(points))
    if len(points):
        print("Z percentiles 5/50/95 (mm):",
              np.round(np.percentile(points[:, 2], [5, 50, 95]) * 1000, 1))
