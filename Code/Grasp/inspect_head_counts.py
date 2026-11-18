import cv2
import numpy as np
import yaml
from housing_grasp.color_marker import head_mask, head_points

p = "../../Media/Grasp/work/failed_grasps/20261115_032054/"
f = np.load(p + "insufficient_head.npz")

with open("config/grasp.yaml") as stream:
    g = yaml.safe_load(stream)["color_marker"]
g["dark_v"] = 90

mask, _ = head_mask(
    f["color"], f["K"], f["T_BC"], f["center"], g
)
valid = mask & np.isfinite(f["depth"]) & (f["depth"] > 0)

print("Housing-mask pixels:", int(mask.sum()))
print("Housing pixels with depth:", int(valid.sum()))

points = head_points(
    f["depth"], f["color"], f["K"], f["T_BC"], f["center"], g
)
print("Points after all filters:", len(points))

image = f["color"].copy()
image[mask & ~valid] = (0, 0, 255)
image[valid] = (0, 255, 0)
cv2.imwrite(p + "housing_depth_mask.png", image)
