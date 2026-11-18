import cv2
import numpy as np
import yaml
from housing_grasp.color_marker import head_mask

p = "../../Media/Grasp/work/runs/20261114_211721/"
frame = np.load(p + "frame.npz")
sticker = np.load(p + "marker_close_points.npy")

with open("config/grasp.yaml") as f:
    settings = yaml.safe_load(f)["color_marker"]

image = frame["color"].copy()
mask, _ = head_mask(
    image, frame["K"], frame["T_BC"],
    np.median(sticker, axis=0), settings
)

overlay = image.copy()
overlay[mask] = (0, 0, 255)
preview = cv2.addWeighted(image, 0.5, overlay, 0.5, 0)
cv2.imwrite(p + "housing_mask.png", preview)
print("Housing-mask pixels:", int(mask.sum()))
