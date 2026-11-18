import cv2
import numpy as np

p = "../../Media/Grasp/work/failed_grasps/20261115_032054/"
f = np.load(p + "insufficient_head.npz")

color = f["color"]
depth = f["depth"]
valid = np.isfinite(depth) & (depth > 0)

scaled = np.clip(
    (np.nan_to_num(depth) - 0.07) / 0.43 * 255, 0, 255
).astype("uint8")

view = cv2.applyColorMap(scaled, cv2.COLORMAP_TURBO)
view[~valid] = 0

cv2.imwrite(p + "failed_preview.png", np.hstack([color, view]))
print("Sticker points:", len(f["sticker"]))
print("Housing points:", len(f["head"]))
print("Valid depth percentage:", round(valid.mean() * 100, 1))
