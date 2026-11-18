import cv2
import numpy as np
import yaml
from housing_grasp.color_marker import head_mask

p = "../../Media/Grasp/work/failed_grasps/20261115_032054/"
f = np.load(p + "insufficient_head.npz")

with open("config/grasp.yaml") as stream:
    settings = yaml.safe_load(stream)["color_marker"]

tiles = []
for radius in [0.012, 0.015, 0.018, 0.022]:
    g = dict(settings, dark_v=90, footprint_radius_m=radius)
    mask, _ = head_mask(
        f["color"], f["K"], f["T_BC"], f["center"], g
    )
    image = f["color"].copy()
    image[mask] = (0, 255, 0)

    # Crop around the USB connector in this saved image.
    crop = image[145:270, 375:485]
    crop = cv2.resize(crop, None, fx=3, fy=3,
                      interpolation=cv2.INTER_NEAREST)
    cv2.putText(crop, f"radius={radius*1000:.0f} mm", (8, 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
    tiles.append(crop)
    print(radius, "mask pixels:", int(mask.sum()))

cv2.imwrite(p + "threshold_comparison.png", np.hstack(tiles))
